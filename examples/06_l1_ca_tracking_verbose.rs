
extern crate nalgebra as na;

use std::collections::HashSet;
use std::ffi::CString;
use std::fs::File;
use std::io::BufReader;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use clap::{Arg, App};
use colored::*;
use rustfft::num_complex::Complex;

use uhd_rs::types::{TuneRequest, TuneRequestPolicy};
use uhd_rs::usrp::USRP;

use rust_radio::block::{BlockFunctionality, BlockResult};
use rust_radio::block::block_tree_sync_static::acquire_and_track::AcquireAndTrack;
use rust_radio::block::block_tree_sync_static::split_and_merge::RotatingSplitAndMerge;
use rust_radio::{io::BufferedSource, Sample};
use rust_radio::gnss::common::acquisition::two_stage_pcps;
use rust_radio::gnss::common::tracking::TrackReport;
use rust_radio::gnss::gps_l1_ca;
use rust_radio::gnss::gps_l1_ca::tracking::{self, TrackingDebug};

#[tokio::main]
pub async fn main() -> Result<(), &'static str> {

    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

  	let matches = App::new("GPS L1 CA Tracking")
		.version("0.1.0")
		.author("John Stanford (johnwstanford@gmail.com)")
		.about("Takes IQ samples centered on 1575.42 MHz and produces acquisition results for the L1 CA signal")
		.arg(Arg::with_name("filename")
			.long("filename")
			.help("Input filename")
			.takes_value(true))
		.arg(Arg::with_name("usrp")
			.long("usrp")
			.help("USRP device arguments; can be an empty string")
			.takes_value(true))
		.arg(Arg::with_name("input_type")
			.short("t").long("type")
			.takes_value(true)
			.possible_value("i16"))
		.arg(Arg::with_name("sample_rate_sps")
			.short("s").long("sample_rate_sps")
			.takes_value(true))
		.arg(Arg::with_name("json_prompt_out")
			.long("json_prompt_out")
			.help("JSON filename where prompt information should be output")
			.takes_value(true))
		.arg(Arg::with_name("json_verbose_idx_in")
			.long("json_verbose_idx_in")
			.help("JSON file containing a list of sample indices tuples where verbose tracking information should be output")
			.takes_value(true))
		.arg(Arg::with_name("json_verbose_idx_out")
			.long("json_verbose_idx_out")
			.help("JSON filename where verbose tracking information should be output")
			.takes_value(true))
		.get_matches();

	let fs = matches.value_of("sample_rate_sps").unwrap_or("2e6").parse().unwrap();

	let verbose_indexes:HashSet<usize> = match matches.value_of("json_verbose_idx_in") {
		Some(fname) => {
			let f = File::open(fname).map_err(|_| "Unable to open JSON-formatted verbose index list")?;
			serde_json::from_reader(BufReader::new(f)).map_err(|_| "Unable to parse JSON")?
		},
		None => HashSet::new()
	};

    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    }).expect("Error setting Ctrl-C handler");

   	let src:Box<dyn Iterator<Item = ((i16, i16), usize)>> = match (matches.value_of("filename"), matches.value_of("usrp")) {
		(Some(fname), _) => {
			eprintln!("Decoding {} at {} [samples/sec]", &fname, &fs);
			Box::new(BufferedSource::new(File::open(&fname).unwrap()).unwrap())
		},
		(_, opt_usrp_args) => {
		
			// If USRP arguments weren't provided, just use an empty string	
			let usrp_args:&str = opt_usrp_args.unwrap_or("");

			eprintln!("Creating USRP device with args {:?}", &usrp_args);
			let mut usrp = USRP::new(usrp_args)?;

			let tune_args = CString::new("").unwrap();

			let tune_request = TuneRequest {
			    target_freq:    1575.42e6,					// Target frequency for RF chain in Hz
			    rf_freq_policy: TuneRequestPolicy::Auto, 	// RF frequency policy
			    rf_freq: 		0.0,						// RF frequency in Hz
			    dsp_freq_policy:TuneRequestPolicy::Auto, 	// DSP frequency policy
			    dsp_freq:		0.0,						// DSP frequency in Hz
			    args:tune_args.as_ptr()						// Key-value pairs delimited by commas		
			};

			// TODO: find the max gain by querying the device; it might not always be 76 [dB]
			usrp.set_rx_gain(76.0, 0, "")?;	
			usrp.set_rx_rate(fs, 0)?;
			
			let tune_result = usrp.set_rx_freq(&tune_request, 0)?;
			eprintln!("Tune Result: {:#?}", tune_result);

			let rx_streamer = usrp.start_continuous_stream::<i16, i16>("")?;
			Box::new(BufferedSource::new(rx_streamer).unwrap())
		}
	};

	let mut sam = RotatingSplitAndMerge::from_iter((1..=32).map( |prn| {

		let symbol:Vec<i8> = gps_l1_ca::signal_modulation::prn_int_sampled(prn, fs);
		let acq = two_stage_pcps::Acquisition::new(symbol.into_iter().map(|x| Complex{ re: x as f64, im: 0.0 }).collect(), fs, prn, 9, 3, 50.0, 0.008, 8);

		let trk = tracking::new_2nd_order_tracker(prn, 0.0, fs, 0.0, 0.0);

		AcquireAndTrack::new(acq, trk)

	}), 200_000, None);

	let mut all_records:Vec<TrackReport> = vec![];
	let mut all_verbose:Vec<(usize, Vec<TrackingDebug>)> = vec![];

	'outer: for s in src.map(|(x, idx)| Sample{ val: Complex{ re: x.0 as f64, im: x.1 as f64 }, idx }) {

		if verbose_indexes.contains(&s.idx) {

			let mut this_idx:Vec<TrackingDebug> = vec![];
			for (tracking, aat) in &sam.blocks {
				if *tracking {
					this_idx.push(aat.trk.debug());
				}
			}
			all_verbose.push((s.idx, this_idx));
		}

		let blk_result:BlockResult<TrackReport> = sam.apply(&s);
		match blk_result {
			BlockResult::Ready(report) => {
				let s =	format!("{:6.2} [sec], PRN {:02}, test_stat={:.5}, {:6.2} [kHz], {:.3e}", 
					(report.sample_idx as f64)/fs, report.id, report.test_stat, report.freq_hz*1.0e-3, report.prompt_i);
				if      report.test_stat > 0.04 { eprintln!("{}", s.green());  }
				else if report.test_stat > 0.02 { eprintln!("{}", s.yellow()); }
				else                            { eprintln!("{}", s.red());    }

				all_records.push(report);
			},
			_ => ()
		}

		// Break out of this loop if SIGINT is detected (Ctrl-C)
		if !running.load(Ordering::SeqCst) { break 'outer; }

	}

	// Output data in JSON format
	if let Some(outfile) = matches.value_of("json_prompt_out") {
		std::fs::write(outfile, serde_json::to_string_pretty(&all_records).unwrap().as_bytes()).map_err(|_| "Unable to write prompt JSON")?;
	}

	if let Some(outfile) = matches.value_of("json_verbose_idx_out") {
		std::fs::write(outfile, serde_json::to_string_pretty(&all_verbose).unwrap().as_bytes()).map_err(|_| "Unable to write verbose JSON")?;
	}

	Ok(())

}