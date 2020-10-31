
extern crate clap;
extern crate colored;
extern crate dirs;
extern crate nalgebra as na;
extern crate rust_radio;
extern crate rustfft;
extern crate serde;

use std::fs::File;

use clap::{Arg, App};
use colored::*;
use rustfft::num_complex::Complex;

use rust_radio::block::{BlockFunctionality, BlockResult};
use rust_radio::block::block_tree_sync_static::acquire_and_track::AcquireAndTrack;
use rust_radio::block::block_tree_sync_static::split_and_merge::SplitAndMerge;
use rust_radio::{io::BufferedSource, Sample};
use rust_radio::gnss::common::acquisition;
use rust_radio::gnss::gps_l1_ca::tracking::TrackReport;
use rust_radio::gnss::gps_l1_ca::{self, tracking::algorithm_standard};

fn main() -> Result<(), &'static str> {

	let matches = App::new("GPS L1 CA Tracking")
		.version("0.1.0")
		.author("John Stanford (johnwstanford@gmail.com)")
		.about("Takes IQ samples centered on 1575.42 MHz and produces acquisition results for the L1 CA signal")
		.arg(Arg::with_name("filename")
			.short("f").long("filename")
			.help("Input filename")
			.required(true).takes_value(true))
		.arg(Arg::with_name("input_type")
			.short("t").long("type")
			.takes_value(true)
			.possible_value("i16"))
		.arg(Arg::with_name("sample_rate_sps")
			.short("s").long("sample_rate_sps")
			.takes_value(true).required(true))
		.arg(Arg::with_name("max_records")
			.short("m").long("max_records")
			.takes_value(true))
		.get_matches();

	let fname:&str = matches.value_of("filename").unwrap();
	let src:BufferedSource<File, (i16, i16)> = BufferedSource::new(File::open(&fname).unwrap()).unwrap();

	let fs = matches.value_of("sample_rate_sps").unwrap().parse().unwrap();
	let opt_max_records:Option<usize> = matches.value_of("max_records").map(|s| s.parse().unwrap() );

	eprintln!("Decoding {} at {} [samples/sec], max_records={:?}", &fname, &fs, &opt_max_records);

	let mut sam = SplitAndMerge::from_iter((1..=32).map( |prn| {

		let symbol:Vec<i8> = gps_l1_ca::signal_modulation::prn_int_sampled(prn, fs);
		let acq = acquisition::make_acquisition(symbol, fs, prn, 9, 17, 0.008, 0);

		let trk = algorithm_standard::new_default_tracker(prn, 0.0, fs);

		AcquireAndTrack::new(acq, trk)

	}));

	let mut all_records:Vec<TrackReport> = vec![];

	'outer: for s in src.map(|(x, idx)| Sample{ val: Complex{ re: x.0 as f64, im: x.1 as f64 }, idx }) {

		match sam.apply(&s) {
			BlockResult::Ready(report) => {
				let s =	format!("{:6.2} [sec], PRN {:02}, test_stat={:.5}, {:.3e}", 
					(report.sample_idx as f64)/fs, report.prn, report.test_stat, report.prompt_i);
				if      report.test_stat > 0.04 { eprintln!("{}", s.green());  }
				else if report.test_stat > 0.02 { eprintln!("{}", s.yellow()); }
				else                            { eprintln!("{}", s.red());    }

				all_records.push(report);
			},
			_ => ()
		}

		if let Some(max_records) = opt_max_records {
			if all_records.len() >= max_records { break 'outer; }
		}

	}

	// Output data in JSON format
	println!("{}", serde_json::to_string_pretty(&all_records).unwrap());

	Ok(())

}