
extern crate clap;
extern crate colored;
extern crate dirs;
extern crate nalgebra as na;
extern crate rust_radio;
extern crate rustfft;
extern crate serde;

use clap::{Arg, App};
use colored::*;
use rust_radio::io;
use rust_radio::gnss::gps_l1_ca;
use rust_radio::gnss::common::acquisition;
use rust_radio::gnss::common::acquisition::Acquisition;
use rust_radio::gnss::gps_l1_ca::tracking::algorithm_dev;
use rustfft::num_complex::Complex;

fn main() {

	let matches = App::new("GPS L1 CA Tracking")
		.version("0.1.0")
		.author("John Stanford (johnwstanford@gmail.com)")
		.about("Takes IQ samples centered on 1575.42 MHz and produces tracking results for the L1 CA signal")
		.arg(Arg::with_name("filename")
			.short("f").long("filename")
			.help("Input filename")
			.required(true).takes_value(true))
		.arg(Arg::with_name("sample_rate_sps")
			.short("s").long("sample_rate_sps")
			.takes_value(true).required(true))
		.arg(Arg::with_name("prn")
			.short("p").long("prn")
			.takes_value(true).required(true))
		.arg(Arg::with_name("max_records")
			.short("m").long("max_records")
			.takes_value(true))
		.get_matches();

	// Parse mandatory fields
	let fname:&str       = matches.value_of("filename").unwrap();
	let fs:f64           = matches.value_of("sample_rate_sps").unwrap().parse().unwrap();
	let prn:usize        = matches.value_of("prn").unwrap().parse().unwrap();

	// Parse optional fields
	let opt_max_records:Option<usize> = matches.value_of("max_records").map(|s| s.parse().unwrap() );

	eprintln!("Decoding {} at {} [samples/sec], max_records={:?}", &fname, &fs, &opt_max_records);

	let symbol:Vec<i8> = gps_l1_ca::signal_modulation::prn_int_sampled(prn, fs);
	let mut acq = acquisition::make_acquisition(symbol, fs, prn, 9, 500, 0.0, 0);
	
	let mut trk = algorithm_dev::new_default_tracker(prn, 0.0, fs);
	let mut code_phase:usize = 0;
	let mut all_results:Vec<algorithm_dev::TrackingDebug> = vec![];

	'outer_acq: for s in io::file_source_i16_complex(&fname).map(|(x, idx)| (Complex{ re: x.0 as f64, im: x.1 as f64 }, idx)) {
		acq.provide_sample(s).unwrap();
		match acq.block_for_result(prn) {
			Ok(Some(result)) => {
				eprintln!("PRN {:02}: {:9.2} [Hz], {:6} [chips], {:.8}", prn, result.doppler_hz, result.code_phase, result.test_statistic());
				trk = algorithm_dev::new_default_tracker(prn, result.doppler_hz, fs);
				code_phase = result.code_phase;
				break 'outer_acq;
			},
			_ => {},
		}
	}

	// Open a brand new file
	'outer_trk: for s in io::file_source_i16_complex(&fname).map(|(x, idx)| (Complex{ re: x.0 as f64, im: x.1 as f64 }, idx)).skip(code_phase) {

		trk.apply(s);
		if s.1 % 2000000 == 0 {
			let debug = trk.debug();
			match trk.state {
				algorithm_dev::TrackingState::WaitingForInitialLockStatus => eprintln!("A: WaitingForInitialLockStatus {}", format!("{:9.2} [Hz], {:9.2}", debug.carrier_hz, debug.estimated_snr_coh).yellow()),
				algorithm_dev::TrackingState::WaitingForFirstTransition   => eprintln!("A: WaitingForFirstTransition {}", format!("{:9.2} [Hz], {:9.2}", debug.carrier_hz, debug.estimated_snr_coh).yellow()),
				algorithm_dev::TrackingState::Tracking                    => eprintln!("A: Tracking {}", format!("{:9.2} [Hz], {:9.2}", debug.carrier_hz, debug.estimated_snr_coh).green()),
				algorithm_dev::TrackingState::LostLock                    => break 'outer_trk,
			}
			all_results.push(debug);
			if let Some(max_records) = opt_max_records {
				if all_results.len() >= max_records { break 'outer_trk; }
			}
		}

	}

	// Output data in JSON format
	println!("{}", serde_json::to_string_pretty(&all_results).unwrap());

}