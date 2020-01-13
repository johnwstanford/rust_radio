
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
use rust_radio::gnss::gps_l1_ca::tracking::algorithm_a;
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
		.arg(Arg::with_name("acq_freq_hz")
			.short("h").long("acq_freq_hz")
			.takes_value(true).required(true))
		.arg(Arg::with_name("code_phase")
			.short("c").long("code_phase")
			.takes_value(true).required(true))
		.arg(Arg::with_name("max_records")
			.short("m").long("max_records")
			.takes_value(true))
		.get_matches();

	// Parse mandatory fields
	let fname:&str       = matches.value_of("filename").unwrap();
	let fs:f64           = matches.value_of("sample_rate_sps").unwrap().parse().unwrap();
	let prn:usize        = matches.value_of("prn").unwrap().parse().unwrap();
	let acq_freq_hz:f64  = matches.value_of("acq_freq_hz").unwrap().parse().unwrap();
	let code_phase:usize = matches.value_of("code_phase").unwrap().parse().unwrap();

	// Parse optional fields
	let opt_max_records:Option<usize> = matches.value_of("max_records").map(|s| s.parse().unwrap() );

	eprintln!("Decoding {} at {} [samples/sec], max_records={:?}", &fname, &fs, &opt_max_records);

	let mut trk = algorithm_a::new_default_tracker(prn, acq_freq_hz, fs);
	let mut all_results:Vec<algorithm_a::TrackingDebug> = vec![];

	'outer: for s in io::file_source_i16_complex(&fname).map(|(x, idx)| (Complex{ re: x.0 as f64, im: x.1 as f64 }, idx)).skip(code_phase) {

		trk.apply(s);
		if s.1 % 2000000 == 0 {
			let debug = trk.debug();
			match trk.state {
				algorithm_a::TrackingState::WaitingForInitialLockStatus => eprintln!("WaitingForInitialLockStatus {}", format!("{:9.2} [Hz], {:.8}", debug.carrier_hz, debug.test_stat).yellow()),
				algorithm_a::TrackingState::WaitingForFirstTransition   => eprintln!("WaitingForFirstTransition {}", format!("{:9.2} [Hz], {:.8}", debug.carrier_hz, debug.test_stat).yellow()),
				algorithm_a::TrackingState::Tracking                    => eprintln!("Tracking {}", format!("{:9.2} [Hz], {:.8}", debug.carrier_hz, debug.test_stat).green()),
				algorithm_a::TrackingState::LostLock                    => eprintln!("LostLock {}", format!("{:9.2} [Hz], {:.8}", debug.carrier_hz, debug.test_stat).red()),
			}
			all_results.push(debug);
			if let Some(max_records) = opt_max_records {
				if all_results.len() >= max_records { break 'outer; }
			}
		}

	}

	// Output data in JSON format
	println!("{}", serde_json::to_string_pretty(&all_results).unwrap());

}