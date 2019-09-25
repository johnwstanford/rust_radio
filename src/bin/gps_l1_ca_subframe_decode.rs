
extern crate byteorder;
extern crate clap;
extern crate colored;
extern crate rust_radio;
extern crate dirs;
extern crate serde;

use clap::{Arg, App};
use colored::*;
use rust_radio::io;
use rust_radio::gnss::{acquisition, channel};
use rust_radio::gnss::gps::l1_ca_signal;
use rust_radio::gnss::telemetry_decode::gps;
use serde::{Serialize, Deserialize};

const DEFAULT_ACQ_SAMPLES_TO_TRY:usize = 200_000;

#[derive(Serialize, Deserialize, Debug)]
struct Result {
	prn:usize,
	acq_doppler_hz:i16,
	acq_test_statistic:f64,
	final_doppler_hz:f64,
	nav_data:Vec<(String, gps::l1_ca_subframe::Subframe, usize)>,
}
fn main() {
	let matches = App::new("GPS L1 C/A Subframe Decode")
		.version("0.1.0")
		.author("John Stanford (johnwstanford@gmail.com)")
		.about("Takes IQ samples centered on 1575.42 MHz and decodes the GPS L1 C/A navigation messages by subframe")
		.arg(Arg::with_name("filename")
			.short("f").long("filename")
			.help("Input filename")
			.required(true).takes_value(true))
		.arg(Arg::with_name("input_type")
			.short("t").long("type")
			.takes_value(true)
			.possible_value("i16"))
		.arg(Arg::with_name("acq_samples_to_try")
			.short("a").long("acq_samples_to_try")
			.help("Defaults to 2e5")
			.takes_value(true))
		.arg(Arg::with_name("sample_rate_sps")
			.short("s").long("sample_rate_sps")
			.takes_value(true).required(true))
		.get_matches();

	let fname:&str = matches.value_of("filename").unwrap();
	let fs = matches.value_of("sample_rate_sps").unwrap().parse().unwrap();
	let acq_samples_to_try:usize = match matches.value_of("acq_samples_to_try") {
		Some(n) => n.parse().unwrap(),
		None => DEFAULT_ACQ_SAMPLES_TO_TRY,
	};

	eprintln!("Decoding {} at {} [samples/sec]", &fname, &fs);
	let mut all_results:Vec<Result> = vec![];

	for prn in 1..=32 {
		eprintln!("  PRN {}: Searching...", prn);
		let mut signal = io::file_source_i16_complex(&fname);
		let symbol:Vec<i8> = l1_ca_signal::prn_int_sampled(prn, fs);
		let mut acq = acquisition::make_acquisition(symbol, fs, 50, 10000, 0.008);
		let mut acq_samples_so_far:usize = 1;

		while let Some((x, _)) = signal.next() {
			acq_samples_so_far += 1;

			if let Some(r) = acq.apply(x) {
				// We've acquired a satellite and we'll stay in this block until we run out of data or loose the lock
				eprintln!("{}", format!("  PRN {}: Acquired at {} [Hz] doppler, {} test statistic, attempting to track", prn, r.doppler_hz, r.test_statistic).green());

				// Drop the number of samples required to get us to the start of the next PRN symbol
				signal.drop(r.code_phase);
				
				// Create a new channel to track the signal and decode the subframes
				let mut chn = channel::new_default_channel(prn, fs, r.doppler_hz as f64);
				let mut nav_data:Vec<(String, gps::l1_ca_subframe::Subframe, usize)> = vec![];

				while let Some(sample) = signal.next() {
					// While we have samples available in the signal

					match chn.apply(sample) {
						channel::ChannelResult::Ok(hex, sf, start_idx) => {
							let subframe_str = format!("{:?}", sf).blue();
							eprintln!("    {}", subframe_str);
							nav_data.push((hex, sf, start_idx))
						},
						channel::ChannelResult::NotReady => {},
						channel::ChannelResult::Err(e) => {
							// The tracking block reported a loss of lock
							eprintln!("{}", format!("  Loss of lock due to {:?}, {} of {}", e, acq_samples_so_far, acq_samples_to_try).red());
							break;
						}
					}
				}

				// Store the results of this acquisition if subframes were found
				if nav_data.len() > 0 {
					let this_result = Result{ prn, acq_doppler_hz: r.doppler_hz, acq_test_statistic: r.test_statistic, final_doppler_hz: chn.carrier_freq_hz(), nav_data };
					all_results.push(this_result);
				}
			}

			if acq_samples_so_far > acq_samples_to_try { 
				break; 
			}
			if acq_samples_so_far%20000 == 0 {
				eprintln!("{}", format!("  No acquisition, {} of {} samples", acq_samples_so_far, acq_samples_to_try).red());
			}
		}

	}

	// This is the only output to STDOUT.  This allows you to pipe the results to a JSON file, but still see the status updates through STDERR as the code runs.
	println!("{}", serde_json::to_string(&all_results).unwrap());
}