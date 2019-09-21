
extern crate byteorder;
extern crate clap;
extern crate colored;
extern crate rust_radio;
extern crate dirs;
extern crate serde;

use clap::{Arg, App};
use colored::*;
use rust_radio::io;
use rust_radio::gnss::{acquisition, tracking};
use rust_radio::gnss::gps::l1_ca_signal;
use rust_radio::gnss::telemetry_decode::gps;
use rust_radio::utils;
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
		let mut acq = acquisition::make_acquisition(&symbol, fs, 50, 10000, 0.008);
		let mut acq_samples_so_far:usize = 1;

		while let Some((x, _)) = signal.next() {
			acq_samples_so_far += 1;

			if let Some(r) = acq.apply(x) {
				// We've acquired a satellite and we'll stay in this block until we run out of data or loose the lock
				eprintln!("{}", format!("  PRN {}: Acquired at {} [Hz] doppler, {} test statistic, attempting to track", prn, r.doppler_hz, r.test_statistic).green());

				// Drop the number of samples required to get us to the start of the next PRN symbol
				signal.drop(r.code_phase);
				
				// Create a new tracker to track the signal we just acquired
				let mut trk = tracking::new_default_tracker(prn, r.doppler_hz as f64, fs, 40.0, 4.0);

				// Create a GPS L1 C/A telemetry decoder and a vector to store the decoded subframes
				let mut tlm = gps::TelemetryDecoder::new();
				let mut nav_data:Vec<(String, gps::l1_ca_subframe::Subframe, usize)> = vec![];

				while let Some(sample) = signal.next() {
					// While we have samples available in the signal

					match trk.apply(sample) {
						Ok(Some((bit, bit_idx))) => {
							// While the tracker still has a lock and keeps producing prompt values, pass them into the telemetry decoder and match on the result
							match tlm.apply((bit.re > 0.0, bit_idx)) {
								Ok(Some((subframe, start_idx))) => {
									// The telemetry decoder successfully decoded a subframe, but we just have a sequence of bits right now.  We need to interpret them.
									if let Ok(sf) = gps::l1_ca_subframe::decode(subframe) {
										// The bits of this subframe have been successfully interpreted.  Output the results to STDERR and store them in nav_data
										let bytes:Vec<String> = utils::bool_slice_to_byte_vec(&subframe, true).iter().map(|b| format!("{:02X}", b)).collect();
										let subframe_str = format!("{:?}", sf).blue();
										eprintln!("    {}", subframe_str);
										eprintln!("    Hex: {}", bytes.join(""));

										nav_data.push((bytes.join(""), sf, start_idx));
									}
									else { 
										eprintln!("    Invalid subframe");
									}
								},
								Ok(None) => {
									// The telemetry decoder hasn't produced a new subframe yet, but everything is still okay, so do nothing
								},
								Err(e) => {
									// The telemetry decoder somehow found invalid data, so report a loss of lock and break out of this inner while loop
									if acq_samples_so_far > acq_samples_to_try { break; }
									eprintln!("{}", format!("  Loss of lock due to {:?}, {} of {}", e, acq_samples_so_far, acq_samples_to_try).red());
									break;
								}
							}
						}
						Ok(None) => {
							// We don't have a new bit available, but we also haven't lost the lock, so do nothing
						},
						Err(e) => {
							// The tracking block reported a loss of lock
							eprintln!("{}", format!("  Loss of lock due to {:?}, {} of {}", e, acq_samples_so_far, acq_samples_to_try).red());
							break;
						}
					}
				}

				// Store the results of this acquisition.  At a minimum, we have the acquisition itself to report.  We may or may not also have subframes.
				let this_result = Result{ prn, acq_doppler_hz: r.doppler_hz, acq_test_statistic: r.test_statistic, final_doppler_hz: trk.carrier_freq_hz(), nav_data };
				all_results.push(this_result);
			}

			if acq_samples_so_far > acq_samples_to_try { 
				break; 
			}
			if acq_samples_so_far%2000 == 0 {
				eprintln!("{}", format!("  No acquisition, {} of {} samples", acq_samples_so_far, acq_samples_to_try).red());
			}
		}

	}

	// This is the only output to STDOUT.  This allows you to pipe the results to a JSON file, but still see the status updates through STDERR as the code runs.
	println!("{}", serde_json::to_string(&all_results).unwrap());
}