
extern crate clap;
extern crate colored;
extern crate rust_radio;
extern crate dirs;
extern crate serde;

use std::collections::VecDeque;

use clap::{Arg, App};
use colored::*;
use rust_radio::io;
use rust_radio::gnss::channel;
use rust_radio::gnss::pvt;
use rust_radio::gnss::telemetry_decode::gps;
use serde::{Serialize, Deserialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
struct Result {
	prn:usize,
	acq_doppler_hz:i16,
	acq_test_statistic:f64,
	final_doppler_hz:f64,
	subframes:Vec<gps::l1_ca_subframe::Subframe>,
	ecef_positions:Vec<pvt::SatellitePosition>,
}
fn main() {
	let matches = App::new("GPS L1 C/A Subframe Decode")
		.version("0.1.0")
		.author("John Stanford (johnwstanford@gmail.com)")
		.about("Takes IQ samples centered on 1575.42 MHz and produces a GPS fix")
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
		.get_matches();

	let fname:&str = matches.value_of("filename").unwrap();
	let fs = matches.value_of("sample_rate_sps").unwrap().parse().unwrap();

	eprintln!("Decoding {} at {} [samples/sec]", &fname, &fs);

	let mut channels:Vec<(channel::Channel, VecDeque<gps::l1_ca_subframe::Subframe>, Result)> = (1..=32).map(|prn| {
		let channel = channel::new_default_channel(prn, fs, 0.0);
		let result = Result{ prn, acq_doppler_hz: 0, acq_test_statistic: 0.0, final_doppler_hz: 0.0, subframes:Vec::new(), ecef_positions:Vec::new() };
		(channel, VecDeque::new(), result)
	}).collect();

	let mut all_results:Vec<Result> = Vec::new();

	for s in io::file_source_i16_complex(&fname) {
		for (chn, sf_buffer, current_result) in &mut channels {
			match chn.apply(s) {
				channel::ChannelResult::Acquisition{ doppler_hz, test_stat } => {
					if current_result.subframes.len() > 0 {
						all_results.push(current_result.clone());
					}
					current_result.acq_doppler_hz = doppler_hz;
					current_result.acq_test_statistic = test_stat;
					current_result.final_doppler_hz = doppler_hz as f64;
					current_result.subframes.clear();
					current_result.ecef_positions.clear();

					eprintln!("{}", format!("  PRN {}: Acquired at {} [Hz] doppler, {} test statistic, attempting to track", chn.prn, doppler_hz, test_stat).green());
				},
				channel::ChannelResult::Ok(_, sf, _) => {
					// Print subframe to STDERR
					let subframe_str = format!("{:?}", &sf).blue();
					eprintln!("    {}", subframe_str);

					sf_buffer.push_back(sf);
					current_result.subframes.push(sf);
					
					// Limit subframe buffer size to 3
					while sf_buffer.len() > 3 { sf_buffer.pop_front(); }
					if sf_buffer.len() == 3 {
						if let Some(ecef) = pvt::get_ecef(sf_buffer[0], sf_buffer[1], sf_buffer[2]) {
							current_result.ecef_positions.push(ecef)
						}
					}

				},
				channel::ChannelResult::Err(e) => eprintln!("{}", format!("  Error due to {:?}", e).red()),
				_ => {}
			}
		}

	}

	for (_, _, result) in channels {
		if result.subframes.len() > 0 {
			all_results.push(result);
		}
	}

	// This is the only output to STDOUT.  This allows you to pipe the results to a JSON file, but still see the status updates through STDERR as the code runs.
	println!("{}", serde_json::to_string(&all_results).unwrap());

}