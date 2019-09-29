
extern crate byteorder;
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
use rust_radio::gnss::telemetry_decode::gps;
use serde::{Serialize, Deserialize};

const NUM_ACTIVE_CHANNELS:usize = 7;

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
		.arg(Arg::with_name("sample_rate_sps")
			.short("s").long("sample_rate_sps")
			.takes_value(true).required(true))
		.get_matches();

	let fname:&str = matches.value_of("filename").unwrap();
	let fs = matches.value_of("sample_rate_sps").unwrap().parse().unwrap();

	eprintln!("Decoding {} at {} [samples/sec]", &fname, &fs);

	let mut inactive_channels:VecDeque<(channel::Channel, Result, Vec<Result>)> = (1..=32).map(|prn| {
		let channel = channel::new_channel(prn, fs, 0.0, 0.01);
		let result = Result{ prn, acq_doppler_hz:0, acq_test_statistic:0.0, final_doppler_hz:0.0, nav_data:Vec::new() };
		(channel, result, Vec::new())
	}).collect();

	let mut active_channels:VecDeque<(channel::Channel, Result, Vec<Result>)> = inactive_channels.drain(..NUM_ACTIVE_CHANNELS).collect();

	for s in io::file_source_i16_complex(&fname) {
		for (chn, current_result, result_buffer) in &mut active_channels {
			match chn.apply(s) {
				channel::ChannelResult::Acquisition{ doppler_hz, test_stat } => {
					// If we have subframes from the previous acquisition, commit them to the buffer
					if current_result.nav_data.len() > 0 {
						let result_copy = Result{ prn: current_result.prn, 
							acq_doppler_hz: current_result.acq_doppler_hz, 
							acq_test_statistic: current_result.acq_test_statistic, 
							final_doppler_hz: current_result.final_doppler_hz, 
							nav_data: current_result.nav_data.drain(..).collect() };
						result_buffer.push(result_copy);
					}

					current_result.acq_doppler_hz = doppler_hz;
					current_result.acq_test_statistic = test_stat;
					current_result.final_doppler_hz = doppler_hz as f64;

					eprintln!("{}", format!("  PRN {}: Acquired at {} [Hz] doppler, {} test statistic, attempting to track", chn.prn, doppler_hz, test_stat).green());
				},
				channel::ChannelResult::Ok(hex, sf, start_idx) => {
					let subframe_str = format!("{:?}", sf).blue();
					eprintln!("    {}", subframe_str);
					current_result.nav_data.push((hex, sf, start_idx));
					current_result.final_doppler_hz = chn.carrier_freq_hz();
				},
				channel::ChannelResult::Err(e) => eprintln!("{}", format!("  Error due to {:?}", e).red()),
				_ => {}
			}
		}

		// Every 0.1 sec, move channels without a signal lock to the inactive buffer and replace them with new ones
		if (s.1 % (fs as usize / 10) == 0) && (s.1 > 0) {
			for _ in 0..NUM_ACTIVE_CHANNELS {
				let this_channel = active_channels.pop_front().unwrap();
				if this_channel.0.state() == channel::ChannelState::Acquisition {
					// Move this channel to inactive and replace it
					let replacement_channel = inactive_channels.pop_front().unwrap();
					eprintln!("{:.1} [sec]: Putting PRN {} in the inactive buffer, replacing with PRN {}", (s.1 as f64)/fs, this_channel.0.prn, replacement_channel.0.prn);
					inactive_channels.push_back(this_channel);
					active_channels.push_back(replacement_channel);
				} else {
					// Keep this channel in the active buffer
					active_channels.push_back(this_channel);
				}
			}
			assert!(active_channels.len() == NUM_ACTIVE_CHANNELS);
			assert!(inactive_channels.len() == (32 - NUM_ACTIVE_CHANNELS));
		}

	}

	let mut all_results:Vec<Result> = Vec::new();

	// Move all channels to the active buffer, then collect all results from the active buffer
	while let Some(chn_tuple) = inactive_channels.pop_front() {
		active_channels.push_back(chn_tuple);
	}
	for (_, current_result, result_buffer) in active_channels {
		for result in result_buffer {
			all_results.push(result);
		}
		if current_result.nav_data.len() > 0 {
			all_results.push(current_result);
		}
	}

	// This is the only output to STDOUT.  This allows you to pipe the results to a JSON file, but still see the status updates through STDERR as the code runs.
	println!("{}", serde_json::to_string(&all_results).unwrap());
}