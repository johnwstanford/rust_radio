
extern crate clap;
extern crate colored;
extern crate rust_radio;
extern crate dirs;
extern crate serde;
extern crate nalgebra as na;

use std::collections::VecDeque;

use clap::{Arg, App};
use colored::*;
use rust_radio::io;
use rust_radio::gnss::channel;

// TODO: make these configurable
const NUM_ACTIVE_CHANNELS:usize = 7;

fn main() {

	let matches = App::new("GPS L1 C/A GPS Receiver")
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
	let mut opt_t0:Option<f64> = None;

	eprintln!("Decoding {} at {} [samples/sec]", &fname, &fs);

	let mut inactive_channels:VecDeque<channel::Channel> = (1..=32).map(|prn| channel::new_channel(prn, fs, 0.0, 0.01)).collect();
	let mut active_channels:VecDeque<channel::Channel>   = inactive_channels.drain(..NUM_ACTIVE_CHANNELS).collect();

	let mut all_results:Vec<channel::ChannelObservation> = Vec::new();
	let pvt_rate_samples:usize = (fs * 0.02) as usize;

	for s in io::file_source_i16_complex(&fname) {

		let current_rx_time:f64 = (s.1 as f64) / fs;
		if let Some(ref mut t0) = opt_t0 {
			*t0 += 1.0 / fs;
		}

		for chn in &mut active_channels {
			if (s.1)%pvt_rate_samples == 0 {
				let opt_co = chn.get_observation(current_rx_time - 0.1, opt_t0.unwrap_or(0.0) - 0.1);
				//eprintln!("{:.2} [sec], {} [samples]: PRN {} {:?}", current_rx_time, s.1, chn.prn, &opt_co);						
				if let Some(co) = opt_co {
					all_results.push(co);
				}
			}

			match chn.apply(s) {
				channel::ChannelResult::Acquisition{ doppler_hz, test_stat } =>
					eprintln!("{}", format!("PRN {}: Acquired at {} [Hz] doppler, {} test statistic, attempting to track", chn.prn, doppler_hz, test_stat).green()),
				channel::ChannelResult::Ok{sf:Some(new_sf)} => {
					opt_t0.get_or_insert(new_sf.time_of_week() + 0.086);
					eprintln!("New Subframe: {}", format!("{:?}", new_sf).blue());
				},
				channel::ChannelResult::Err(e) => 
					eprintln!("{}", format!("PRN {}: Error due to {:?}", chn.prn, e).red()),
				_ => {}
			}
		}

		// Every 0.1 sec, move channels without a signal lock to the inactive buffer and replace them with new ones
		if (s.1 % (fs as usize / 10) == 0) && (s.1 > 0) {
			for _ in 0..NUM_ACTIVE_CHANNELS {
				let this_channel = active_channels.pop_front().unwrap();
				if this_channel.state() == channel::ChannelState::Acquisition {
					// Move this channel to inactive and replace it
					let replacement_channel = inactive_channels.pop_front().unwrap();
					eprintln!("{:.1} [sec]: Putting PRN {} in the inactive buffer, replacing with PRN {}", current_rx_time, this_channel.prn, replacement_channel.prn);
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

	// Output data in JSON format for rapid-prototyping positioning
	println!("{}", serde_json::to_string_pretty(&all_results).unwrap());

}