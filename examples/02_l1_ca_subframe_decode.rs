
extern crate clap;
extern crate colored;
extern crate dirs;
extern crate nalgebra as na;
extern crate rust_radio;
extern crate rustfft;
extern crate serde;

use std::collections::VecDeque;
use std::fs::File;

use clap::{Arg, App};
use colored::*;
use rust_radio::{io::BufferedSource, Sample};
use rust_radio::gnss::gps_l1_ca::telemetry_decode::subframe::Subframe as SF;
use rust_radio::gnss::gps_l1_ca::channel;
use rustfft::num_complex::Complex;
use serde::{Serialize, Deserialize};

// TODO: make these configurable
const NUM_ACTIVE_CHANNELS:usize = 7;

#[derive(Debug, Serialize, Deserialize)]
struct SubframeWithMetadata {
	subframe: SF,
	sample_idx: usize,
	acq_carrier_hz:f64,
	acq_test_stat:f64,
	trk_carrier_hz:f64,
	trk_test_stat:f64,
	prn:usize,
}

fn main() {

	let matches = App::new("GPS L1 C/A GPS Subframe Decoder")
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

	#[cfg(debug_assertions)]
	{
		eprintln!("Compiled with DEV profile");
	}

	eprintln!("Decoding {} at {} [samples/sec]", &fname, &fs);

	let mut inactive_channels:VecDeque<channel::DefaultChannel> = (1..=32).map(|prn| channel::new_channel(prn, fs, 0.008)).collect();
	let mut active_channels:VecDeque<channel::DefaultChannel>   = inactive_channels.drain(..NUM_ACTIVE_CHANNELS).collect();

	let mut all_results:Vec<SubframeWithMetadata> = Vec::new();

	let src:BufferedSource<File, (i16, i16)> = BufferedSource::new(File::open(&fname).unwrap()).unwrap();
	for s in src.map(|(x, idx)| Sample{ val: Complex{ re: x.0 as f64, im: x.1 as f64 }, idx }) {

		for chn in &mut active_channels {
			match chn.apply(&s) {
				channel::ChannelResult::Acquisition{ doppler_hz, doppler_step_hz, test_stat } => {
					eprintln!("{}", format!("PRN {}: Acquired at {:.1} +/- {:.1} [Hz] doppler, {} test statistic, attempting to track", chn.prn, doppler_hz, 0.5*doppler_step_hz, test_stat).green());
				},
				channel::ChannelResult::Ok{sf:Some(subframe), ..} => {
		
					eprintln!("New Subframe: {}", format!("{:?}", subframe).blue());
					let sf_with_metadata = SubframeWithMetadata{ subframe, sample_idx: s.idx,
						acq_carrier_hz:    chn.last_acq_doppler(),
						acq_test_stat:     chn.last_acq_test_stat(),
						trk_carrier_hz:    chn.carrier_freq_hz(), 
						trk_test_stat:     chn.test_stat(),
						prn:               chn.prn };
					all_results.push(sf_with_metadata);
				},
				channel::ChannelResult::Err(e) => eprintln!("{}", format!("PRN {}: Error due to {:?}", chn.prn, e).red()),
				_ => {}
			}
		}

		// Every 0.1 sec, move channels without a signal lock to the inactive buffer and replace them with new ones
		if (s.idx % (fs as usize / 10) == 0) && (s.idx > 0) {
			for _ in 0..NUM_ACTIVE_CHANNELS {
				let this_channel = active_channels.pop_front().unwrap();
				if this_channel.state() == channel::track_and_tlm::ChannelState::AwaitingAcquisition {
					// Move this channel to inactive and replace it
					let replacement_channel = inactive_channels.pop_front().unwrap();
					eprintln!("{:.1} [sec]: Putting PRN {} in the inactive buffer, replacing with PRN {}", (s.idx as f64)/fs, this_channel.prn, replacement_channel.prn);
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

	// Output data in JSON format
	println!("{}", serde_json::to_string_pretty(&all_results).unwrap());

}