
extern crate clap;
extern crate colored;
extern crate dirs;
extern crate nalgebra as na;
extern crate rust_radio;
extern crate rustfft;
extern crate serde;

use std::collections::VecDeque;

use clap::{Arg, App};
use colored::*;
use rust_radio::io;
use rust_radio::gnss::{channel, acquisition::fast_pcps};
use rust_radio::gnss::telemetry_decode::gps::l1_ca_subframe::Subframe as SF;
use rustfft::num_complex::Complex;
use serde::{Serialize, Deserialize};

#[derive(Debug, Serialize, Deserialize)]
struct SubframeWithMetadata {
	subframe: SF,
	carrier_freq_hz:f64,
	cn0_snv_db_hz:f64,
	carrier_lock_test:f64,
	acq_test_stat:f64,
	prn:usize,
	snr:f64,
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

	eprintln!("Decoding {} at {} [samples/sec]", &fname, &fs);

	let mut all_channels:VecDeque<channel::Channel<fast_pcps::Acquisition>> = (1..=32).map(|prn| channel::new_channel(prn, fs, 0.01)).collect();
	let mut all_results:Vec<SubframeWithMetadata> = Vec::new();

	for s in io::file_source_i16_complex(&fname).map(|(x, idx)| (Complex{ re: x.0 as f64, im: x.1 as f64 }, idx)) {

		for chn in &mut all_channels {
			match chn.apply(s) {
				channel::ChannelResult::Acquisition{ doppler_hz, test_stat } => {
					eprintln!("{}", format!("PRN {}: Acquired at {} [Hz] doppler, {} test statistic, attempting to track", chn.prn, doppler_hz, test_stat).green());
				},
				channel::ChannelResult::Ok{sf:Some(subframe)} => {
		
					eprintln!("New Subframe: {}", format!("{:?}", subframe).blue());
					let sf_with_metadata = SubframeWithMetadata{ subframe, 
						carrier_freq_hz:   chn.carrier_freq_hz(), 
						cn0_snv_db_hz:     chn.last_cn0_snv_db_hz(),
						carrier_lock_test: chn.last_carrier_lock_test(),
						acq_test_stat:     chn.last_acq_test_stat(),
						prn:               chn.prn,
						snr:               chn.estimated_snr() };
					all_results.push(sf_with_metadata);
				},
				channel::ChannelResult::Err(e) => eprintln!("{}", format!("PRN {}: Error due to {:?}", chn.prn, e).red()),
				_ => {}
			}
		}

	}

	// Output data in JSON format
	println!("{}", serde_json::to_string_pretty(&all_results).unwrap());

}