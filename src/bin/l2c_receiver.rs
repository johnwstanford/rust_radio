
extern crate clap;
extern crate colored;
extern crate dirs;
extern crate nalgebra as na;
extern crate rust_radio;
extern crate rustfft;
extern crate serde;

use clap::{Arg, App};
use colored::*;
use rust_radio::{io, Sample};
use rust_radio::gnss::common::acquisition::{Acquisition, two_stage_pcps};
use rust_radio::gnss::gps_l2c::{signal_modulation, tracking, L2_CM_PERIOD_SEC};
use rustfft::num_complex::Complex;

#[derive(Debug)]
enum ChannelState {
	Acquisition,
	Tracking,
}

fn main() {

	let matches = App::new("GPS L2C Receiver")
		.version("0.1.0")
		.author("John Stanford (johnwstanford@gmail.com)")
		.about("Takes IQ samples centered on 1227.6 MHz and produces subframes and fixes for the L2C signal")
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

	let prn = 15;

	// Just track one SV for now and create channels in parallel once we know this works
	let mut acq: two_stage_pcps::Acquisition = {
		// Create CM code and resample
		let cm_code:[bool; 10230] = signal_modulation::cm_code(prn);
		let n_samples:usize = (fs * L2_CM_PERIOD_SEC as f64) as usize;		// [samples/sec] * [sec]
		let mut symbol_cm:Vec<i8> = vec![];
		for sample_idx in 0..n_samples {
			let chip_idx_f64:f64 = sample_idx as f64 * (10230.0 / n_samples as f64);
			if chip_idx_f64 - chip_idx_f64.floor() < 0.5 {
				if cm_code[chip_idx_f64.floor() as usize] { symbol_cm.push(1) } else { symbol_cm.push(-1) }
			} else {
				symbol_cm.push(0)
			}
		}

		two_stage_pcps::Acquisition::new(symbol_cm, fs, prn, 140, 2, 10.0, 0.001, 0)
	};

	let mut trk = tracking::new_default_tracker(prn, 0.0, fs);
	let mut state = ChannelState::Acquisition;

	for s in io::file_source_i16_complex(&fname).map(|(x, idx)| Sample{ val: Complex{ re: x.0 as f64, im: x.1 as f64 }, idx}) {

		let opt_next_state:Option<ChannelState> = match state {
			ChannelState::Acquisition => {
				acq.provide_sample(&s).unwrap();

				match acq.block_for_result() {
					Ok(Some(result)) => {
						eprintln!("PRN {:02} {}", prn, 
							format!("CM: {:7.1} +/- {:7.1} [Hz], {:6} [chips], {:.8}", result.doppler_hz, result.doppler_step_hz, result.code_phase, result.test_statistic()).yellow());

						trk.initialize(result.doppler_hz);
						Some(ChannelState::Tracking)
					},
					Err(msg) => {
						eprintln!("PRN {}: Error, {}", prn, msg);
						None
					},
					Ok(None) => None
				}

			},
			ChannelState::Tracking => {
				eprintln!("Tracking...");

				None
			}
		};

		if let Some(next_state) = opt_next_state {
			state = next_state;
		}


	}

}