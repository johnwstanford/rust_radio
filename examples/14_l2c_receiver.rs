
extern crate clap;
extern crate colored;
extern crate dirs;
extern crate nalgebra as na;
extern crate rust_radio;
extern crate rustfft;
extern crate serde;

use std::fs::File;

use clap::{Arg, App};
use colored::*;
use rust_radio::{io::BufferedSource, Sample};
use rust_radio::gnss::common::acquisition::{two_stage_pcps, basic_pcps};
use rust_radio::gnss::gps_l2c::{signal_modulation, L2_CM_PERIOD_SEC, L2_CL_PERIOD_SEC};
use rust_radio::gnss::gps_l2c::tracking_cl::{self, TrackingResult};
use rustfft::num_complex::Complex;

const MAX_ACQ_TRIES_SAMPLES:usize = 2000000;

#[derive(Debug)]
enum ChannelState {
	AcquisitionCM(usize),
	AcquisitionCL(usize),
	PullIn(usize),
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
		.arg(Arg::with_name("prn")
			.long("prn")
			.takes_value(true).required(true))
		.get_matches();

	let fname:&str = matches.value_of("filename").unwrap();
	let fs = matches.value_of("sample_rate_sps").unwrap().parse().unwrap();

	eprintln!("Decoding {} at {} [samples/sec]", &fname, &fs);

	let prn:usize = matches.value_of("prn").unwrap().parse().unwrap();

	// Just track one SV for now and create channels in parallel once we know this works
	let mut acq_cm: two_stage_pcps::Acquisition = {
		// Create CM code and resample
		let cm_code:[bool; 10230] = signal_modulation::cm_code(prn);
		let n_samples:usize = (fs * L2_CM_PERIOD_SEC as f64) as usize;		// [samples/sec] * [sec]
		let mut symbol_cm:Vec<i8> = vec![];
		for sample_idx in 0..n_samples {
			let chip_idx_f64:f64 = sample_idx as f64 * (10230.0 / n_samples as f64);
			if chip_idx_f64 - chip_idx_f64.floor() < 0.5 {
				symbol_cm.push(0)
			} else {
				if cm_code[chip_idx_f64.floor() as usize] { symbol_cm.push(1) } else { symbol_cm.push(-1) }
			}
		}

		two_stage_pcps::Acquisition::new(symbol_cm.into_iter().map(|x| Complex{ re: x as f64, im: 0.0 }).collect(), fs, prn, 140, 2, 1.0, 0.0005, 0)
	};

	let mut acq_cl: basic_pcps::Acquisition = {
		// Create CL code and resample
		let cl_code:[bool; 767250] = signal_modulation::cl_code(prn);
		let n_samples:usize = (fs * L2_CL_PERIOD_SEC as f64) as usize;		// [samples/sec] * [sec]
		let mut symbol_cl:Vec<i8> = vec![];
		for sample_idx in 0..n_samples {
			let chip_idx_f64:f64 = sample_idx as f64 * (767250.0 / n_samples as f64);
			if chip_idx_f64 - chip_idx_f64.floor() < 0.5 {
				if cl_code[chip_idx_f64.floor() as usize] { symbol_cl.push(1) } else { symbol_cl.push(-1) }
			} else {
				symbol_cl.push(0)
			}
		}

		basic_pcps::Acquisition::new(symbol_cl.into_iter().map(|x| Complex{ re: x as f64, im: 0.0 }).collect(), fs, prn, 0.00000075, vec![0.0])
	};

	let mut trk = tracking_cl::new_default_tracker(prn, 0.0, fs);
	let mut state = ChannelState::AcquisitionCM(0);

	let mut bits:Vec<(f64, usize)> = vec![];

	// FEC algorithm described starting on page 35 of IS-GPS-200K
	// Telemetry decoding described starting on page 130 of IS-GPS-200K

	let src:BufferedSource<File, (i16, i16)> = BufferedSource::new(File::open(fname).unwrap()).unwrap();
	'outer: for s in src.map(|(x, idx)| Sample{ val: Complex{ re: x.0 as f64, im: x.1 as f64 }, idx}) {

		let opt_next_state:Option<ChannelState> = match state {
			ChannelState::AcquisitionCM(mut num_tries_so_far) => {

				if num_tries_so_far > MAX_ACQ_TRIES_SAMPLES { break 'outer; }
				else { num_tries_so_far += 1; }

				acq_cm.provide_sample(&s).unwrap();

				match acq_cm.block_for_result() {
					Ok(Some(result)) => {
						eprintln!("{:5.1} [sec] PRN {:02} {}", (s.idx as f64)/fs, prn, 
							format!("CM: {:7.1} +/- {:7.1} [Hz], {:8} [chips], {:.8}", result.doppler_hz, result.doppler_step_hz, result.code_phase, result.test_statistic()).yellow());

						let ctr = result.doppler_hz;
						let step = result.doppler_step_hz;
						acq_cl.doppler_freqs = vec![ctr - (1.2*step), ctr - (0.9*step), ctr - (0.6*step), ctr - (0.3*step), ctr, 
							ctr + (0.3*step), ctr + (0.6*step), ctr + (0.9*step), ctr + (1.2*step)];

						Some(ChannelState::AcquisitionCL(0))
						
					},
					Err(msg) => {
						eprintln!("PRN {}: Error, {:?}", prn, msg);
						break 'outer;
					},
					Ok(None) => None
				}

			},
			ChannelState::AcquisitionCL(mut num_tries_so_far) => {

				if num_tries_so_far > MAX_ACQ_TRIES_SAMPLES { break 'outer; }
				else { num_tries_so_far += 1; }

				acq_cl.provide_sample(&s).unwrap();

				match acq_cl.block_for_result() {
					Ok(Some(result)) => {
						eprintln!("{:5.1} [sec] PRN {:02} {}", (s.idx as f64)/fs, prn, 
							format!("CL: {:7.1} +/- {:7.1} [Hz], {:8} [chips], {:.8}", result.doppler_hz, result.doppler_step_hz, result.code_phase, result.test_statistic()).green());

						trk.initialize(result.doppler_hz);

						let next_state = match result.code_phase {
							0 => ChannelState::Tracking,
							n => ChannelState::PullIn(n),
						};
						Some(next_state)
						
					},
					Err(msg) => {
						eprintln!("PRN {}: Error, {:?}", prn, msg);
						break 'outer;
					},
					Ok(None) => None
				}

			},
			ChannelState::PullIn(n) => {
				let next_state = match n {
					1 => ChannelState::Tracking,
					_ => ChannelState::PullIn(n-1),
				};
				Some(next_state)
			},
			ChannelState::Tracking => {

				match trk.apply(&s) {
					TrackingResult::Ok{ prompt_i, bit_idx } => {

						eprintln!("{:5.1} [sec] PRN {:02} {}", (s.idx as f64)/fs, prn, format!("TRK OK: {:.8}, {:8.1} [Hz], {:7.1e}", trk.test_stat(), trk.carrier_freq_hz(), prompt_i).green());
						bits.push((prompt_i, bit_idx));

						None
					},
					TrackingResult::Err(e) => {
						eprintln!("PRN {:02} {}", prn, format!("ERR: {:?}", e).red());

						// After the lock is lost, don't try to re-acquire
						// Some(ChannelState::Acquisition)
						println!("{}", serde_json::to_string_pretty(&bits).unwrap());
						return;

						// None
					}
					TrackingResult::NotReady => None
				}

			}
		};

		if let Some(next_state) = opt_next_state {
			state = next_state;
		}


	}

	println!("{}", serde_json::to_string_pretty(&bits).unwrap());

}