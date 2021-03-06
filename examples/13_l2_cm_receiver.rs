
extern crate nalgebra as na;

use std::fs::File;

use clap::{Arg, App};
use colored::*;
use rust_radio::{io::BufferedSource, Sample};
use rust_radio::block::{BlockFunctionality, BlockResult};
use rust_radio::gnss::common::acquisition::two_stage_pcps;
use rust_radio::gnss::gps_l2c::{signal_modulation, L2_CM_PERIOD_SEC};
use rust_radio::gnss::gps_l2c::tlm_decode::{error_correction, preamble_and_crc, message_decode};
use rust_radio::gnss::gps_l2c::tracking_cm;
use rustfft::num_complex::Complex;

const MAX_ACQ_TRIES_SAMPLES:usize = 2000000;
const FEC_DECODE_LEN:usize = 300;

#[derive(Debug)]
enum ChannelState {
	Acquisition(usize),
	PullIn(usize),
	Tracking{ tried_skip:bool },
	LostLock,
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

		two_stage_pcps::Acquisition::new(symbol_cm.into_iter().map(|x| Complex{ re: x as f64, im: 0.0 }).collect(), fs, prn, 140, 2, 2.0, 0.0005, 0)
	};

	// Tracking
	let mut trk = tracking_cm::new_default_tracker(prn, 0.0, fs);

	// Telemetry decoding
	let mut pac = preamble_and_crc::PreambleAndCrc::new();

	let mut state = ChannelState::Acquisition(0);

	let mut symbols:Vec<bool> = vec![];
	let mut messages:Vec<message_decode::Message> = vec![];

	let src:BufferedSource<File, (i16, i16)> = BufferedSource::new(File::open(fname).unwrap()).unwrap();
	'outer: for s in src.map(|(x, idx)| Sample{ val: Complex{ re: x.0 as f64, im: x.1 as f64 }, idx}) {

		let opt_next_state:Option<ChannelState> = match state {
			ChannelState::Acquisition(num_tries_so_far) => {

				if num_tries_so_far > MAX_ACQ_TRIES_SAMPLES { break 'outer; }

				acq.provide_sample(&s).unwrap();

				match acq.block_for_result() {
					Ok(Some(result)) => {
						eprintln!("{:5.1} [sec] PRN {:02} {}", (s.idx as f64)/fs, prn, 
							format!("CM: {:7.1} +/- {:7.1} [Hz], {:6} [chips], {:.8}", result.doppler_hz, result.doppler_step_hz, result.code_phase, result.test_statistic()).yellow());

						trk.initialize(result.doppler_hz);

						let next_state = match result.code_phase {
							0 => ChannelState::Tracking{ tried_skip: false },
							n => ChannelState::PullIn(n),
						};
						Some(next_state)
						
					},
					_ => Some(ChannelState::Acquisition(num_tries_so_far+1))
				}

			},
			ChannelState::PullIn(n) => {
				let next_state = match n {
					1 => ChannelState::Tracking{ tried_skip: false },
					_ => ChannelState::PullIn(n-1),
				};
				Some(next_state)
			},
			ChannelState::Tracking{ mut tried_skip } => {

				match trk.apply(&s) {
					BlockResult::Ready(report) => {

						symbols.push(report.prompt_i > 0.0);
						if symbols.len() == FEC_DECODE_LEN {
							let opt_decoded_bits = error_correction::decode(symbols.drain(..).collect());
							match opt_decoded_bits {
								Some(decoded_bits) => {
									
									for b in decoded_bits {
										if let Some(msg_bits) = pac.apply(b) {
											// This set of bits passed the preamble and CRC check, so decode them as a message
											match message_decode::Message::new(&msg_bits) {
												Ok(msg) => {
													eprintln!("{:6.1} [sec] PRN {:02} {}", (s.idx as f64)/fs, prn, format!("New MSG: {:?}", msg).blue());
													messages.push(msg)
												},
												Err(e)  => eprintln!("{:6.1} [sec] PRN {:02} {}", (s.idx as f64)/fs, prn, format!("MSG Decode Err: {:?}", e).red())
											}
										}
									}

									eprintln!("{:6.1} [sec] PRN {:02} {}", (s.idx as f64)/fs, prn, format!("TRK OK: {:.8}, {:.1} [Hz]", trk.test_stat(), trk.carrier_freq_hz()).green());
									
									None
								},
								None => {
									if tried_skip {
										Some(ChannelState::LostLock)
									} else {
										// Try skipping a symbol if this doesn't work the first time; we don't know if we started on a G1 or G2 symbol
										symbols.push(report.prompt_i > 0.0);
										tried_skip = true;
										None
									}
									
								}
							}
						} else {
							None
						}

					},
					BlockResult::Err(e) => {
						eprintln!("PRN {:02} {}", prn, format!("ERR: {:?}", e).red());

						Some(ChannelState::LostLock)
					}
					BlockResult::NotReady => None
				}

			},
			ChannelState::LostLock => { 

				break 'outer;
			}
		};

		if let Some(next_state) = opt_next_state {
			state = next_state;
		}

	}

	println!("{}", serde_json::to_string_pretty(&messages).unwrap());

}