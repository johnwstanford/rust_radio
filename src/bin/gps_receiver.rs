
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
use rust_radio::gnss::pvt;
use rust_radio::gnss::telemetry_decode::gps;
use serde::{Serialize, Deserialize};

use na::base::DMatrix;

const C:f64 = 2.99792458e8;					 // [m/s] speed of light

// TODO: make these configurable
const NUM_ITERATIONS:usize = 50;
const NUM_ACTIVE_CHANNELS:usize = 7;

#[derive(Debug, Serialize, Deserialize)]
struct Result {
	all_sv_positions: Vec<pvt::SatellitePosition>,
	obs_ecef:(f64, f64, f64),
	obs_time_at_zero_code_phase:f64,
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

	let mut inactive_channels:VecDeque<(channel::Channel, VecDeque<gps::l1_ca_subframe::Subframe>)> = (1..=32).map(|prn| {
		(channel::new_channel(prn, fs, 0.0, 0.012), VecDeque::new())
	}).collect();

	let mut active_channels:VecDeque<(channel::Channel, VecDeque<gps::l1_ca_subframe::Subframe>)> = inactive_channels.drain(..NUM_ACTIVE_CHANNELS).collect();

	let mut all_results:Vec<pvt::SatellitePosition> = Vec::new();

	for s in io::file_source_i16_complex(&fname) {

		for (chn, sf_buffer) in &mut active_channels {
			match chn.apply(s) {
				channel::ChannelResult::Acquisition{ doppler_hz, test_stat } => {
					eprintln!("{}", format!("PRN {}: Acquired at {} [Hz] doppler, {} test statistic, attempting to track", chn.prn, doppler_hz, test_stat).green());
				},
				channel::ChannelResult::Ok(_, sf, _) => {
					// Print subframe to STDERR
					let subframe_str = format!("{:?}", &sf).blue();
					eprintln!("    {}", subframe_str);

					sf_buffer.push_back(sf);
					
					// Limit subframe buffer size to 3
					while sf_buffer.len() > 3 { sf_buffer.pop_front(); }
					if sf_buffer.len() == 3 {
						if let Some(ecef) = pvt::get_ecef(sf_buffer[0], sf_buffer[1], sf_buffer[2]) {
							all_results.push(ecef);
						}
					}

				},
				channel::ChannelResult::Err(e) => eprintln!("{}", format!("Error due to {:?}", e).red()),
				_ => {}
			}
		}

		// Once per second, move channels without a signal lock to the inactive buffer and replace them with new ones
		if (s.1 % (fs as usize) == 0) && (s.1 > 0) {
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

	// Position fix
	let mut x_hat = DMatrix::from_element(4, 1, 0.0);
	x_hat[(0,0)] = 6.371e6;
	x_hat[(3,0)] = all_results[0].gps_system_time;
	let dt_s:f64 = 1.0 / fs;

	let mut jacobian = DMatrix::from_element(all_results.len(), 4, 0.0);

	for _ in 0..NUM_ITERATIONS {
		let mut f_vec    = DMatrix::from_element(all_results.len(), 1, 0.0);
		for i in 0..all_results.len() {
			// Calculate the Jacobian matrix
			let (x,y,z) = all_results[i].sv_ecef_position;
			let t:f64 = all_results[i].gps_system_time;
			let phi_c:f64 = all_results[i].receiver_code_phase as f64;
			jacobian[(i, 0)] = -2.0 * (x_hat[(0,0)] - x);
			jacobian[(i, 1)] = -2.0 * (x_hat[(1,0)] - y);
			jacobian[(i, 2)] = -2.0 * (x_hat[(2,0)] - z);
			jacobian[(i, 3)] =  2.0 * (x_hat[(3,0)] + dt_s*phi_c - t) * C.powi(2);

			// Calculate f vector, representing the error for each rows
			f_vec[(i, 0)] = (x_hat[(3,0)] + dt_s*phi_c - t).powi(2) * C.powi(2) -
				(x_hat[(0,0)] - x).powi(2) -
				(x_hat[(1,0)] - y).powi(2) -
				(x_hat[(2,0)] - z).powi(2);
		}

		// Calculate the pseudoinverse of the Jacobian
		let mut jacobian_cpy1 = DMatrix::from_element(all_results.len(), 4, 0.0);
		let mut jacobian_cpy2 = DMatrix::from_element(all_results.len(), 4, 0.0);
		jacobian_cpy1.copy_from(&jacobian);
		jacobian_cpy2.copy_from(&jacobian);

		let pseudoinverse = (jacobian.transpose() * jacobian_cpy1).try_inverse().unwrap();

		let mut x_hat1 = DMatrix::from_element(4, 1, 0.0);
		let mut x_hat2 = DMatrix::from_element(4, 1, 0.0);
		x_hat1.copy_from(&x_hat);
		x_hat2.copy_from(&x_hat);

		x_hat = x_hat1 - (pseudoinverse * jacobian.transpose() * f_vec);
		eprintln!("x={:1.3e} y={:1.3e} z={:1.3e} t={:1.3e}", x_hat[(0,0)], x_hat[(1,0)], x_hat[(2,0)], x_hat[(3,0)]);
	}

	// This is the only output to STDOUT.  This allows you to pipe the results to a JSON file, but still see the status updates through STDERR as the code runs.
	let result = Result{ all_sv_positions: all_results, obs_ecef:(x_hat[(0,0)], x_hat[(1,0)], x_hat[(2,0)]), obs_time_at_zero_code_phase:x_hat[(3,0)] };
	println!("{}", serde_json::to_string(&result).unwrap());

}