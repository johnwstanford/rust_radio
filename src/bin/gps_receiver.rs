
extern crate clap;
extern crate colored;
extern crate rust_radio;
extern crate dirs;
extern crate serde;
extern crate nalgebra as na;

use std::collections::VecDeque;
//use std::{thread, time};

use clap::{Arg, App};
use colored::*;
use na::{Vector3, Vector4, DVector, DMatrix};
use rust_radio::io;
use rust_radio::gnss::channel;
use rust_radio::utils::kinematics;
use serde::{Serialize, Deserialize};

// TODO: make these configurable
const NUM_ACTIVE_CHANNELS:usize = 7;
const MAX_ITER:usize = 10;
const SV_COUNT_THRESHOLD:usize = 5;
const RESIDUAL_NORM_THRESHOLD_METERS:f64 = 200.0;

#[derive(Debug, Serialize, Deserialize)]
struct GnssFix {
	pos_ecef:(f64, f64, f64),
	residual_norm:f64,
	sv_count:usize,
	current_rx_time: f64,
}

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

	let pvt_rate_samples:usize = (fs * 0.02) as usize;
	let mut all_fixes:Vec<GnssFix> = vec![];

	let mut x_master = Vector4::new(0.0, 0.0, 0.0, 0.0);

	for s in io::file_source_i16_complex(&fname) {

		let current_rx_time:f64 = (s.1 as f64) / fs;
		if let Some(ref mut t0) = opt_t0 {
			*t0 += 1.0 / fs;
		}

		let mut obs_this_soln:Vec<channel::ChannelObservation> = Vec::new();
		for chn in &mut active_channels {
			if (s.1)%pvt_rate_samples == 0 {
				let opt_co = chn.get_observation(current_rx_time - 0.1, opt_t0.unwrap_or(0.0) - 0.1);
				if let Some(co) = opt_co {
					obs_this_soln.push(co);
				}
			}

			match chn.apply(s) {
				channel::ChannelResult::Acquisition{ doppler_hz, test_stat } =>
					eprintln!("PRN {}: Acquired at {} [Hz] doppler, {} test statistic, attempting to track", chn.prn, doppler_hz, test_stat),
				channel::ChannelResult::Ok{sf:Some(new_sf)} => {
					opt_t0.get_or_insert(new_sf.time_of_week() + 0.086);
					eprintln!("New Subframe: {}", format!("{:?}", new_sf).cyan());
				},
				channel::ChannelResult::Err(e) => 
					eprintln!("{}", format!("PRN {}: Error due to {:?}", chn.prn, e).red()),
				_ => {}
			}
		}

		if obs_this_soln.len() >= SV_COUNT_THRESHOLD {
			let n = obs_this_soln.len();

			let mut x = x_master.clone();
			let mut v = DVector::from_element(n, 0.0);

			// Try to solve for position
			for _ in 0..MAX_ITER {
				let pos_wgs84 = kinematics::ecef_to_wgs84(x[0], x[1], x[2]);

				let mut h = DMatrix::from_element(n, 4, 0.0);

				for (i, ob) in obs_this_soln.iter().enumerate() {
					let (e,r) = kinematics::dist_with_sagnac_effect(
						Vector3::new(ob.pos_ecef.0, ob.pos_ecef.1, ob.pos_ecef.2),
						Vector3::new(x[0], x[1], x[2]));
					let (_, el) = kinematics::az_el(pos_wgs84.latitude, pos_wgs84.longitude, pos_wgs84.height_above_ellipsoid, e);

					let sig = ((0.9 / el.sin()) + 5.94).sqrt();

					v[i] = (ob.pseudorange_m + (kinematics::C)*(ob.sv_clock - ob.t_gd) - r - x[3]) / sig;
					for j in 0..3 { h[(i,j)] = -e[j]/sig; }
					h[(i,3)] = 1.0/sig;
				}


				if let Some(q) = (h.tr_mul(&h)).try_inverse() {
					let dx = q * h.tr_mul(&v);

					x = x + dx.clone();

					if dx.norm() < 1.0e-4 { 

						// The iterative least squares method has converged
						let fix = GnssFix{pos_ecef:(x[0], x[1], x[2]), residual_norm:v.norm(), sv_count:n, current_rx_time };
						if fix.residual_norm.is_finite() && fix.pos_ecef.0.is_finite() && fix.pos_ecef.1.is_finite() && fix.pos_ecef.2.is_finite() && fix.residual_norm <= RESIDUAL_NORM_THRESHOLD_METERS {
							let new_pos = kinematics::ecef_to_wgs84(x[0], x[1], x[2]);
							eprintln!("{}", format!("Position Fix: {:.5} [deg] lat, {:.5} [deg] lon, {:.1} [m]", 
								new_pos.latitude * 57.3, new_pos.longitude * 57.3, new_pos.height_above_ellipsoid).green().bold());

							// Commit this fix to the master fix
							for i in 0..3 { x_master[i] = x[i]; }
							if let Some(ref mut t0) = opt_t0 { *t0 -= x[3] / (kinematics::C); }

							all_fixes.push(fix);
						}

						// Whether we committed the fix or not, break out of the for loop						
						break; 
					}

				} else { 
					// If we get a non-invertible matrix, just break out of the for loop
					break; 
				}

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

	println!("{}", serde_json::to_string_pretty(&all_fixes).unwrap());

}