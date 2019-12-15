
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
use na::Vector4;
use rust_radio::io;
use rust_radio::gnss::{channel, pvt};
use rust_radio::utils::kinematics;
use rustfft::num_complex::Complex;

// TODO: make these configurable
const NUM_ACTIVE_CHANNELS:usize = 7;
const WEEK_SEC:f64 = 3600.0 * 24.0 * 7.0;

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
	let mut tow_rcv:f64 = 0.0;

	eprintln!("Decoding {} at {} [samples/sec]", &fname, &fs);

	let mut inactive_channels:VecDeque<channel::Channel> = (1..=32).map(|prn| channel::new_channel(prn, fs, 0.0, 0.01)).collect();
	let mut active_channels:VecDeque<channel::Channel>   = inactive_channels.drain(..NUM_ACTIVE_CHANNELS).collect();

	let pvt_rate_samples:usize = (fs * 0.02) as usize;
	let mut all_fixes:Vec<pvt::GnssFix> = vec![];

	let mut x_master = Vector4::new(0.0, 0.0, 0.0, 0.0);

	for s in io::file_source_i16_complex(&fname).map(|(x, idx)| (Complex{ re: x.0 as f64, im: x.1 as f64 }, idx)) {

		let current_rx_time:f64 = (s.1 as f64) / fs;
		tow_rcv += 1.0 / fs;
		if tow_rcv > WEEK_SEC { tow_rcv -= WEEK_SEC; }

		let mut obs_this_soln:Vec<channel::track_and_tlm::ChannelObservation> = Vec::new();
		for chn in &mut active_channels {
			if (s.1)%pvt_rate_samples == 0 {
				let opt_co = chn.get_observation(current_rx_time - 0.1, tow_rcv - 0.1);
				if let Some(co) = opt_co {
					obs_this_soln.push(co);
				}
			}

			match chn.apply(s) {
				channel::ChannelResult::Acquisition{ doppler_hz, test_stat } =>
					eprintln!("PRN {}: Acquired at {} [Hz] doppler, {} test statistic, attempting to track", chn.prn, doppler_hz, test_stat),
				channel::ChannelResult::Ok{sf:Some(new_sf)} => {
					if (new_sf.time_of_week() - tow_rcv).abs() > 1.0 { tow_rcv = new_sf.time_of_week() + 0.086 }
					eprintln!("New Subframe: {}", format!("{:?}", new_sf).cyan());
				},
				channel::ChannelResult::Err(e) => 
					eprintln!("{}", format!("PRN {}: Error due to {:?}", chn.prn, e).red()),
				_ => {}
			}
		}

		if let Ok((fix, x)) = pvt::solve_position_and_time(obs_this_soln, x_master, current_rx_time) {
			if fix.residual_norm < 200.0 {
				let new_pos = kinematics::ecef_to_wgs84(fix.pos_ecef.0, fix.pos_ecef.1, fix.pos_ecef.2);
				eprintln!("{}", format!("Position Fix: {:.5} [deg] lat, {:.5} [deg] lon, {:.1} [m]", 
					new_pos.latitude * 57.3, new_pos.longitude * 57.3, new_pos.height_above_ellipsoid).green().bold());

				tow_rcv -= x[3] / (kinematics::C);
				for i in 0..3 { x_master[i] = x[i]; }
				all_fixes.push(fix);
			}
		}

		// Every 0.1 sec, move channels without a signal lock to the inactive buffer and replace them with new ones
		if (s.1 % (fs as usize / 10) == 0) && (s.1 > 0) {
			for _ in 0..NUM_ACTIVE_CHANNELS {
				let this_channel = active_channels.pop_front().unwrap();
				if this_channel.state() == channel::track_and_tlm::ChannelState::AwaitingAcquisition {
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