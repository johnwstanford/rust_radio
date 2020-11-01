
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
use na::Vector4;
use rustfft::num_complex::Complex;

use rust_radio::block::{BlockFunctionality, BlockResult};
use rust_radio::block::block_tree_sync_static::split_and_merge::RotatingSplitAndMerge;
use rust_radio::{io::BufferedSource, Sample};
use rust_radio::gnss::gps_l1_ca::pvt;
use rust_radio::gnss::gps_l1_ca::channel::{self, ChannelReport};
use rust_radio::utils::kinematics;

// TODO: make these configurable
const WEEK_SEC:f64 = 3600.0 * 24.0 * 7.0;

pub fn main() -> Result<(), &'static str> {

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
		.arg(Arg::with_name("output_fixes")
			.long("output_fixes")
			.help("Output filename for JSON-formatted fixes")
			.takes_value(true))
		.arg(Arg::with_name("output_rollovers")
			.long("output_rollovers")
			.help("Output filename for JSON-formatted TOW rollovers")
			.takes_value(true))
		.get_matches();

	let fname:&str = matches.value_of("filename").unwrap();
	let fs = matches.value_of("sample_rate_sps").unwrap().parse().unwrap();
	
	let mut tow_rcv:f64 = 0.0;
	let mut last_tow_rcv:f64 = 0.0;
	let mut updated_once:bool = false;

	eprintln!("Decoding {} at {} [samples/sec]", &fname, &fs);

	let pvt_rate_samples:usize = (fs * 0.5) as usize;

	let mut sam = RotatingSplitAndMerge::from_iter((1..=32).map( |prn| {

		channel::new_channel(prn, fs, 0.008, pvt_rate_samples)
	
	}), 200_000, None);


	let mut all_fixes:Vec<pvt::GnssFix> = vec![];
	let mut all_rollovers:Vec<(f64, usize)> = vec![];

	let mut x_master = Vector4::new(0.0, 0.0, 0.0, 0.0);
	let ionosphere:Option<pvt::ionosphere::Model> = None;

	let src:BufferedSource<File, (i16, i16)> = BufferedSource::new(File::open(&fname).unwrap()).unwrap();
	for s in src.map(|(x, idx)| Sample{ val: Complex{ re: x.0 as f64, im: x.1 as f64 }, idx }) {

		let current_rx_time:f64 = (s.idx as f64 + 0.5) / fs;
		
		tow_rcv += 1.0 / fs;
		if tow_rcv > WEEK_SEC { tow_rcv -= WEEK_SEC; }

		// See if we've rolled over into a new GPS second
		if tow_rcv.floor() != last_tow_rcv.floor() && updated_once {
			// The tuple is (second we rolled over into, sample where we rolled over into it)
			all_rollovers.push((tow_rcv.floor(), s.idx));
		}

		last_tow_rcv = tow_rcv;

		let sample_w_time = (s, tow_rcv);

		let mut obs_this_soln:Vec<pvt::Observation> = Vec::new();

		let result:BlockResult<Vec<ChannelReport>> = sam.apply(&sample_w_time);

		match result {
			BlockResult::Ready(reports) => {
				for ChannelReport { opt_subframe, opt_observation, new_ionosphere:_ } in reports {
					if let Some(new_sf) = opt_subframe {

						if (new_sf.time_of_week() - tow_rcv).abs() > 1.0 { tow_rcv = new_sf.time_of_week() + 0.086 }
						eprintln!("New Subframe: {}", format!("{:?}", new_sf).cyan());

					}

					if let Some(obs) = opt_observation { 
						obs_this_soln.push(obs);
					}
				}
			},
			BlockResult::Err(e) => eprintln!("{}", format!("Error: {:?}", e).red()),
			_ => {}

		}

		if let Ok((fix, x)) = pvt::solve_position_and_time(obs_this_soln, x_master, current_rx_time, ionosphere) {
			if fix.residual_norm < 400.0 {
				let new_pos = kinematics::ecef_to_wgs84(fix.pos_ecef.0, fix.pos_ecef.1, fix.pos_ecef.2);
				eprintln!("{}", format!("Position/Time Fix: {:.3} [sec], {:.5} [deg] lat, {:.5} [deg] lon, {:.1} [m]", 
					tow_rcv, new_pos.latitude * 57.3, new_pos.longitude * 57.3, new_pos.height_above_ellipsoid).green().bold());

				tow_rcv -= x[3] / (kinematics::C);
				for i in 0..3 { x_master[i] = x[i]; }
				updated_once = true;
				all_fixes.push(fix);
			}
		}

	}

	if let Some(outfile) = matches.value_of("output_fixes") {
		std::fs::write(outfile, serde_json::to_string_pretty(&all_fixes).unwrap().as_bytes()).map_err(|_| "Unable to write fixes JSON")?;
	}

	if let Some(outfile) = matches.value_of("output_rollovers") {
		std::fs::write(outfile, serde_json::to_string_pretty(&all_rollovers).unwrap().as_bytes()).map_err(|_| "Unable to write rollovers JSON")?;
	}

	Ok(())
}