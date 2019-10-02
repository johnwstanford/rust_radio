
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
use rust_radio::utils::kinematics;
use rust_radio::gnss::{channel, pvt, telemetry_decode};
use serde::{Serialize, Deserialize};

use na::base::{DMatrix, Matrix3x1, U3, U1};

type SF = telemetry_decode::gps::l1_ca_subframe::Subframe;
type SF4 = telemetry_decode::gps::l1_ca_subframe::Subframe4;

const C:f64 = 2.99792458e8;					 // [m/s] speed of light

// TODO: make these configurable
const NUM_ITERATIONS:usize = 50;
const NUM_ACTIVE_CHANNELS:usize = 7;

#[derive(Debug, Serialize, Deserialize)]
struct PositionWithMetadata {
	position: pvt::SatellitePosition,
	prn: usize,
	receiver_code_phase: usize,
	carrier_freq_hz: f64,
	cn0_snv_db_hz: f64,
	carrier_lock_test: f64,
	delay_iono: f64,
}

#[derive(Debug, Serialize, Deserialize)]
struct Result {
	all_sv_positions: Vec<PositionWithMetadata>,
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

	let mut inactive_channels:VecDeque<channel::Channel> = (1..=32).map(|prn| channel::new_channel(prn, fs, 0.0, 0.01)).collect();
	let mut active_channels:VecDeque<channel::Channel>   = inactive_channels.drain(..NUM_ACTIVE_CHANNELS).collect();

	let mut all_results:Vec<PositionWithMetadata> = Vec::new();
	let mut ionosphere:Option<pvt::IonosphericModel> = None;

	for s in io::file_source_i16_complex(&fname) {

		for chn in &mut active_channels {
			match chn.apply(s) {
				channel::ChannelResult::Acquisition{ doppler_hz, test_stat } => {
					eprintln!("{}", format!("PRN {}: Acquired at {} [Hz] doppler, {} test statistic, attempting to track", chn.prn, doppler_hz, test_stat).green());
				},
				channel::ChannelResult::Ok(_, sf, start_idx) => {
					// Print subframe to STDERR
					let subframe_str = format!("{:?}", &sf).blue();
					eprintln!("Subframe: {}", subframe_str);

					match sf {
						SF::Subframe4{common:_, data_id:_, sv_id:_, page:SF4::Page18{ alpha0, alpha1, alpha2, alpha3, beta0, beta1, beta2, beta3, 
							a1:_, a0:_, t_ot:_, wn_t:_, delta_t_LS:_, wn_LSF:_, delta_t_LSF:_ }} => {
								ionosphere = Some(pvt::IonosphericModel{alpha0, alpha1, alpha2, alpha3, beta0, beta1, beta2, beta3})
							},
						_ => {}
					}

					if let Some(ecef) = chn.ecef_position(sf.time_of_week()) {
						let pos_with_metadata = PositionWithMetadata{
							position: ecef,
							prn: chn.prn,
							receiver_code_phase: start_idx,
							carrier_freq_hz: chn.carrier_freq_hz(),
							cn0_snv_db_hz: chn.last_cn0_snv_db_hz(),
							carrier_lock_test: chn.last_carrier_lock_test(),
							delay_iono: 0.0,
						};
						all_results.push(pos_with_metadata);
					}

				},
				channel::ChannelResult::Err(e) => eprintln!("{}", format!("PRN {}: Error due to {:?}", chn.prn, e).red()),
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
					eprintln!("{:.1} [sec]: Putting PRN {} in the inactive buffer, replacing with PRN {}", (s.1 as f64)/fs, this_channel.prn, replacement_channel.prn);
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
	if all_results.len() >= 4 {
		let a:f64 = kinematics::WGS84_SEMI_MAJOR_AXIS_METERS;
		let b:f64 = kinematics::WGS84_SEMI_MINOR_AXIS_METERS;
		let mut x_hat = Matrix3x1::from_row_slice_generic(U3, U1, &[0.0, 0.0, all_results[0].position.gps_system_time]);
		let dt_s:f64 = 1.0 / fs;

		let mut jacobian = DMatrix::from_element(all_results.len(), 3, 0.0);

		for _ in 0..NUM_ITERATIONS {
			let mut f_vec    = DMatrix::from_element(all_results.len(), 1, 0.0);
			for i in 0..all_results.len() {
				// Calculate the Jacobian matrix
				let (x,y,z) = all_results[i].position.sv_ecef_position;
				let t:f64 = all_results[i].position.gps_system_time;
				let phi_c:f64 = all_results[i].receiver_code_phase as f64;

				let phi:f64 = x_hat[(0,0)];
				let lam:f64 = x_hat[(1,0)];

				let et:f64 = t - x_hat[(2,0)] - dt_s*phi_c;
				let ex:f64 = x - a*lam.cos()*phi.cos();
				let ey:f64 = y - a*lam.sin()*phi.cos();
				let ez:f64 = z - b*phi.sin();

				// d_residual / d_phi
				jacobian[(i, 0)] = -2.0*ex*(a*lam.cos()*phi.sin()) + 2.0*ey*(a*lam.sin()*phi.sin()) + 2.0*ez*(b*phi.cos());

				// d_residual / d_lam
				jacobian[(i, 1)] = -2.0*ex*(a*lam.sin()*phi.cos()) + 2.0*ey*(a*lam.cos()*phi.cos());

				// d_residual / d_time
				jacobian[(i, 2)] = -2.0 * et * C.powi(2);

				// Calculate f vector, representing the error for each row
				f_vec[(i, 0)] = (et.powi(2)*C.powi(2)) - ex.powi(2) - ey.powi(2) - ez.powi(2);
			}

			// Calculate the pseudoinverse of the Jacobian
			let pseudoinverse = (jacobian.tr_mul(&jacobian)).try_inverse().unwrap();

			x_hat = x_hat.clone_owned() - (pseudoinverse * jacobian.transpose() * f_vec);
			x_hat[(0,0)] = (x_hat[(0,0)]).sin().atan2((x_hat[(0,0)]).cos());
			x_hat[(1,0)] = (x_hat[(1,0)]).sin().atan2((x_hat[(1,0)]).cos());
			eprintln!("phi={:.4} [deg], lam={:.4} [deg], t0={} [sec]", x_hat[(0,0)]*57.3, x_hat[(1,0)]*57.3, x_hat[(2,0)]);
		}

		// This is the only output to STDOUT.  This allows you to pipe the results to a JSON file, but still see the status updates through STDERR as the code runs.
		let phi:f64 = x_hat[(0,0)];
		let lam:f64 = x_hat[(1,0)];
		let obs_ecef = (a*lam.cos()*phi.cos(), a*lam.sin()*phi.cos(), b*phi.sin());
		if let Some(iono) = ionosphere {
			for result in &mut all_results {
				result.delay_iono = iono.delay(obs_ecef, result.position.sv_ecef_position, result.position.gps_system_time);
			}			
		}

		let result = Result{ all_sv_positions: all_results, obs_ecef, obs_time_at_zero_code_phase:x_hat[(2,0)] };
		println!("{}", serde_json::to_string(&result).unwrap());

	}

}