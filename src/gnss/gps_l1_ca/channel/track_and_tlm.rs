

extern crate itertools;
extern crate rustfft;
extern crate serde;

use std::collections::VecDeque;

use self::rustfft::num_complex::Complex;
use self::itertools::Itertools;
use self::serde::{Serialize, Deserialize};

use ::DigSigProcErr;
use ::gnss::gps_l1_ca::{pvt, telemetry_decode, tracking};
use ::gnss::gps_l1_ca::telemetry_decode::subframe;

pub const DEFAULT_CARRIER_A1:f64 = 0.9;
pub const DEFAULT_CARRIER_A2:f64 = 0.9;
pub const DEFAULT_CODE_A1:f64 = 0.7;
pub const DEFAULT_CODE_A2:f64 = 0.7;

pub const C_METERS_PER_SEC:f64 = 2.99792458e8;    // [m/s] speed of light
pub const C_METERS_PER_MS:f64  = 2.99792458e5;    // [m/ms] speed of light

const SYNCHRO_BUFFER_SIZE:usize = 100;

type Sample = (Complex<f64>, usize);
type SF = subframe::Subframe;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ChannelState {
	AwaitingAcquisition,
	PullIn(usize),
	Tracking,
}

#[derive(Debug)]
pub enum ChannelResult {
	NotReady(&'static str),
	Ok{sf:Option<SF>},
	Err(DigSigProcErr),
}

struct ChannelSynchro {
	rx_time: f64,
	tow_at_current_symbol_ms: f64,
}

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub struct ChannelObservation {
	pub rx_time: f64,
	pub interp_tow_ms: f64,
	pub pseudorange_m: f64,
	pub pos_ecef: (f64, f64, f64),
	pub sv_clock: f64,
	pub t_gd: f64,
}

pub struct Channel {
	pub prn:usize,
	pub fs:f64,
	state:ChannelState,
	trk: tracking::algorithm_standard::Tracking,
	tlm: telemetry_decode::TelemetryDecoder,
	last_acq_doppler:f64,
	last_acq_test_stat:f64,
	last_sample_idx:usize,
	sf_buffer:VecDeque<SF>,
	synchro_buffer:VecDeque<ChannelSynchro>,
	calendar_and_ephemeris:Option<pvt::CalendarAndEphemeris>,
	opt_tow_sec:Option<f64>,
}

impl Channel {

	// Read-only getter methods
	pub fn carrier_freq_hz(&self) -> f64 { self.trk.carrier_freq_hz() }
	pub fn test_stat(&self) -> f64 { self.trk.test_stat() }

	pub fn last_acq_doppler(&self) -> f64 { self.last_acq_doppler }
	pub fn last_acq_test_stat(&self) -> f64 { self.last_acq_test_stat }
	pub fn state(&self) -> ChannelState { self.state }
	pub fn calendar_and_ephemeris(&self) -> Option<pvt::CalendarAndEphemeris> { self.calendar_and_ephemeris }

	pub fn initialize(&mut self, acq_freq:f64, code_phase:usize) {
		self.state = match code_phase {
			0 => ChannelState::Tracking,
			n => ChannelState::PullIn(n),
		};
		self.trk.initialize(acq_freq);
		self.tlm.initialize();
	}

	pub fn acquire(&mut self, test_statistic:f64, doppler_hz:f64, code_phase:usize) {
		self.initialize(doppler_hz, code_phase);
		self.last_acq_doppler = doppler_hz;
		self.last_acq_test_stat = test_statistic;		
	}

	pub fn apply(&mut self, s:Sample) -> ChannelResult { 
		if s.1 <= self.last_sample_idx && s.1 > 0 { panic!("Somehow got the same sample twice or went backwards"); }
		self.last_sample_idx = s.1;

		match self.state {
			ChannelState::AwaitingAcquisition => ChannelResult::NotReady("Waiting on acquisition"),
			ChannelState::PullIn(n) => {
				self.state = match n {
					1 => ChannelState::Tracking,
					_ => ChannelState::PullIn(n-1),
				};
				ChannelResult::NotReady("Pulling in signal")
			},
			ChannelState::Tracking => { 
				match self.trk.apply(s) {
					tracking::algorithm_standard::TrackingResult::Ok{prompt_i, bit_idx} => {
						// The tracker has a lock and produced a bit, so pass it into the telemetry decoder and match on the result
						if let Some(tow_sec) = &mut self.opt_tow_sec {
							*tow_sec += 0.02;
						}

						// See if a new subframe is available
						let sf:Option<SF> = match self.tlm.apply((prompt_i > 0.0, bit_idx)) {
							telemetry_decode::TelemetryDecoderResult::Ok(sf, _, _) => {
		
								self.opt_tow_sec = Some(sf.time_of_week());

								// Populate the subframe buffer
								self.sf_buffer.push_back(sf);
								while self.sf_buffer.len() > 3 {
									self.sf_buffer.pop_front();
								}

								// Populate channel data derived from subframes
								self.check_calendar_and_ephemeris();

								Some(sf)
							},
							telemetry_decode::TelemetryDecoderResult::NotReady => None,
							telemetry_decode::TelemetryDecoderResult::Err(_) => {
								self.state = ChannelState::AwaitingAcquisition;
								None
							}
						};

						// Populate the synchro buffer
						if let Some(tow_sec) = self.opt_tow_sec {
							let this_synchro = ChannelSynchro{ rx_time: (bit_idx as f64 + self.trk.code_phase_samples())/self.fs,
								tow_at_current_symbol_ms: tow_sec*1000.0 };
							self.synchro_buffer.push_back(this_synchro);
							while self.synchro_buffer.len() > SYNCHRO_BUFFER_SIZE {
								self.synchro_buffer.pop_front();
							}
						}

						ChannelResult::Ok{sf}

					},
					tracking::algorithm_standard::TrackingResult::NotReady => ChannelResult::NotReady("Waiting on next bit from tracker"),
					tracking::algorithm_standard::TrackingResult::Err(e) => {
						self.state = ChannelState::AwaitingAcquisition;
						ChannelResult::Err(e)
					},
				}
			}
		}
	}

	pub fn get_observation(&self, rx_time:f64, rx_tow_sec:f64) -> Option<ChannelObservation> {
		let interp:Option<f64> = self.synchro_buffer.iter().tuple_windows().find(|(a,b)| a.rx_time <= rx_time && b.rx_time >= rx_time).map(|(a,b)| {
			let time_factor:f64 = (rx_time - a.rx_time) / (b.rx_time - a.rx_time);
			a.tow_at_current_symbol_ms + ((b.tow_at_current_symbol_ms - a.tow_at_current_symbol_ms) * time_factor)
		});

		//eprintln!("interp_tow={} cae={}", interp.is_some(), self.calendar_and_ephemeris.is_some());
		if let (Some(interp_tow_ms), Some(cae)) = (interp, self.calendar_and_ephemeris) {
			let interp_tow_sec = interp_tow_ms / 1000.0;
			let pseudorange_m:f64 = (rx_tow_sec - interp_tow_sec) * C_METERS_PER_SEC;
			let (pos_ecef, sv_clock) = cae.pos_and_clock(interp_tow_sec);
			Some(ChannelObservation{ rx_time, interp_tow_ms, pseudorange_m, pos_ecef, sv_clock, t_gd: cae.t_gd })
		} else { None}
	}

	fn check_calendar_and_ephemeris(&mut self) {
		match (self.sf_buffer.get(0), self.sf_buffer.get(1), self.sf_buffer.get(2)) {
			(Some(SF::Subframe1{common:_, week_number, code_on_l2:_, ura_index:_, sv_health:_, iodc, t_gd, t_oc, a_f2, a_f1, a_f0}), 
			 Some(SF::Subframe2{common:_, iode:iode2, crs, dn, m0, cuc, e, cus, sqrt_a, t_oe, fit_interval, aodo }), 
			 Some(SF::Subframe3{common:_, cic, omega0, cis, i0, crc, omega, omega_dot, iode:iode3, idot})) => {
				// TODO: make other time corrections (ionosphere, etc) 
				// TODO: account for GPS week rollover possibility
				// TODO: check for ephemeris validity time
				// TODO: consider returning a Result where the Err describes the reason for not producing a position
				if (*iodc % 256) == (*iode2 as u16) && *iode2 == *iode3 { 
					let new_calendar_and_ephemeris = pvt::CalendarAndEphemeris { week_number:*week_number, t_gd:*t_gd, fit_interval:*fit_interval, aodo:*aodo,
						t_oc:(*t_oc as f64), a_f0:*a_f0, a_f1:*a_f1, a_f2:*a_f2, t_oe:*t_oe, 
						sqrt_a:*sqrt_a, dn:*dn, m0:*m0, e:*e, omega:*omega, omega0:*omega0, omega_dot:*omega_dot, cus:*cus, cuc:*cuc, crs:*crs, 
						crc:*crc, cis:*cis, cic:*cic, i0:*i0, idot:*idot, iodc:*iodc };
					self.calendar_and_ephemeris = Some(new_calendar_and_ephemeris);
				}
			},
			(_, _, _) => {}
		}
	}

}

pub fn new_default_channel(prn:usize, fs:f64) -> Channel { new_channel(prn, fs, DEFAULT_CARRIER_A1, DEFAULT_CARRIER_A2, DEFAULT_CODE_A1, DEFAULT_CODE_A2) }

pub fn new_channel(prn:usize, fs:f64, a1_carr:f64, a2_carr:f64, a1_code:f64, a2_code:f64) -> Channel {
	let state = ChannelState::AwaitingAcquisition;
	let trk = tracking::algorithm_standard::new_default_tracker(prn, 0.0, fs, a1_carr, a2_carr, a1_code, a2_code);
	let tlm = telemetry_decode::TelemetryDecoder::new();

	Channel{ prn, fs, state, trk, tlm, last_acq_doppler:0.0, last_acq_test_stat: 0.0, last_sample_idx: 0, 
		sf_buffer: VecDeque::new(), synchro_buffer: VecDeque::new(), calendar_and_ephemeris: None, opt_tow_sec: None }
}