

extern crate itertools;
extern crate rustfft;
extern crate serde;

use std::collections::VecDeque;

use self::rustfft::num_complex::Complex;
use self::itertools::Itertools;
use self::serde::{Serialize, Deserialize};

use ::DigSigProcErr;
use ::gnss::gps_l1_ca::{pvt, telemetry_decode, tracking};
use ::gnss::gps_l1_ca::telemetry_decode::subframe::{self, Subframe as SF, SubframeBody as SFB};

pub const DEFAULT_CARRIER_A1:f64 = 0.9;
pub const DEFAULT_CARRIER_A2:f64 = 0.9;
pub const DEFAULT_CODE_A1:f64 = 0.7;
pub const DEFAULT_CODE_A2:f64 = 0.7;

pub const C_METERS_PER_SEC:f64 = 2.99792458e8;    // [m/s] speed of light
pub const C_METERS_PER_MS:f64  = 2.99792458e5;    // [m/ms] speed of light

const SYNCHRO_BUFFER_SIZE:usize = 100;

type Sample = (Complex<f64>, usize);

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
	last_sf1:Option<subframe::subframe1::Body>,
	last_sf2:Option<subframe::subframe2::Body>,
	last_sf3:Option<subframe::subframe3::Body>,
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

								match sf.body {
									SFB::Subframe1(sf1) => self.last_sf1 = Some(sf1),
									SFB::Subframe2(sf2) => self.last_sf2 = Some(sf2),
									SFB::Subframe3(sf3) => {
										self.last_sf3 = Some(sf3);

										// If we just received subframe 3, we might have a new complete calendar and ephemeris ready
										match (self.last_sf1, self.last_sf2) {
											(Some(subframe::subframe1::Body{week_number, code_on_l2:_, ura_index:_, sv_health:_, iodc, t_gd, t_oc, a_f2, a_f1, a_f0}), 
											 Some(subframe::subframe2::Body{iode:iode2, crs, dn, m0, cuc, e, cus, sqrt_a, t_oe, fit_interval, aodo })) => {
												// TODO: make other time corrections (ionosphere, etc) 
												// TODO: account for GPS week rollover possibility
												// TODO: check for ephemeris validity time
												// TODO: consider returning a Result where the Err describes the reason for not producing a position
												if (iodc % 256) == (iode2 as u16) && iode2 == sf3.iode { 
													let new_calendar_and_ephemeris = pvt::CalendarAndEphemeris { week_number, t_gd, fit_interval, aodo,
														t_oc:(t_oc as f64), a_f0, a_f1, a_f2, t_oe, sqrt_a, dn, m0, e, omega: sf3.omega, omega0: sf3.omega0, 
														omega_dot: sf3.omega_dot, cus, cuc, crs, crc: sf3.crc, cis: sf3.cis, cic: sf3.cic, i0: sf3.i0, 
														idot: sf3.idot, iodc };
													self.calendar_and_ephemeris = Some(new_calendar_and_ephemeris);
												}
											},
											(_, _) => {}
										}
									},
									_ => { /* No special action for subframes 4 and 5 right now */ }
								}

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

}

pub fn new_default_channel(prn:usize, fs:f64) -> Channel { new_channel(prn, fs, DEFAULT_CARRIER_A1, DEFAULT_CARRIER_A2, DEFAULT_CODE_A1, DEFAULT_CODE_A2) }

pub fn new_channel(prn:usize, fs:f64, a1_carr:f64, a2_carr:f64, a1_code:f64, a2_code:f64) -> Channel {
	let state = ChannelState::AwaitingAcquisition;
	let trk = tracking::algorithm_standard::new_default_tracker(prn, 0.0, fs, a1_carr, a2_carr, a1_code, a2_code);
	let tlm = telemetry_decode::TelemetryDecoder::new();

	Channel{ prn, fs, state, trk, tlm, last_acq_doppler:0.0, last_acq_test_stat: 0.0, last_sample_idx: 0, 
		synchro_buffer: VecDeque::new(), calendar_and_ephemeris: None, opt_tow_sec: None,
		last_sf1: None, last_sf2: None, last_sf3: None }
}