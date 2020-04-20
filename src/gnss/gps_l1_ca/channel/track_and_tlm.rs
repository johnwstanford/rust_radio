
extern crate rustfft;
extern crate serde;

use self::rustfft::num_complex::Complex;

use ::DigSigProcErr;
use ::gnss::gps_l1_ca::{pvt, telemetry_decode, tracking};
use ::gnss::gps_l1_ca::telemetry_decode::subframe::{self, Subframe as SF, SubframeBody as SFB};

pub const DEFAULT_CARRIER_A1:f64 = 0.9;
pub const DEFAULT_CARRIER_A2:f64 = 0.9;
pub const DEFAULT_CODE_A1:f64 = 0.7;
pub const DEFAULT_CODE_A2:f64 = 0.7;

pub const C_METERS_PER_SEC:f64 = 2.99792458e8;    // [m/s] speed of light

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
	Ok{sf:Option<SF>, new_ionosphere:bool },
	Err(DigSigProcErr),
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
	ephemeris:Option<pvt::ephemeris::Ephemeris>,
	ionosphere:Option<pvt::ionosphere::Model>,
}

impl Channel {

	// Read-only getter methods
	pub fn carrier_freq_hz(&self) -> f64 { self.trk.carrier_freq_hz() }
	pub fn test_stat(&self) -> f64 { self.trk.test_stat() }

	pub fn last_acq_doppler(&self) -> f64 { self.last_acq_doppler }
	pub fn last_acq_test_stat(&self) -> f64 { self.last_acq_test_stat }
	pub fn state(&self) -> ChannelState { self.state }
	pub fn ephemeris(&self)  -> Option<pvt::ephemeris::Ephemeris> { self.ephemeris }
	pub fn ionosphere(&self) -> Option<pvt::ionosphere::Model> { self.ionosphere }

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
				let mut new_ionosphere = false;

				match self.trk.apply(s) {
					tracking::algorithm_standard::TrackingResult::Ok{prompt_i, bit_idx} => {
						// prompt_i is an f64 representing the prompt value of this bit
						// bit_idx is the index of the last sample that made up this bit

						// The tracker has a lock and produced a bit, so pass it into the telemetry decoder and match on the result
						let sf:Option<SF> = match self.tlm.apply((prompt_i > 0.0, bit_idx)) {
							telemetry_decode::TelemetryDecoderResult::Ok(sf, _, _) => {
		
								self.trk.reset_clock(sf.time_of_week() + (self.trk.code_phase_samples()/self.fs));

								match sf.body {
									SFB::Subframe1(sf1) => self.last_sf1 = Some(sf1),
									SFB::Subframe2(sf2) => self.last_sf2 = Some(sf2),
									SFB::Subframe3(sf3) => {
										self.last_sf3 = Some(sf3);

										// If we just received subframe 3, we might have a new complete calendar and ephemeris ready
										match (self.last_sf1, self.last_sf2) {
											(Some(subframe::subframe1::Body{week_number, code_on_l2:_, ura_index:_, sv_health:_, iodc, t_gd, t_oc, a_f2, a_f1, a_f0}), 
											 Some(subframe::subframe2::Body{iode:iode2, crs, dn, m0, cuc, e, cus, sqrt_a, t_oe, fit_interval, aodo })) => {
												if (iodc % 256) == (iode2 as u16) && iode2 == sf3.iode { 
													let new_ephemeris = pvt::ephemeris::Ephemeris { week_number, t_gd, fit_interval, aodo,
														t_oc:(t_oc as f64), a_f0, a_f1, a_f2, t_oe, sqrt_a, dn, m0, e, omega: sf3.omega, omega0: sf3.omega0, 
														omega_dot: sf3.omega_dot, cus, cuc, crs, crc: sf3.crc, cis: sf3.cis, cic: sf3.cic, i0: sf3.i0, 
														idot: sf3.idot, iodc };
													self.ephemeris = Some(new_ephemeris);
												}
											},
											(_, _) => {}
										}
									},
									SFB::Subframe4(sf4) => {
										match sf4.page {
											subframe::subframe4::Page::Page18{ alpha0, alpha1, alpha2, alpha3, beta0, beta1, beta2, beta3, .. } => {
												new_ionosphere = true;
												self.ionosphere = Some(pvt::ionosphere::Model{alpha0, alpha1, alpha2, alpha3, beta0, beta1, beta2, beta3})
											},
											_ => { /* No special action for pages other than 18 right now */}
										}
									},
									_ => { /* No special action for subframe 5 right now */ }
								}

								Some(sf)
							},
							telemetry_decode::TelemetryDecoderResult::NotReady => None,
							telemetry_decode::TelemetryDecoderResult::Err(_) => {
								self.state = ChannelState::AwaitingAcquisition;
								None
							}
						};

						ChannelResult::Ok{sf, new_ionosphere}

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

	pub fn get_observation(&self, rx_tow_sec:f64) -> Option<pvt::Observation> {
		if let Some(eph) = self.ephemeris {
			// TODO: account for GPS week rollover possibility
			// TODO: check for ephemeris validity time
			// TODO: consider returning a Result where the Err describes the reason for not producing a position
			let sv_tow_sec:f64 = self.trk.sv_time_of_week();
			let (pos_ecef, sv_clock) = eph.pos_and_clock(sv_tow_sec);
			let carrier_freq_hz:f64 = self.trk.carrier_freq_hz();
			let pseudorange_m:f64 = (rx_tow_sec - sv_tow_sec + sv_clock - eph.t_gd) * C_METERS_PER_SEC;
			Some(pvt::Observation{ sv_tow_sec, pseudorange_m, pos_ecef, sv_clock, t_gd: eph.t_gd, carrier_freq_hz })
		} else { None }
	}

}

pub fn new_default_channel(prn:usize, fs:f64) -> Channel { new_channel(prn, fs, DEFAULT_CARRIER_A1, DEFAULT_CARRIER_A2, DEFAULT_CODE_A1, DEFAULT_CODE_A2) }

pub fn new_channel(prn:usize, fs:f64, a1_carr:f64, a2_carr:f64, a1_code:f64, a2_code:f64) -> Channel {
	let state = ChannelState::AwaitingAcquisition;
	let trk = tracking::algorithm_standard::new_default_tracker(prn, 0.0, fs, a1_carr, a2_carr, a1_code, a2_code);
	let tlm = telemetry_decode::TelemetryDecoder::new();

	Channel{ prn, fs, state, trk, tlm, last_acq_doppler:0.0, last_acq_test_stat: 0.0, last_sample_idx: 0, 
		ephemeris: None, ionosphere: None, last_sf1: None, last_sf2: None, last_sf3: None }
}