
use crate::{Sample, DigSigProcErr as DSPErr};

use crate::block::{BlockFunctionality, BlockResult};
use crate::block::block_tree_sync_static::acquire_and_track::AcquireAndTrack;

use crate::filters::{SecondOrderFIR as FIR};
use crate::gnss::common::acquisition::{two_stage_pcps::Acquisition, AcquisitionResult};
use crate::gnss::common::tracking::TrackReport;
use crate::gnss::gps_l1_ca::{self, pvt};
use crate::gnss::gps_l1_ca::telemetry_decode;
use crate::gnss::gps_l1_ca::telemetry_decode::subframe::{self, Subframe as SF, SubframeBody as SFB};
use crate::gnss::gps_l1_ca::tracking;

pub const DEFAULT_DOPPLER_STEP_HZ:usize = 50;
pub const DEFAULT_DOPPLER_MAX_HZ:i16 = 10000;
pub const DEFAULT_TEST_STAT_THRESHOLD:f64 = 0.008;

pub const C_METERS_PER_SEC:f64 = 2.99792458e8;    // [m/s] speed of light

#[derive(Debug, Clone)]
pub enum ChannelCommand {
	Ionosphere,
	Reset,
}

#[derive(Debug)]
pub enum ChannelResponse {
	Ionosphere(Option<pvt::ionosphere::Model>),
	Ack,
}

#[derive(Debug)]
pub struct ChannelReport {
	pub opt_subframe:Option<SF>,
	pub opt_observation:Option<pvt::Observation>,
	pub new_ionosphere:bool,
}

pub struct Channel {
	pub prn: usize,
	pub fs:  f64,
	pub aat:     AcquireAndTrack<Sample, AcquisitionResult, TrackReport, Acquisition, tracking::Tracking<FIR, FIR>>,
	pub tlm:     telemetry_decode::TelemetryDecoder,
	pub last_acq_doppler:   f64,
	pub last_acq_test_stat: f64,
	pub last_sample_idx:    usize,
	pub last_sf1:Option<subframe::subframe1::Body>,
	pub last_sf2:Option<subframe::subframe2::Body>,
	pub last_sf3:Option<subframe::subframe3::Body>,
	pub ephemeris:Option<pvt::ephemeris::Ephemeris>,
	pub ionosphere:Option<pvt::ionosphere::Model>,
	pub pvt_rate_samples:usize,
}

impl BlockFunctionality<ChannelCommand, ChannelResponse, (Sample, f64), ChannelReport> for Channel {

	fn control(&mut self, c:&ChannelCommand) -> Result<ChannelResponse, &'static str> {
		match c {
			ChannelCommand::Ionosphere => Ok(ChannelResponse::Ionosphere(self.ionosphere.clone())),
			ChannelCommand::Reset => {
				// TODO: implement reset logic
				Ok(ChannelResponse::Ack)
			}
		}
	}

	fn apply(&mut self, input:&(Sample, f64)) -> BlockResult<ChannelReport> {
		self.apply_tuple(input)
	}

}

impl BlockFunctionality<(), bool, (Sample, f64), ChannelReport> for Channel {

	// A struct can implement more than one BlockFunctionality interface.  If this struct is implemented somewhere
	// that expectes a () -> bool control/response interface, we can do that.  It'll just respond with whether or not
	// this channel is actively tracking a signal

	fn control(&mut self, _:&()) -> Result<bool, &'static str> {
		Ok(!self.aat.awaiting_acq)
	}

	fn apply(&mut self, input:&(Sample, f64)) -> BlockResult<ChannelReport> {
		self.apply_tuple(input)
	}

}

impl Channel {

	// Read-only getter methods
	pub fn carrier_freq_hz(&self) -> f64 { self.aat.trk.carrier_freq_hz() }
	pub fn test_stat(&self) -> f64 { self.aat.trk.test_stat() }

	pub fn last_acq_doppler(&self) -> f64 { self.last_acq_doppler }
	pub fn last_acq_test_stat(&self) -> f64 { self.last_acq_test_stat }
	pub fn ephemeris(&self)  -> Option<pvt::ephemeris::Ephemeris> { self.ephemeris }
	pub fn ionosphere(&self) -> Option<pvt::ionosphere::Model> { self.ionosphere }

	fn apply_tuple(&mut self, input:&(Sample, f64)) -> BlockResult<ChannelReport> {
		let (s, tow_rcv) = input;

		let mut new_ionosphere = false;

		match self.aat.apply(s) {
			BlockResult::Ready(TrackReport { prompt_i, sample_idx, ..}) => {
				// prompt_i is an f64 representing the prompt value of this bit
				// bit_idx is the index of the last sample that made up this bit

				// The tracker has a lock and produced a bit, so pass it into the telemetry decoder and match on the result
				let opt_subframe:Option<SF> = match self.tlm.apply_sample((prompt_i > 0.0, sample_idx)) {
					telemetry_decode::TelemetryDecoderResult::Ok(sf, _, _) => {

						self.aat.trk.reset_clock(sf.time_of_week() + (self.aat.trk.code_phase_samples()/self.fs));

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
						// TODO: consider resetting AAT if the TLM data is bad
						None
					}
				};

				let opt_observation = if s.idx % self.pvt_rate_samples == 0 {
					self.opt_observation(*tow_rcv)
				} else {
					None
				};

				BlockResult::Ready(ChannelReport{ opt_subframe, opt_observation, new_ionosphere })

			},
			BlockResult::NotReady => {
				// Even if the tracking block isn't ready, we might need to produce an observation
				if s.idx % self.pvt_rate_samples == 0 {
					BlockResult::Ready(ChannelReport{ opt_subframe: None, 
						opt_observation: self.opt_observation(*tow_rcv), new_ionosphere: false })
				} else {
					BlockResult::NotReady
				}
			},
			BlockResult::Err(_) => BlockResult::Err(DSPErr::LossOfLock),
		}

	}

	pub fn opt_observation(&self, rx_tow_sec:f64) -> Option<pvt::Observation> {
		if self.aat.awaiting_acq { None } else {
			if let Some(eph) = self.ephemeris {
				// TODO: account for GPS week rollover possibility
				// TODO: check for ephemeris validity time
				// TODO: consider returning a Result where the Err describes the reason for not producing a position
				let sv_tow_sec:f64 = self.aat.trk.sv_time_of_week();
				let (pos_ecef, sv_clock) = eph.pos_and_clock(sv_tow_sec);
				let carrier_freq_hz:f64 = self.aat.trk.carrier_freq_hz();
				let pseudorange_m:f64 = (rx_tow_sec - sv_tow_sec + sv_clock - eph.t_gd) * C_METERS_PER_SEC;
				let obs = pvt::Observation{ sv_id: self.prn, sv_tow_sec, pseudorange_m, pos_ecef, sv_clock, t_gd: eph.t_gd, carrier_freq_hz };
				Some(obs)
			} else { 
				None
			}
		}		
	}
}

pub fn new_channel(prn:usize, fs:f64, test_stat_threshold:f64, pvt_rate_samples:usize) -> Channel { 
	let symbol:Vec<i8> = gps_l1_ca::signal_modulation::prn_int_sampled(prn, fs);
	let acq = Acquisition::new(symbol, fs, prn, 9, 3, 50.0, test_stat_threshold, 8);
	let trk = tracking::new_2nd_order_tracker(prn, 0.0, fs, 0.0, 0.0);
	let tlm = telemetry_decode::TelemetryDecoder::new();

	let aat = AcquireAndTrack::new(acq, trk);

	Channel { prn, fs, aat, tlm, last_acq_doppler:0.0, last_acq_test_stat: 0.0, last_sample_idx: 0, 
		ephemeris: None, ionosphere: None, last_sf1: None, last_sf2: None, last_sf3: None, pvt_rate_samples }
}
