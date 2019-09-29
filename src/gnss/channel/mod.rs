
extern crate rustfft;

use self::rustfft::num_complex::Complex;

use ::DigSigProcErr;
use ::gnss::{acquisition, tracking, telemetry_decode};
use ::gnss::telemetry_decode::gps::l1_ca_subframe;
use ::gnss::gps::l1_ca_signal;
use ::utils;

pub const DEFAULT_PLL_BW_HZ:f64 = 40.0;
pub const DEFAULT_DLL_BW_HZ:f64 = 4.0;
pub const DEFAULT_DOPPLER_STEP_HZ:usize = 50;
pub const DEFAULT_DOPPLER_MAX_HZ:i16 = 10000;
pub const DEFAULT_TEST_STAT_THRESHOLD:f64 = 0.01;

type Sample = (Complex<f64>, usize);

#[derive(Clone, Copy, PartialEq)]
pub enum ChannelState {
	Acquisition,
	PullIn(usize),
	Tracking,
}

#[derive(Debug)]
pub enum ChannelResult {
	NotReady(&'static str),
	Acquisition{ doppler_hz:i16, test_stat:f64 },
	Ok(String, l1_ca_subframe::Subframe, usize),
	Err(DigSigProcErr),
}

pub struct Channel {
	pub prn:usize,
	pub fs:f64,
	state:ChannelState,
	acq: acquisition::Acquisition,
	trk: tracking::Tracking,
	tlm: telemetry_decode::gps::TelemetryDecoder,
	last_acq_doppler:i16,
	last_acq_test_stat:f64,
	last_sample_idx:usize,
}

impl Channel {

	// Read-only getter methods
	pub fn carrier_freq_hz(&self) -> f64 { self.trk.carrier_freq_hz() }
	pub fn last_cn0_snv_db_hz(&self) -> f64 { self.trk.last_cn0_snv_db_hz() }
	pub fn last_carrier_lock_test(&self) -> f64 { self.trk.last_carrier_lock_test() }
	pub fn last_acq_doppler(&self) -> i16 { self.last_acq_doppler }
	pub fn last_acq_test_stat(&self) -> f64 { self.last_acq_test_stat }
	pub fn state(&self) -> ChannelState { self.state }

	pub fn initialize(&mut self, acq_freq:f64, code_phase:usize) {
		self.state = match code_phase {
			0 => ChannelState::Tracking,
			n => ChannelState::PullIn(n),
		};
		self.trk.initialize(acq_freq);
		self.tlm.initialize();
	}

	pub fn apply(&mut self, s:Sample) -> ChannelResult { 
		if s.1 <= self.last_sample_idx && s.1 > 0 { panic!("Somehow got the same sample twice or went backwards"); }
		self.last_sample_idx = s.1;

		match self.state {
			ChannelState::Acquisition => {
				if let Some(r) = self.acq.apply(s.0) {
					self.initialize(r.doppler_hz as f64, r.code_phase);
					self.last_acq_doppler = r.doppler_hz;
					self.last_acq_test_stat = r.test_statistic;
					ChannelResult::Acquisition{ doppler_hz: r.doppler_hz, test_stat: r.test_statistic }
				}
				else { ChannelResult::NotReady("Waiting on acquisition") }
			},
			ChannelState::PullIn(n) => {
				self.state = match n {
					1 => ChannelState::Tracking,
					_ => ChannelState::PullIn(n-1),
				};
				ChannelResult::NotReady("Pulling in signal")
			},
			ChannelState::Tracking => { 
				match self.trk.apply(s) {
					tracking::TrackingResult::Ok{bit, bit_idx} => {
						// The tracker has a lock and produced a bit, so pass it into the telemetry decoder and match on the result
						match self.tlm.apply((bit, bit_idx)) {
							telemetry_decode::gps::TelemetryDecoderResult::Ok(sf, bits, start_idx) => {
								let bytes:Vec<String> = utils::bool_slice_to_byte_vec(&bits).iter().map(|b| format!("{:02X}", b)).collect();
								ChannelResult::Ok(bytes.join(""), sf, start_idx)							
							},
							telemetry_decode::gps::TelemetryDecoderResult::NotReady => ChannelResult::NotReady("Have a new bit, but new subframe not yet ready"),
							telemetry_decode::gps::TelemetryDecoderResult::Err(e) => {
								self.state = ChannelState::Acquisition;
								ChannelResult::Err(e)
							}
						}					
					},
					tracking::TrackingResult::NotReady => ChannelResult::NotReady("Waiting on next bit from tracker"),
					tracking::TrackingResult::Err(e) => {
						self.state = ChannelState::Acquisition;
						ChannelResult::Err(e)
					},
				}
			}
		}
	}

}

pub fn new_default_channel(prn:usize, fs:f64, acq_freq:f64) -> Channel { new_channel(prn, fs, acq_freq, DEFAULT_TEST_STAT_THRESHOLD) }

pub fn new_channel(prn:usize, fs:f64, acq_freq:f64, test_stat:f64) -> Channel {
	let state = ChannelState::Acquisition;
	let symbol:Vec<i8> = l1_ca_signal::prn_int_sampled(prn, fs);
	let acq = acquisition::make_acquisition(symbol, fs, DEFAULT_DOPPLER_STEP_HZ, DEFAULT_DOPPLER_MAX_HZ, test_stat);
	let trk = tracking::new_default_tracker(prn, acq_freq, fs, DEFAULT_PLL_BW_HZ, DEFAULT_DLL_BW_HZ);
	let tlm = telemetry_decode::gps::TelemetryDecoder::new();

	Channel{ prn, fs, state, acq, trk, tlm, last_acq_doppler:0, last_acq_test_stat: 0.0, last_sample_idx: 0 }
}