
pub mod track_and_tlm;

extern crate rustfft;
extern crate serde;

use self::rustfft::num_complex::Complex;

use ::DigSigProcErr;
use ::gnss::common::acquisition;
use ::gnss::gps_l1_ca;
use ::gnss::gps_l1_ca::{pvt, telemetry_decode::subframe::Subframe as SF};

pub const DEFAULT_DOPPLER_STEP_HZ:usize = 50;
pub const DEFAULT_DOPPLER_MAX_HZ:i16 = 10000;
pub const DEFAULT_TEST_STAT_THRESHOLD:f64 = 0.01;

type Sample = (Complex<f64>, usize);

#[derive(Debug)]
pub enum ChannelResult {
	NotReady(&'static str),
	Acquisition{ doppler_hz:f64, test_stat:f64 },
	Ok{sf:Option<SF>},
	Err(DigSigProcErr),
}

pub struct Channel<A: acquisition::Acquisition> {
	pub prn: usize,
	pub fs:  f64,
	acq:     A,
	trk_tlm: track_and_tlm::Channel,
}

impl<A: acquisition::Acquisition> Channel<A> {

	// Read-only getter methods
	pub fn carrier_freq_hz(&self) -> f64 { self.trk_tlm.carrier_freq_hz() }
	pub fn test_stat(&self) -> f64 { self.trk_tlm.test_stat() }

	pub fn last_acq_doppler(&self) -> f64 { self.trk_tlm.last_acq_doppler() }
	pub fn last_acq_test_stat(&self) -> f64 { self.trk_tlm.last_acq_test_stat() }
	pub fn state(&self) -> track_and_tlm::ChannelState { self.trk_tlm.state() }
	pub fn calendar_and_ephemeris(&self) -> Option<pvt::CalendarAndEphemeris> { self.trk_tlm.calendar_and_ephemeris() }

	pub fn apply(&mut self, s:Sample) -> ChannelResult { 
		match self.state() {
			track_and_tlm::ChannelState::AwaitingAcquisition => {
				self.acq.provide_sample(s).unwrap();
				if let Ok(Some(r)) = self.acq.block_for_result(self.prn) {
					self.trk_tlm.acquire(r.test_statistic(), r.doppler_hz as f64, r.code_phase);
					ChannelResult::Acquisition{ doppler_hz: r.doppler_hz, test_stat: r.test_statistic() }
				} else {
					ChannelResult::NotReady("Waiting on acquisition")		
				}
			},
			_ => match self.trk_tlm.apply(s) {
				track_and_tlm::ChannelResult::NotReady(s) => ChannelResult::NotReady(s),
				track_and_tlm::ChannelResult::Ok{sf}      => ChannelResult::Ok{ sf },
				track_and_tlm::ChannelResult::Err(e)      => ChannelResult::Err(e)
			}
		}

	}

	pub fn get_observation(&self, rx_time:f64, rx_tow_sec:f64) -> Option<track_and_tlm::ChannelObservation> {
		self.trk_tlm.get_observation(rx_time, rx_tow_sec)
	}

	pub fn with_acq(prn:usize, fs:f64, acq:A) -> Channel<A> {
		let trk_tlm = track_and_tlm::new_channel(prn, fs);
		Channel { prn, fs, acq, trk_tlm }
	}

}

pub fn new_default_channel<A: acquisition::Acquisition>(prn:usize, fs:f64) -> Channel<acquisition::fast_pcps::Acquisition> { 
	new_channel(prn, fs, DEFAULT_TEST_STAT_THRESHOLD) 
}

pub fn new_channel(prn:usize, fs:f64, test_stat:f64) -> Channel<acquisition::fast_pcps::Acquisition> {
	let symbol:Vec<i8> = gps_l1_ca::signal_modulation::prn_int_sampled(prn, fs);
	let acq = acquisition::make_acquisition(symbol, fs, prn, 9, 17, test_stat, 8);
	Channel::with_acq(prn, fs, acq)
}