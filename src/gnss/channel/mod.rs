
pub mod track_and_tlm;

extern crate rustfft;
extern crate serde;

use std::rc::Rc;

use self::rustfft::num_complex::Complex;

use ::DigSigProcErr;
use ::gnss::acquisition;
use ::gnss::telemetry_decode::gps::l1_ca_subframe;
use ::gnss::gps::l1_ca_signal;
use ::gnss::pvt;

pub const DEFAULT_DOPPLER_STEP_HZ:usize = 50;
pub const DEFAULT_DOPPLER_MAX_HZ:i16 = 10000;
pub const DEFAULT_TEST_STAT_THRESHOLD:f64 = 0.01;

type Sample = (Complex<f64>, usize);
type SF = l1_ca_subframe::Subframe;

#[derive(Debug)]
pub enum ChannelResult {
	NotReady(&'static str),
	Acquisition{ doppler_hz:i16, test_stat:f64 },
	Ok{sf:Option<SF>},
	Err(DigSigProcErr),
}

pub struct Channel {
	pub prn:usize,
	pub fs:f64,
	acq:     Rc<dyn acquisition::Acquisition>,
	trk_tlm: track_and_tlm::Channel,
}

impl Channel {

	// Read-only getter methods
	pub fn carrier_freq_hz(&self) -> f64 { self.trk_tlm.carrier_freq_hz() }
	pub fn last_cn0_snv_db_hz(&self) -> f64 { self.trk_tlm.last_cn0_snv_db_hz() }
	pub fn last_carrier_lock_test(&self) -> f64 { self.trk_tlm.last_carrier_lock_test() }
	pub fn last_signal_plus_noise_power(&self) -> f64 { self.trk_tlm.last_signal_plus_noise_power() }
	pub fn last_signal_power(&self) -> f64 { self.trk_tlm.last_signal_power() }
	pub fn estimated_snr(&self) -> f64 { self.trk_tlm.estimated_snr() }

	pub fn last_acq_doppler(&self) -> f64 { self.trk_tlm.last_acq_doppler() }
	pub fn last_acq_test_stat(&self) -> f64 { self.trk_tlm.last_acq_test_stat() }
	pub fn state(&self) -> track_and_tlm::ChannelState { self.trk_tlm.state() }
	pub fn calendar_and_ephemeris(&self) -> Option<pvt::CalendarAndEphemeris> { self.trk_tlm.calendar_and_ephemeris() }

	pub fn apply(&mut self, s:Sample) -> ChannelResult { 
		match self.state() {
			track_and_tlm::ChannelState::AwaitingAcquisition => {
				if let Some(a) = Rc::get_mut(&mut self.acq) {
					a.provide_sample(s).unwrap();
					if let Ok(Some(r)) = a.block_for_result(self.prn) {
						self.trk_tlm.acquire(r.test_statistic, r.doppler_hz as f64, r.code_phase);
						ChannelResult::Acquisition{ doppler_hz: r.doppler_hz, test_stat: r.test_statistic }
					} else {
						ChannelResult::NotReady("Unable to borrow acquisition object mutably")		
					}
				}
				else { ChannelResult::NotReady("Waiting on acquisition") }
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

}

pub fn new_default_channel(prn:usize, fs:f64, acq_freq:f64) -> Channel { 
	new_channel(prn, fs, acq_freq, DEFAULT_TEST_STAT_THRESHOLD) 
}

pub fn new_channel(prn:usize, fs:f64, acq_freq:f64, test_stat:f64) -> Channel {
	let symbol:Vec<i8> = l1_ca_signal::prn_int_sampled(prn, fs);
	let acq = Rc::new(acquisition::make_acquisition(symbol, fs, prn, 9, 17, test_stat));
	let trk_tlm = track_and_tlm::new_channel(prn, fs, acq_freq);

	Channel { prn, fs, acq, trk_tlm }
}