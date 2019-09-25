
extern crate rustfft;

use self::rustfft::num_complex::Complex;

use ::DigSigProcErr;
use ::gnss::{tracking, telemetry_decode};
use ::gnss::telemetry_decode::gps::l1_ca_subframe;
use ::utils;

pub const DEFAULT_PLL_BW_HZ:f64 = 40.0;
pub const DEFAULT_DLL_BW_HZ:f64 = 4.0;

type Sample = (Complex<f64>, usize);

pub enum ChannelState {
	PullIn(usize),
	Tracking,
}

#[derive(Debug)]
pub enum ChannelResult {
	NotReady,
	Ok(String, l1_ca_subframe::Subframe, usize),
	Err(DigSigProcErr),
}

pub struct Channel {
	pub prn:usize,
	pub fs:f64,
	pub state:ChannelState,
	trk: tracking::Tracking,
	tlm: telemetry_decode::gps::TelemetryDecoder,
}

impl Channel {

	pub fn carrier_freq_hz(&self) -> f64 { self.trk.carrier_freq_hz() }

	pub fn initialize(&mut self, acq_freq:f64, code_phase:usize) {
		self.state = match code_phase {
			0 => ChannelState::Tracking,
			n => ChannelState::PullIn(n),
		};
		self.trk.initialize(acq_freq);
		self.tlm.initialize();
	}

	pub fn apply(&mut self, s:Sample) -> ChannelResult { match self.state {
		ChannelState::PullIn(n) => {
			self.state = match n {
				1 => ChannelState::Tracking,
				_ => ChannelState::PullIn(n-1),
			};
			ChannelResult::NotReady
		},
		ChannelState::Tracking => { 
			match self.trk.apply(s) {
				tracking::TrackingResult::Ok{bit, bit_idx} => {
					// The tracker has a lock and produced a bit, so pass it into the telemetry decoder and match on the result
					match self.tlm.apply((bit, bit_idx)) {
						Ok(Some((subframe, start_idx))) => {
							// The telemetry decoder successfully decoded a subframe, but we just have a sequence of bits right now.  We need to interpret them.
							match l1_ca_subframe::decode(subframe, start_idx) {
								Ok(sf) => {
									// The bits of this subframe have been successfully interpreted.  Output the results to STDERR and store them in nav_data
									let bytes:Vec<String> = utils::bool_slice_to_byte_vec(&subframe).iter().map(|b| format!("{:02X}", b)).collect();
									ChannelResult::Ok(bytes.join(""), sf, start_idx)
								},
								Err(e) => {
									//self.state = ChannelState::Failed(e);
									ChannelResult::Err(e)
								}
							}
						},
						Ok(None) => ChannelResult::NotReady,
						Err(e) => {
							//self.state = ChannelState::Failed(e);
							ChannelResult::Err(e)
						}
					}					
				},
				tracking::TrackingResult::NotReady => ChannelResult::NotReady,
				tracking::TrackingResult::Err(e) => {
					//self.state = ChannelState::Failed(e);
					ChannelResult::Err(e)
				},
			}
		}
	}}

}

pub fn new_default_channel(prn:usize, fs:f64, acq_freq:f64, code_phase:usize) -> Channel {
	let state = match code_phase {
		0 => ChannelState::Tracking,
		n => ChannelState::PullIn(n),
	};
	let trk = tracking::new_default_tracker(prn, acq_freq, fs, DEFAULT_PLL_BW_HZ, DEFAULT_DLL_BW_HZ);
	let tlm = telemetry_decode::gps::TelemetryDecoder::new();

	Channel{ prn, fs, state, trk, tlm }
}