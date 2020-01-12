
extern crate rustfft;

use std::collections::VecDeque;
use std::f64::consts;

use self::rustfft::num_complex::Complex;

use ::gnss::gps_l1_ca;
use ::DigSigProcErr;

pub struct Tracking {
	carrier: Complex<f64>,
	carrier_inc: Complex<f64>,
	carrier_dphase_rad: f64,
	code_phase: f64,
	code_dphase: f64,
	sum_early:  Complex<f64>,
	sum_prompt: Complex<f64>,
	sum_late:   Complex<f64>,
	prompt_buffer: VecDeque<Complex<f64>>,
	state: TrackingState,
	pub fs:f64,
	pub local_code:Vec<Complex<f64>>,
	pub threshold_carrier_lock_test:f64,
	pub threshold_cn0_snv_db_hz:f64,
	last_cn0_snv_db_hz:f64,
	last_carrier_lock_test:f64,
	last_coh_total_power:f64,
	last_coh_noise_power:f64,
}

#[derive(Debug)]
enum TrackingState {
	WaitingForInitialLockStatus,
	WaitingForFirstTransition,
	Tracking,
}

#[derive(Debug)]
pub enum TrackingResult {
	NotReady,
	Ok{ prompt_i:f64, bit_idx:usize },
	Err(DigSigProcErr),
}

impl Tracking {

	pub fn last_cn0_snv_db_hz(&self) -> f64 { self.last_cn0_snv_db_hz }
	pub fn last_carrier_lock_test(&self) -> f64 { self.last_carrier_lock_test }
	pub fn estimated_snr_coh(&self) -> f64 {
		if self.last_coh_total_power > 0.0 { (self.last_coh_total_power / self.last_coh_noise_power) - 1.0 } 
		else { 0.0 }
	}
	pub fn carrier_freq_hz(&self) -> f64 { (self.carrier_dphase_rad * self.fs) / (2.0 * consts::PI) }
	pub fn carrier_phase_rad(&self) -> f64 { self.carrier.arg() }
	pub fn code_phase_samples(&self) -> f64 { self.code_phase }

	// Public interface
	/// Takes a sample in the form of a tuple of the complex sample itself and the sample number.  Returns a TrackingResult.
	pub fn apply(&mut self, sample:(Complex<f64>, usize)) -> TrackingResult {
		// Remove the carrier and code modulation
		self.carrier = self.carrier * self.carrier_inc;
		let x = sample.0 * self.carrier;

	    let mut idx:f64 = self.code_phase - 0.5;
	    while idx.floor() < 0.0    { idx += 1022.0; }
	    while idx.floor() > 1022.0 { idx -= 1022.0; }
	    self.sum_early  += self.local_code[idx.floor() as usize] * x;

	    idx += 0.5;
	    if idx.floor() > 1022.0 { idx -= 1022.0; }
	    self.sum_prompt += self.local_code[idx.floor() as usize] * x;
		
	    idx += 0.5;
	    if idx.floor() > 1022.0 { idx -= 1022.0; }
	    self.sum_late   += self.local_code[idx.floor() as usize] * x;			
		
		self.code_phase += self.code_dphase;

		// If there's a new prompt value available, do correlation on it and add it to the prompt buffer
		if self.code_phase >= 1023.0 {

			// Update carrier tracking
			let carrier_error:f64 = match (self.sum_prompt.im / self.sum_prompt.re).atan() {
				a if a > ( 0.9 * consts::PI) =>  0.9 * consts::PI,
				a if a < (-0.9 * consts::PI) => -0.9 * consts::PI,
				a                            => a,
			};
			self.carrier_dphase_rad += carrier_error / (self.local_code.len() as f64);
			self.carrier_inc = Complex{ re: self.carrier_dphase_rad.cos(), im: -self.carrier_dphase_rad.sin() };
			self.carrier = self.carrier * Complex{ re: carrier_error.cos(), im: -carrier_error.sin()};
	
			// Update code tracking
	        let carrier_hz = (self.carrier_dphase_rad * self.fs) / (2.0 * consts::PI);
			let radial_velocity_factor = (1.57542e9 + carrier_hz) / 1.57542e9;
			self.code_dphase = (radial_velocity_factor * 1.023e6) / self.fs;
	
			self.code_phase -= 1023.0;
			let code_error = match (self.sum_early.norm() - self.sum_late.norm()) / (4.0*self.sum_early.norm() - 8.0*self.sum_prompt.norm() + 4.0*self.sum_late.norm()) {
				e if e >  0.1 =>  0.1,
				e if e < -0.1 => -0.1,
				e             =>  e,
			};
			self.code_phase += code_error;

			// Add this prompt value to the buffer
			self.prompt_buffer.push_back(self.sum_prompt);

			// Reset the sum accumulators for the next prompt
			self.sum_early  = Complex{ re: 0.0, im: 0.0};
			self.sum_prompt = Complex{ re: 0.0, im: 0.0};
			self.sum_late   = Complex{ re: 0.0, im: 0.0};
		}

		// Match on the current state.
		match self.state {
			TrackingState::WaitingForInitialLockStatus => {
				// Limit the size of the prompt buffer to 20
				// TODO: make this a variable
				while self.prompt_buffer.len() > 20 { self.prompt_buffer.pop_front(); }
				
				if self.prompt_buffer.len() >= 20 { 
					self.state = TrackingState::WaitingForFirstTransition;
				}
				TrackingResult::NotReady
			},
			TrackingState::WaitingForFirstTransition => {
				let (found_transition, back_pos) = match (self.prompt_buffer.front(), self.prompt_buffer.back()) {
					(Some(front), Some(back)) => ((front.re > 0.0) != (back.re > 0.0), back.re > 0.0),
					(_, _) => (false, false)
				};

				if found_transition {
					// We've found the first transition, get rid of everything before the transition
					self.prompt_buffer.retain(|c| (c.re > 0.0) == back_pos);

					if self.prompt_buffer.len() > 0 {
						self.state = TrackingState::Tracking;
					} else {
						panic!("Somehow ended up with an empty prompt buffer after detecting the first transition");
					}
				} 

				TrackingResult::NotReady
			},
			TrackingState::Tracking => {
				if self.prompt_buffer.len() >= 20 { 
					// Save coherent SNR components
					self.last_coh_total_power = (&self.prompt_buffer).into_iter().map(|x| x.norm_sqr()).sum();
					self.last_coh_noise_power = (&self.prompt_buffer).into_iter().map(|x| 2.0 * x.im.powi(2)).sum();

					// Normalize the carrier at the end of every bit, which is every 20 ms
					self.carrier = self.carrier / self.carrier.norm();

					// We have enough prompts to build a bit
					let this_bit_re:f64 = self.prompt_buffer.drain(..20).map(|c| c.re).fold(0.0, |a,b| a+b);
					TrackingResult::Ok{ prompt_i:this_bit_re, bit_idx: sample.1} 
				} else { TrackingResult::NotReady }

			}
		}
		
	}

	pub fn initialize(&mut self, acq_freq_hz:f64) {

		let acq_carrier_rad_per_sec = acq_freq_hz * 2.0 * consts::PI;
		self.carrier            = Complex{ re: 1.0, im: 0.0};
		self.carrier_dphase_rad = acq_carrier_rad_per_sec / self.fs;

		let radial_velocity_factor:f64 = (1.57542e9 + acq_freq_hz) / 1.57542e9;
		self.code_phase = 0.0;
		self.code_dphase = (radial_velocity_factor * 1.023e6) / self.fs;

		self.prompt_buffer.clear();
		self.sum_early  = Complex{ re: 0.0, im: 0.0};
		self.sum_prompt = Complex{ re: 0.0, im: 0.0};
		self.sum_late   = Complex{ re: 0.0, im: 0.0};

		self.state = TrackingState::WaitingForInitialLockStatus;
		self.last_cn0_snv_db_hz = 0.0;
		self.last_carrier_lock_test = 0.0;

		// Leave lock_fail_limit, fs, local_code, threshold_carrier_lock_test, and threshold_cn0_snv_db_hz as is
	}

}

pub fn new_default_tracker(prn:usize, acq_freq_hz:f64, fs:f64) -> Tracking {
	let local_code: Vec<Complex<f64>> = gps_l1_ca::signal_modulation::prn_complex(prn);

	let acq_carrier_rad_per_sec = acq_freq_hz * 2.0 * consts::PI;
	let carrier_dphase_rad:f64 = acq_carrier_rad_per_sec / fs;
	let carrier     = Complex{ re: 1.0, im: 0.0};
	let carrier_inc = Complex{ re: carrier_dphase_rad.cos(), im: -carrier_dphase_rad.sin() };

	let radial_velocity_factor:f64 = (1.57542e9 + acq_freq_hz) / 1.57542e9;
	let code_phase      = 0.0;
	let code_dphase     = (radial_velocity_factor * 1.023e6) / fs;

	Tracking { carrier, carrier_inc, carrier_dphase_rad, 
		code_phase, code_dphase,
		sum_early: Complex{re: 0.0, im: 0.0}, sum_prompt: Complex{re: 0.0, im: 0.0}, sum_late: Complex{re: 0.0, im: 0.0}, 
		prompt_buffer: VecDeque::new(), 
		state: TrackingState::WaitingForInitialLockStatus,
		fs, local_code, threshold_carrier_lock_test: 0.8, threshold_cn0_snv_db_hz: 30.0,
		last_cn0_snv_db_hz: 0.0, last_carrier_lock_test: 0.0, last_coh_total_power: 0.0, last_coh_noise_power: 0.0,
	}		
}