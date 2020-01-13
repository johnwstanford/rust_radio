
extern crate rustfft;
extern crate serde;

use std::collections::VecDeque;
use std::f64::consts;

use self::rustfft::num_complex::Complex;
use self::serde::{Serialize, Deserialize};

use ::gnss::gps_l1_ca;
use ::DigSigProcErr;

pub struct Tracking {
	carrier: Complex<f64>,
	carrier_step: Complex<f64>,
	d_carrier_radians_per_sample: f64,
	code_len_samples: f64,

	code_phase: f64,
	code_dphase: f64,

	input_signal_power: f64,
	correlation_early:  Complex<f64>,
	correlation_prompt: Complex<f64>,
	correlation_late:   Complex<f64>,

	prompt_buffer: VecDeque<Complex<f64>>,
	test_stat: f64,
	pub state: TrackingState,
	pub fs:f64,
	pub local_code:Vec<Complex<f64>>,
	last_coh_total_power:f64,
	last_coh_noise_power:f64,
}

#[derive(Debug)]
pub enum TrackingState {
	WaitingForInitialLockStatus,
	WaitingForFirstTransition,
	Tracking,
	LostLock,
}

#[derive(Debug)]
pub enum TrackingResult {
	NotReady,
	Ok{ prompt_i:f64, bit_idx:usize },
	Err(DigSigProcErr),
}

#[derive(Debug, Serialize, Deserialize)]
pub struct TrackingDebug {
	pub carrier_re:f64,
	pub carrier_im:f64,
	pub carrier_hz:f64,
	pub correlation_prompt_re:f64,
	pub correlation_prompt_im:f64,
	pub test_stat:f64,
	pub estimated_snr_coh:f64,
}

impl Tracking {

	pub fn estimated_snr_coh(&self) -> f64 {
		if self.last_coh_total_power > 0.0 { (self.last_coh_total_power / self.last_coh_noise_power) - 1.0 } 
		else { 0.0 }
	}

	pub fn debug(&self) -> TrackingDebug {
		TrackingDebug {
			carrier_re: self.carrier.re,
			carrier_im: self.carrier.im,
			carrier_hz: (self.d_carrier_radians_per_sample * self.fs) / (2.0 * consts::PI),
			correlation_prompt_re: self.correlation_prompt.re,
			correlation_prompt_im: self.correlation_prompt.im,
			test_stat: self.test_stat,
			estimated_snr_coh: self.estimated_snr_coh(),
		}
	}

	// Public interface
	/// Takes a sample in the form of a tuple of the complex sample itself and the sample number.  Returns a TrackingResult.
	pub fn apply(&mut self, sample:(Complex<f64>, usize)) -> TrackingResult {
		// Remove the carrier and code modulation
		let x = sample.0 * self.carrier.conj();
		self.input_signal_power += x.norm_sqr();

	    let mut idx:f64 = self.code_phase - 0.5;
	    while idx.floor() < 0.0    { idx += 1022.0; }
	    while idx.floor() > 1022.0 { idx -= 1022.0; }
	    self.correlation_early  += self.local_code[idx.floor() as usize] * x;

	    idx += 0.5;
	    if idx.floor() > 1022.0 { idx -= 1022.0; }
	    self.correlation_prompt += self.local_code[idx.floor() as usize] * x;
		
	    idx += 0.5;
	    if idx.floor() > 1022.0 { idx -= 1022.0; }
	    self.correlation_late   += self.local_code[idx.floor() as usize] * x;			

	    // Step the carrier
		self.carrier = self.carrier * self.carrier_step;

		// Step the code phase		
		self.code_phase += self.code_dphase;

		// If there's a new prompt value available, do correlation on it and add it to the prompt buffer
		if self.code_phase >= 1023.0 {

			// Update carrier tracking
			let angle_err:f64 = match (self.correlation_prompt.im / self.correlation_prompt.re).atan() {
				a if a > ( 0.3 * consts::PI) =>  0.3 * consts::PI,
				a if a < (-0.3 * consts::PI) => -0.3 * consts::PI,
				a                            => a,
			};
			self.d_carrier_radians_per_sample += angle_err / self.code_len_samples;
			self.carrier_step = Complex{ re: self.d_carrier_radians_per_sample.cos(), im: self.d_carrier_radians_per_sample.sin() };
			self.carrier *= Complex{ re: angle_err.cos(), im: angle_err.sin()};
	
			// Update code tracking
	        let carrier_hz = (self.d_carrier_radians_per_sample * self.fs) / (2.0 * consts::PI);
			let radial_velocity_factor = (1.57542e9 + carrier_hz) / 1.57542e9;
			self.code_dphase = (radial_velocity_factor * 1.023e6) / self.fs;
	
			self.code_phase -= 1023.0;
			let code_phase_correction = match (self.correlation_early.norm() - self.correlation_late.norm()) / (4.0*self.correlation_early.norm() - 8.0*self.correlation_prompt.norm() + 4.0*self.correlation_late.norm()) {
				e if e >  0.1 =>  0.1,
				e if e < -0.1 => -0.1,
				e             =>  e,
			};
			self.code_phase += code_phase_correction;

			// Add this prompt value to the buffer
			self.prompt_buffer.push_back(self.correlation_prompt);

			// Record the prompt test statistic
			self.test_stat = self.correlation_prompt.norm_sqr() / (self.input_signal_power * self.code_len_samples);

			// Reset the sum accumulators for the next prompt
			self.correlation_early  = Complex{ re: 0.0, im: 0.0};
			self.correlation_prompt = Complex{ re: 0.0, im: 0.0};
			self.correlation_late   = Complex{ re: 0.0, im: 0.0};
			self.input_signal_power = 0.0;

			// Normalize the carrier after every symbol
			self.carrier.unscale(self.carrier.norm());
		}

		// Match on the current state.
		match self.state {
			TrackingState::WaitingForInitialLockStatus => {

				if self.prompt_buffer.len() >= 40 { 
					// Throw away the first 20 prompts since this is where the prompt state was likely settling on steady state
					self.prompt_buffer.drain(..20);	

					// Begin looking for the first transition marking a bit boundary
					self.state = TrackingState::WaitingForFirstTransition; 
				}

				TrackingResult::NotReady
			},
			TrackingState::WaitingForFirstTransition => {
				let (found_transition, back_positive) = match (self.prompt_buffer.front(), self.prompt_buffer.back()) {
					(Some(front), Some(back)) => ((front.re > 0.0) != (back.re > 0.0), back.re > 0.0),
					(_, _)                    => (false, false)
				};

				if found_transition {
					// We've found the first transition, get rid of everything before the transition
					self.prompt_buffer.retain(|c| (c.re > 0.0) == back_positive);

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

			},
			TrackingState::LostLock => TrackingResult::Err(DigSigProcErr::LossOfLock),
		}
		
	}

}

pub fn new_default_tracker(prn:usize, acq_freq_hz:f64, fs:f64) -> Tracking {
	let local_code: Vec<Complex<f64>> = gps_l1_ca::signal_modulation::prn_complex(prn);

	let d_carrier_cycles_per_sample  = acq_freq_hz / fs;
	let d_carrier_radians_per_sample = d_carrier_cycles_per_sample * 2.0 * consts::PI;

	let carrier      = Complex{ re: 1.0, im: 0.0};
	let carrier_step = Complex{ re: d_carrier_radians_per_sample.cos(), im: d_carrier_radians_per_sample.sin() };

	let radial_velocity_factor:f64 = (1.57542e9 + acq_freq_hz) / 1.57542e9;
	let code_phase      = 0.0;
	let code_dphase     = (radial_velocity_factor * 1.023e6) / fs;

	Tracking { carrier, carrier_step, d_carrier_radians_per_sample, code_len_samples: fs*1.0e-3,
		code_phase, code_dphase, input_signal_power: 0.0,
		correlation_early:  Complex{re: 0.0, im: 0.0}, 
		correlation_prompt: Complex{re: 0.0, im: 0.0}, 
		correlation_late:   Complex{re: 0.0, im: 0.0}, 
		prompt_buffer: VecDeque::new(), test_stat: 0.0,
		state: TrackingState::WaitingForInitialLockStatus,
		fs, local_code, last_coh_total_power: 0.0, last_coh_noise_power: 0.0,
	}		
}