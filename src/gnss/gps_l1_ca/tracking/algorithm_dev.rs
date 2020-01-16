
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

	early_buffer:  VecDeque<Complex<f64>>,
	prompt_buffer: VecDeque<Complex<f64>>,
	late_buffer:   VecDeque<Complex<f64>>,

	test_stat: f64,
	pub state: TrackingState,
	pub fs:f64,
	pub local_code:Vec<Complex<f64>>,
}

#[derive(Debug)]
pub enum TrackingState {
	SeekingBitTransition,
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
}

impl Tracking {

	pub fn debug(&self) -> TrackingDebug {
		TrackingDebug {
			carrier_re: self.carrier.re,
			carrier_im: self.carrier.im,
			carrier_hz: (self.d_carrier_radians_per_sample * self.fs) / (2.0 * consts::PI),
			correlation_prompt_re: self.correlation_prompt.re,
			correlation_prompt_im: self.correlation_prompt.im,
			test_stat: self.test_stat,
		}
	}

	fn coherent_process(&mut self, early:Complex<f64>, prompt:Complex<f64>, late:Complex<f64>, num_symbols:f64) -> f64 {
		// Update carrier tracking
		let angle_err:f64 = (prompt.im / prompt.re).atan();
		self.d_carrier_radians_per_sample += angle_err / (self.code_len_samples * num_symbols);
		self.carrier_step = Complex{ re: self.d_carrier_radians_per_sample.cos(), im: self.d_carrier_radians_per_sample.sin() };
		self.carrier *= Complex{ re: angle_err.cos(), im: angle_err.sin()};

		// Update code tracking
        let carrier_hz = (self.d_carrier_radians_per_sample * self.fs) / (2.0 * consts::PI);
		let radial_velocity_factor = (1.57542e9 + carrier_hz) / 1.57542e9;
		self.code_dphase = (radial_velocity_factor * 1.023e6) / self.fs;

		let code_phase_correction = (early.norm() - late.norm()) / (4.0*early.norm() - 8.0*prompt.norm() + 4.0*late.norm());
		self.code_phase += code_phase_correction;

		// Envelope detection
		self.test_stat = prompt.norm_sqr() / (self.input_signal_power * self.code_len_samples * num_symbols);
		self.input_signal_power = 0.0;

		// Normalize the carrier at the end of coherent processing interval
		self.carrier = self.carrier.unscale(self.carrier.norm());

		angle_err
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

			self.code_phase -= 1023.0;

			// Add the correlations to the buffers
			self.early_buffer.push_back(self.correlation_early);
			self.prompt_buffer.push_back(self.correlation_prompt);
			self.late_buffer.push_back(self.correlation_late);

			// Reset the sum accumulators for the next prompt
			self.correlation_early  = Complex{ re: 0.0, im: 0.0};
			self.correlation_prompt = Complex{ re: 0.0, im: 0.0};
			self.correlation_late   = Complex{ re: 0.0, im: 0.0};

			// Limit the sizes of the buffers to 20
			while self.early_buffer.len() > 20  { self.early_buffer.pop_front();  }
			while self.prompt_buffer.len() > 20 { self.prompt_buffer.pop_front(); }
			while self.late_buffer.len() > 20   { self.late_buffer.pop_front();   }

			// Match on the current state.
			match self.state {
				TrackingState::SeekingBitTransition => {
		
					let early     = self.early_buffer.back().unwrap().clone();
					let prompt    = self.prompt_buffer.back().unwrap().clone();
					let late      = self.late_buffer.back().unwrap().clone();
					let angle_err = self.coherent_process(early, prompt, late, 1.0);

					if self.test_stat < 0.0005 { self.state = TrackingState::LostLock; }

					// Determine whether to transition to normal tracking state
					let (found_transition, back_pos) = match (self.prompt_buffer.front(), self.prompt_buffer.back()) {
						(Some(front), Some(back)) => ((front.re > 0.0) != (back.re > 0.0), back.re > 0.0),
						(_, _) => (false, false)
					};

					if found_transition && angle_err < 0.01 {
						// We've found the first transition, get rid of everything before the transition
						while self.prompt_buffer.get(0).map(|c| c.re > 0.0) != Some(back_pos) {
							self.early_buffer.pop_front();
							self.prompt_buffer.pop_front();
							self.late_buffer.pop_front();
						}

						self.state = TrackingState::Tracking;
					} 

				},
				TrackingState::Tracking => if self.prompt_buffer.len() >= 20 { 

					let early:Complex<f64>  = self.early_buffer.drain(..).sum();
					let prompt:Complex<f64> = self.prompt_buffer.drain(..).sum();
					let late:Complex<f64>   = self.late_buffer.drain(..).sum();
					self.coherent_process(early, prompt, late, 20.0);

					if self.test_stat < 0.00005 { self.state = TrackingState::LostLock; }

					return TrackingResult::Ok{ prompt_i:prompt.re, bit_idx: sample.1};
				},
				TrackingState::LostLock => { return TrackingResult::Err(DigSigProcErr::LossOfLock); },
			};		
		} 

		// If there's a result ready, it is short-circuited somewhere above with a return statement
		TrackingResult::NotReady

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
		early_buffer: VecDeque::new(), 
		prompt_buffer: VecDeque::new(), 
		late_buffer: VecDeque::new(), 
		test_stat: 0.0,
		state: TrackingState::SeekingBitTransition,
		fs, local_code,
	}		
}