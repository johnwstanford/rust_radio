
extern crate rustfft;

use std::collections::VecDeque;
use std::f64::consts;

use self::rustfft::num_complex::Complex;

use ::filters;
use ::gnss::gps::l1_ca_signal;
use ::utils;
use ::DigSigProcErr;

mod lock_detectors;

type Sample = (Complex<f64>, usize);

pub struct Tracking {
	carrier: Complex<f64>,
	carrier_inc: Complex<f64>,
	carrier_dphase_rad: f64,
	code_phase: f64,
	code_dphase: f64,
	next_prn_length: usize,
	carrier_filter: filters::SecondOrderFIR,
	code_filter: filters::SecondOrderFIR,
	lock_fail_count: usize,
	lock_fail_limit: usize,
	sample_buffer: Vec<Sample>,
	prompt_buffer: VecDeque<(Complex<f64>, usize)>,
	state: TrackingState,
	pub fs:f64,								// immutable
	pub local_code:Vec<Complex<f64>>,
	pub threshold_carrier_lock_test:f64,
	pub threshold_cn0_snv_db_hz:f64,
	last_cn0_snv_db_hz:f64,
	last_carrier_lock_test:f64,
	last_signal_plus_noise_power:f64,
	last_signal_power:f64,
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
	pub fn estimated_snr(&self) -> f64 {
		if self.last_signal_power > 0.0 {
			1.0 / ((self.last_signal_plus_noise_power / self.last_signal_power) - 1.0)
		} else { 0.0 }
	}
	pub fn last_signal_plus_noise_power(&self) -> f64 { self.last_signal_plus_noise_power }
	pub fn last_signal_power(&self) -> f64 { self.last_signal_power }
	pub fn carrier_freq_hz(&self) -> f64 { (self.carrier_dphase_rad * self.fs) / (2.0 * consts::PI) }
	pub fn carrier_phase_rad(&self) -> f64 { self.carrier.arg() }
	pub fn code_phase_samples(&self) -> f64 { self.code_phase }

	fn cn0_and_tracking_lock_status(&mut self) -> bool {
		if self.prompt_buffer.len() < 20 { true } else {
			self.last_cn0_snv_db_hz = lock_detectors::cn0_svn_estimator(&self.prompt_buffer, 0.001);
			self.last_carrier_lock_test = lock_detectors::carrier_lock_detector(&self.prompt_buffer);
			(self.last_carrier_lock_test >= self.threshold_carrier_lock_test) && (self.last_cn0_snv_db_hz >= self.threshold_cn0_snv_db_hz)
		}
	}

	/// Checks to see if the buffer currently contains enough samples to produce the next symbol.  If so, returns Some with a tuple
	/// containing the complex samples and the index of the last one.  If not, returns None.
	fn next_prn(&mut self) -> Option<(Vec<Complex<f64>>, usize)> {
		if self.sample_buffer.len() >= self.next_prn_length {
			// We have enough samples to produce the next PRN
			let this_prn:Vec<Complex<f64>> = self.sample_buffer.iter().map(|(c,_)| *c).collect();
			let (_, last_idx) = self.sample_buffer.pop().unwrap();
			self.sample_buffer.clear();

			// Store the signal plus noise power for SNR calculations
			self.last_signal_plus_noise_power = this_prn.iter().map(|c| c.norm().powi(2)).sum::<f64>() / (self.next_prn_length as f64);

			Some((this_prn, last_idx)) 
		} else {
			// Not enough samples in the buffer to produce the next PRN
			None
		}
	
	}

	fn do_correlation_step(&mut self, xin:&Vec<Complex<f64>>) -> (Complex<f64>, Complex<f64>, Complex<f64>) {
		let mut early:Complex<f64>  = Complex{ re: 0.0, im: 0.0};
		let mut prompt:Complex<f64> = Complex{ re: 0.0, im: 0.0};
		let mut late:Complex<f64>   = Complex{ re: 0.0, im: 0.0};
		for x in xin {
			let early_idx:usize  = utils::wrap_floor(self.code_phase - 0.5, 0, 1022);
			let prompt_idx:usize = utils::wrap_floor(self.code_phase      , 0, 1022);
			let late_idx:usize   = utils::wrap_floor(self.code_phase + 0.5, 0, 1022);
			self.code_phase += self.code_dphase;
		    early  += self.local_code[early_idx]  * x;
		    prompt += self.local_code[prompt_idx] * x;
		    late   += self.local_code[late_idx]   * x;			
		}
		while self.code_phase > 511.5 { self.code_phase -= 1023.0; }
		(early, prompt, late)
	}

	// Public interface
	/// Takes a sample in the form of a tuple of the complex sample itself and the sample number.  Returns a TrackingResult.
	pub fn apply(&mut self, sample:Sample) -> TrackingResult {
		// Start by adding the new sample to the sample buffer, after removing the carrier
		self.carrier = self.carrier * self.carrier_inc;
		self.sample_buffer.push((sample.0 * self.carrier, sample.1));

		// If there's a new prompt value available, do correlation on it and add it to the prompt buffer
		if let Some((prn, prn_idx)) = self.next_prn() {
			let (early, prompt, late) = self.do_correlation_step(&prn);

			// Update carrier tracking
			let carrier_error = if prompt.re == 0.0 { 0.0 } else { (prompt.im / prompt.re).atan() / self.fs };
			self.carrier_dphase_rad += self.carrier_filter.apply(carrier_error);
			self.carrier_inc = Complex{ re: self.carrier_dphase_rad.cos(), im: -self.carrier_dphase_rad.sin() };
	
			let code_error = {
				let e:f64 = early.norm();
				let l:f64 = late.norm();
				if l+e == 0.0 { 0.0 } else { 0.5 * (l-e) / (l+e) }
			};
			self.code_dphase += self.code_filter.apply(code_error / self.fs);
			self.next_prn_length = ((1023.0 / self.code_dphase) - self.code_phase).floor() as usize;

			// Add this prompt value to the buffer
			self.prompt_buffer.push_back((prompt, prn_idx))
		}

		// Match on the current state.
		match self.state {
			TrackingState::WaitingForInitialLockStatus => {
				// Limit the size of the prompt buffer to 20
				// TODO: make this a variable
				while self.prompt_buffer.len() > 20 { self.prompt_buffer.pop_front(); }
				
				if self.cn0_and_tracking_lock_status() { 
					self.state = TrackingState::WaitingForFirstTransition;
				}
				TrackingResult::NotReady
			},
			TrackingState::WaitingForFirstTransition => {
				let (found_transition, back_pos) = match (self.prompt_buffer.front(), self.prompt_buffer.back()) {
					(Some((front, _)), Some((back, _))) => ((front.re > 0.0) != (back.re > 0.0), back.re > 0.0),
					(_, _) => (false, false)
				};

				if found_transition {
					// We've found the first transition, get rid of everything before the transition
					self.prompt_buffer.retain(|(c, _)| (c.re > 0.0) == back_pos);

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
					// Normalize the carrier at the end of every PRN symbol, every 1 ms
					self.carrier = self.carrier / self.carrier.norm();

					// Check the quality of the lock
					if !self.cn0_and_tracking_lock_status() { self.lock_fail_count += 1; }
					else if self.lock_fail_count > 0 { self.lock_fail_count -= 1; }

					// Either return an error or the next bit
					if self.lock_fail_count > self.lock_fail_limit { 
						TrackingResult::Err(DigSigProcErr::LossOfLock) 
					} else {
						// We have enough prompts to build a bit
						let last_idx:usize = self.prompt_buffer.back().unwrap().1;
						let this_bit_re:f64 = self.prompt_buffer.drain(..20).map(|(c, _)| c.re).fold(0.0, |a,b| a+b);
						self.last_signal_power = (this_bit_re / (self.fs / 50.0)).powi(2);

						TrackingResult::Ok{ prompt_i:this_bit_re, bit_idx: last_idx} 
					}
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

		self.carrier_filter.initialize();
		self.code_filter.initialize();

		self.lock_fail_count = 0;

		self.sample_buffer.clear();
		self.prompt_buffer.clear();

		self.state = TrackingState::WaitingForInitialLockStatus;
		self.last_cn0_snv_db_hz = 0.0;
		self.last_carrier_lock_test = 0.0;

		// Leave lock_fail_limit, fs, local_code, threshold_carrier_lock_test, and threshold_cn0_snv_db_hz as is
	}

}

pub fn new_default_tracker(prn:usize, acq_freq_hz:f64, fs:f64, bw_pll_hz:f64, bw_dll_hz:f64) -> Tracking {
	let local_code: Vec<Complex<f64>> = l1_ca_signal::prn_complex(prn);

	let acq_carrier_rad_per_sec = acq_freq_hz * 2.0 * consts::PI;
	let carrier_dphase_rad:f64 = acq_carrier_rad_per_sec / fs;
	let carrier     = Complex{ re: 1.0, im: 0.0};
	let carrier_inc = Complex{ re: carrier_dphase_rad.cos(), im: -carrier_dphase_rad.sin() };

	let radial_velocity_factor:f64 = (1.57542e9 + acq_freq_hz) / 1.57542e9;
	let code_phase      = 0.0;
	let code_dphase     = (radial_velocity_factor * 1.023e6) / fs;
	let next_prn_length = (1023.0 / code_dphase).floor() as usize;

	let zeta = 0.7;
	let pdi = 0.001;
	let wn_cod = (bw_dll_hz * 8.0 * zeta) / (4.0 * zeta * zeta + 1.0);
	let wn_car = (bw_pll_hz * 8.0 * zeta) / (4.0 * zeta * zeta + 1.0);
	let tau1_cod = 1.0  / (wn_cod * wn_cod);
	let tau1_car = 0.25 / (wn_car * wn_car);
	let tau2_cod = (2.0 * zeta) / wn_cod;
	let tau2_car = (2.0 * zeta) / wn_car;

	let carrier_filter = filters::new_second_order_fir((pdi + 2.0*tau2_car) / (2.0*tau1_car), (pdi - 2.0*tau2_car) / (2.0*tau1_car));
	let code_filter    = filters::new_second_order_fir((pdi + 2.0*tau2_cod) / (2.0*tau1_cod), (pdi - 2.0*tau2_cod) / (2.0*tau1_cod));

	let sample_buffer = vec![];

	Tracking { carrier, carrier_inc, carrier_dphase_rad, 
		code_phase, code_dphase, next_prn_length,
		carrier_filter, code_filter,
		lock_fail_count: 0, lock_fail_limit: 50, 
		sample_buffer, 
		prompt_buffer: VecDeque::new(), 
		state: TrackingState::WaitingForInitialLockStatus,
		fs, local_code, threshold_carrier_lock_test: 0.8, threshold_cn0_snv_db_hz: 30.0,
		last_cn0_snv_db_hz: 0.0, last_carrier_lock_test: 0.0, last_signal_plus_noise_power: 0.0, last_signal_power: 0.0,
	}		
}