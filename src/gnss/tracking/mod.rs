
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
	carrier_phase: Complex<f64>,		// mutable
	carrier_dphase_rad: f64,
	code_phase: f64,
	code_dphase: f64,
	carrier_filter: filters::FIR,
	code_filter: filters::FIR,
	lock_fail_count: usize,
	lock_fail_limit: usize,
	sample_buffer: VecDeque<Sample>,
	prompt_buffer: VecDeque<(Complex<f64>, usize)>,
	state: TrackingState,
	pub fs:f64,								// immutable
	pub local_code:Vec<Complex<f64>>,
	pub threshold_carrier_lock_test:f64,
	pub threshold_cn0_snv_db_hz:f64,
}

enum TrackingState {
	WaitingForInitialLockStatus,
	WaitingForFirstTransition,
	Tracking,
}

impl Tracking {

	fn cn0_and_tracking_lock_status(&self) -> bool {
		if self.prompt_buffer.len() < 20 { true } else {
			let cn0_snv_db_hz = lock_detectors::cn0_svn_estimator(&self.prompt_buffer, 0.001);
			let carrier_lock_test = lock_detectors::carrier_lock_detector(&self.prompt_buffer);
			(carrier_lock_test >= self.threshold_carrier_lock_test) && (cn0_snv_db_hz >= self.threshold_cn0_snv_db_hz)
		}
	}

	fn next_prn_length(&self) -> usize { ((1023.0 / self.code_dphase) - self.code_phase).floor() as usize }

	/// Checks to see if the buffer currently contains enough samples to produce the next symbol.  If so, returns Some with a tuple
	/// containing the complex samples and the index of the first one.  If not, returns None.
	fn next_prn(&mut self) -> Option<(Vec<Complex<f64>>, usize)> {
		let next_len:usize = self.next_prn_length();
		if self.sample_buffer.len() >= next_len {
			// We have enough samples to produce the next PRN
			let mut this_prn:Vec<Complex<f64>> = vec![];
			if let Some((first_x, first_idx)) = self.sample_buffer.pop_front() {
				this_prn.push(first_x);
				while this_prn.len() < next_len {
					if let Some((x, _)) = self.sample_buffer.pop_front() {
						this_prn.push(x);
					} else {
						panic!("We thought we had enough samples in the buffer when we started building the PRN, but somehow ran out");
					}
				}
				Some((this_prn, first_idx)) 
		
			} else {
				panic!("The buffer length was greater than the required next length, but getting the first element somehow failed");
			}
		} else {
			// Not enough samples in the buffer to produce the next PRN
			None
		}
	
	}

	fn carrier_wipe(&mut self, xin:&Vec<Complex<f64>>) -> Vec<Complex<f64>> {
		let phase_inc:Complex<f64> = Complex{ re: self.carrier_dphase_rad.cos(), im: -self.carrier_dphase_rad.sin() };
		let mut ans:Vec<Complex<f64>> = vec![];
		for x in xin {
			self.carrier_phase = self.carrier_phase * phase_inc;
			ans.push(x * self.carrier_phase);
		}
		self.carrier_phase = self.carrier_phase / self.carrier_phase.norm();

		ans
	}

	fn do_correlation_step(&mut self, xin:&Vec<Complex<f64>>) -> (Complex<f64>, Complex<f64>, Complex<f64>) {
		let carrier_wiped = self.carrier_wipe(xin);
		let mut early:Complex<f64>  = Complex{ re: 0.0, im: 0.0};
		let mut prompt:Complex<f64> = Complex{ re: 0.0, im: 0.0};
		let mut late:Complex<f64>   = Complex{ re: 0.0, im: 0.0};
		for x in carrier_wiped {
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
	/// Takes a sample in the form of a tuple of the complex sample itself and the sample number.  Returns a Result.
	/// If the Result is Err, then lock has been lost.  If the Result is Ok, then we are still tracking the signal and
	/// the Ok will contain an Option.  If the Option is None, then the next prompt value is not ready yet.  If it's
	/// Some, it'll contain a tuple with the next prompt value and the sample index where this symbol starts.
	pub fn apply(&mut self, sample:Sample) -> Result<Option<(Complex<f64>, usize)>, DigSigProcErr> {
		// Start by adding the new sample to the sample buffer
		self.sample_buffer.push_back(sample);

		// If there's a new prompt value available, do correlation on it and add it to the prompt buffer
		if let Some((prn, prn_idx)) = self.next_prn() {
			let (early, prompt, late) = self.do_correlation_step(&prn);

			// Update carrier tracking
			let carrier_error = if prompt.re == 0.0 { 0.0 } else { (prompt.im / prompt.re).atan() / self.fs };
			self.carrier_dphase_rad += self.carrier_filter.apply(&carrier_error);

			let code_error = {
				let e:f64 = early.norm();
				let l:f64 = late.norm();
				if l+e == 0.0 { 0.0 } else { 0.5 * (l-e) / (l+e) }
			};
			self.code_dphase += self.code_filter.apply(&(code_error / self.fs));

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
				Ok(None)
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

				Ok(None)
			},
			TrackingState::Tracking => {
				if self.prompt_buffer.len() > 20 { 
					// We have enough prompts to build a bit
					let first_idx:usize = self.prompt_buffer[0].1;
					let prompts_this_bit:VecDeque<Complex<f64>> = self.prompt_buffer.drain(..20).map(|(c, _)| c).collect(); 

					if !self.cn0_and_tracking_lock_status() { self.lock_fail_count += 1; }
					else if self.lock_fail_count > 0 { self.lock_fail_count -= 1; }

					if self.lock_fail_count > self.lock_fail_limit { 
						Err(DigSigProcErr::LossOfLock) 
					} else {
						let this_bit:Complex<f64> = prompts_this_bit.into_iter().fold(Complex{ re:0.0, im:0.0 }, |a,b| a+b); 
						Ok(Some((this_bit, first_idx))) 
					}
				} else {
					Ok(None)
				}

			}
		}
		
	}

	pub fn carrier_freq_hz(&self) -> f64 { (self.carrier_dphase_rad * self.fs) / (2.0 * consts::PI) }

}

pub fn new_default_tracker(prn:usize, acq_freq_hz:f64, fs:f64, bw_pll_hz:f64, bw_dll_hz:f64) -> Tracking {
	let symbol:Vec<i8> = l1_ca_signal::prn_int(prn);
	let local_code: Vec<Complex<f64>> = symbol.into_iter().map(|b| Complex{ re: b as f64, im: 0.0 }).collect();

	let acq_carrier_rad_per_sec = acq_freq_hz * 2.0 * consts::PI;
	let carrier_phase:Complex<f64> = Complex{ re: 1.0, im: 0.0};
	let carrier_dphase_rad:f64 = acq_carrier_rad_per_sec / fs;

	let radial_velocity_factor:f64 = (1.57542e9 + acq_freq_hz) / 1.57542e9;
	let code_phase = 0.0;
	let code_dphase = (radial_velocity_factor * 1.023e6) / fs;

	let zeta = 0.7;
	let pdi = 0.001;
	let wn_cod = (bw_dll_hz * 8.0 * zeta) / (4.0 * zeta * zeta + 1.0);
	let wn_car = (bw_pll_hz * 8.0 * zeta) / (4.0 * zeta * zeta + 1.0);
	let tau1_cod = 1.0  / (wn_cod * wn_cod);
	let tau1_car = 0.25 / (wn_car * wn_car);
	let tau2_cod = (2.0 * zeta) / wn_cod;
	let tau2_car = (2.0 * zeta) / wn_car;

	let carrier_filter = filters::new_fir(vec![(pdi + 2.0*tau2_car) / (2.0*tau1_car), (pdi - 2.0*tau2_car) / (2.0*tau1_car)]);
	let code_filter    = filters::new_fir(vec![(pdi + 2.0*tau2_cod) / (2.0*tau1_cod), (pdi - 2.0*tau2_cod) / (2.0*tau1_cod)]);

	Tracking { carrier_phase, carrier_dphase_rad, code_phase, code_dphase,
		carrier_filter, code_filter,
		lock_fail_count: 0, lock_fail_limit: 50, 
		sample_buffer: VecDeque::new(), 
		prompt_buffer: VecDeque::new(), 
		state: TrackingState::WaitingForInitialLockStatus,
		fs, local_code, threshold_carrier_lock_test: 0.8, threshold_cn0_snv_db_hz: 30.0
	}		
}