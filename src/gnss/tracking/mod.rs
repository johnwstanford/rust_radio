
extern crate rustfft;

use std::f64::consts;

use self::rustfft::num_complex::Complex;

use ::filters;
use ::gnss::gps::l1_ca_signal;
use ::utils;
use ::DigSigProcErr;

mod lock_detectors;

type Sample = (Complex<f64>, usize);

pub struct Tracking<T: Iterator<Item=Sample>> {
	carrier_phase: Complex<f64>,		// mutable
	carrier_dphase_rad: f64,
	code_phase: f64,
	code_dphase: f64,
	carrier_filter: filters::FIR,
	code_filter: filters::FIR,
	lock_fail_count: usize,
	lock_fail_limit: usize,
	first_transition: bool,
	src: T,
	pub fs:f64,								// immutable
	pub local_code:Vec<Complex<f64>>,
	pub threshold_carrier_lock_test:f64,
	pub threshold_cn0_snv_db_hz:f64,
	pub samples_consumed:usize,
}

impl<T: Iterator<Item=Sample>> Tracking<T> {

	fn cn0_and_tracking_lock_status(&self, prompt_buffer:&Vec<Complex<f64>>) -> bool {
		if prompt_buffer.len() != 20 { true } else {
			let cn0_snv_db_hz = lock_detectors::cn0_svn_estimator(prompt_buffer, 0.001);
			let carrier_lock_test = lock_detectors::carrier_lock_detector(prompt_buffer);
			(carrier_lock_test >= self.threshold_carrier_lock_test) && (cn0_snv_db_hz >= self.threshold_cn0_snv_db_hz)
		}	
	}

	fn next_prn_length(&self) -> usize { ((1023.0 / self.code_dphase) - self.code_phase).floor() as usize }

	fn next_prn(&mut self) -> Result<(Vec<Complex<f64>>, usize), DigSigProcErr> {
		let next_len:usize = self.next_prn_length();
		let mut this_prn:Vec<Complex<f64>> = vec![];
		if let Some((first_x, first_idx)) = self.src.next() {
			this_prn.push(first_x);
			while let Some((x, _)) = self.src.next() {
				this_prn.push(x);
				if this_prn.len() >= next_len { break; }
			}
			if this_prn.len() == next_len { 
				self.samples_consumed += next_len;
				Ok((this_prn, first_idx)) 
			} 
			else { Err(DigSigProcErr::NoSourceData) }
		}
		else { Err(DigSigProcErr::NoSourceData) }
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

	fn next_correlation(&mut self) -> Result<((Complex<f64>, Complex<f64>, Complex<f64>), usize), DigSigProcErr> {
		let (xin, idx) = self.next_prn()?;
		Ok((self.do_correlation_step(&xin), idx))
	}

	fn next_prompt(&mut self) -> Result<(Complex<f64>, usize), DigSigProcErr> {
		let ((early, prompt, late), idx) = self.next_correlation()?;

		// Update carrier tracking
		let carrier_error = if prompt.re == 0.0 { 0.0 } else { (prompt.im / prompt.re).atan() / self.fs };
		self.carrier_dphase_rad += self.carrier_filter.apply(&carrier_error);

		let code_error = {
			let e:f64 = early.norm();
			let l:f64 = late.norm();
			if l+e == 0.0 { 0.0 } else { 0.5 * (l-e) / (l+e) }
		};
		self.code_dphase += self.code_filter.apply(&(code_error / self.fs));

		Ok((prompt, idx))
	}

	fn next_prompt_vec(&mut self, n:usize) -> Result<(Vec<Complex<f64>>, usize), DigSigProcErr> {
		let (first_prompt, first_idx) = self.next_prompt()?;
		let mut ans:Vec<Complex<f64>> = vec![first_prompt];
		while ans.len() < n { ans.push(self.next_prompt()?.0); }
		Ok((ans, first_idx))
	}

	// Public interface
	pub fn next(&mut self) -> Result<(Complex<f64>, usize), DigSigProcErr> {

		let (prompt_buffer, prompt_idx) = if !self.first_transition {
			// Wait until the lock status becomes good
			let mut buff:Vec<Complex<f64>> = self.next_prompt_vec(20)?.0;
			while !self.cn0_and_tracking_lock_status(&buff) { buff = self.next_prompt_vec(20)?.0; }

			let first_prompt:Complex<f64> = self.next_prompt()?.0;

			// Find the first prompt transitions
			let (mut first_transition_prompt, mut first_transition_idx) = self.next_prompt()?;
			while (first_prompt.re > 0.0) == (first_transition_prompt.re > 0.0) {
				let next = self.next_prompt()?;
				first_transition_prompt = next.0;
				first_transition_idx = next.1;
			}

			// After we've found the first transition, set this flag so we don't look again
			self.first_transition = true;

			// After we find the first transition, gather the remaining samples that make up this bit
			let mut buff:Vec<Complex<f64>> = vec![first_transition_prompt];
			for _ in 1..20 { buff.push(self.next_prompt()?.0); }
			(buff, first_transition_idx)
		} else { self.next_prompt_vec(20)? };

		if !self.cn0_and_tracking_lock_status(&prompt_buffer) { self.lock_fail_count += 1; }
		else if self.lock_fail_count > 0 { self.lock_fail_count -= 1; }

		if self.lock_fail_count > self.lock_fail_limit { Err(DigSigProcErr::LossOfLock) }
		else { Ok((prompt_buffer.into_iter().fold(Complex{re: 0.0, im: 0.0}, |a,b| a+b), prompt_idx)) }
	}

	pub fn carrier_freq_hz(&self) -> f64 { (self.carrier_dphase_rad * self.fs) / (2.0 * consts::PI) }

}

pub fn new_default_tracker<T: Iterator<Item=Sample>>(prn:usize, acq_freq_hz:f64, fs:f64, bw_pll_hz:f64, bw_dll_hz:f64, src:T) -> Tracking<T> {
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
		first_transition: false, src, fs, local_code, threshold_carrier_lock_test: 0.8, threshold_cn0_snv_db_hz: 30.0, samples_consumed: 0
	}		
}