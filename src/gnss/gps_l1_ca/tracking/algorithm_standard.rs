
extern crate rustfft;
extern crate serde;

use std::f64::consts;

use self::rustfft::num_complex::Complex;
use self::serde::{Serialize, Deserialize};

use ::filters;
use ::gnss::gps_l1_ca;
use ::DigSigProcErr;

// Design SNR is 0.035 (-14.56 [dB])
// H0 short test_stat follows an exponential distribution w loc=1.38e-09, scale=5.00e-04
// H1 short test_stat follows a beta distribution w a=1.26e+01, b=1.25e+02, loc=-1.81e-03, scale=1.20e-01

// H0 long test_stat follows an exponential distribution w loc=2.27e-09, scale=2.52e-05
// H1 long test_stat follows a beta distribution w a=2.07e+02, b=2.25e+06, loc=-6.96e-04, scale=1.03e+02

pub const SHORT_COH_THRESH_PROMOTE_TO_LONG:f64 = 0.008;
pub const SHORT_COH_THRESH_LOSS_OF_LOCK:f64    = 5.0e-7;
pub const LONG_COH_THRESH_LOSS_OF_LOCK:f64     = 0.001;

const ZERO:Complex<f64> = Complex{ re: 0.0, im: 0.0 };

// Lock detection
pub struct Tracking {
	code_len_samples: f64,
	pub state: TrackingState,
	pub fs:f64,
	pub local_code:Vec<Complex<f64>>,

	// Carrier and code
	carrier: Complex<f64>,
	carrier_inc: Complex<f64>,
	carrier_dphase_rad: f64,
	code_phase: f64,
	code_dphase: f64,

	carrier_filter: filters::SecondOrderFIR,
	code_filter: filters::SecondOrderFIR,

	// Used during summation over the short interval
	sum_early:  Complex<f64>,
	sum_prompt: Complex<f64>,
	sum_late:   Complex<f64>,
	input_signal_power: f64,
}

#[derive(Debug, Copy, Clone)]
pub enum TrackingState {
	WaitingForInitialLockStatus{ prev_prompt: Complex<f64>, prev_test_stat:f64 },
	Tracking{ num_short_intervals: u8, sum_prompt_long: Complex<f64>, input_power_long: f64, test_stat:f64 },
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

	pub fn carrier_freq_hz(&self) -> f64 { (self.carrier_dphase_rad * self.fs) / (2.0 * consts::PI) }
	pub fn carrier_phase_rad(&self) -> f64 { self.carrier.arg() }
	pub fn code_phase_samples(&self) -> f64 { self.code_phase }
	pub fn test_stat(&self) -> f64 { match self.state {
		TrackingState::Tracking{ num_short_intervals:_, sum_prompt_long:_, input_power_long:_, test_stat } => test_stat,
		_ => 0.0,
	}}

	pub fn debug(&self) -> TrackingDebug {
		TrackingDebug {
			carrier_re: self.carrier.re,
			carrier_im: self.carrier.im,
			carrier_hz: (self.carrier_dphase_rad * self.fs) / (2.0 * consts::PI),
			correlation_prompt_re: self.sum_prompt.re,
			correlation_prompt_im: self.sum_prompt.im,
			test_stat: self.test_stat(),
		}
	}

	// Public interface
	/// Takes a sample in the form of a tuple of the complex sample itself and the sample number.  Returns a TrackingResult.
	pub fn apply(&mut self, sample:(Complex<f64>, usize)) -> TrackingResult {
		// Remove the carrier and code modulation
		self.carrier = self.carrier * self.carrier_inc;
		let x = sample.0 * self.carrier;
		self.input_signal_power += x.norm_sqr();

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

		if self.code_phase >= 1023.0 {
			// End of a 1-ms short coherent cycle

			// Update carrier tracking
			let carrier_error = if self.sum_prompt.re == 0.0 { 0.0 } else { (self.sum_prompt.im / self.sum_prompt.re).atan() / self.fs };
			self.carrier_dphase_rad += self.carrier_filter.apply(carrier_error);
			self.carrier_inc = Complex{ re: self.carrier_dphase_rad.cos(), im: -self.carrier_dphase_rad.sin() };
	
			// Update code tracking
			self.code_phase -= 1023.0;
			let code_error = {
				let e:f64 = self.sum_early.norm();
				let l:f64 = self.sum_late.norm();
				if l+e == 0.0 { 0.0 } else { 0.5 * (l-e) / (l+e) }
			};
			self.code_dphase += self.code_filter.apply(code_error / self.fs);

			// Save the values we'll need for long integration and reset the short integration accumulators for the next cycle
			let this_input_signal_power:f64  = self.input_signal_power;
			let this_sum_prompt:Complex<f64> = self.sum_prompt;
			self.input_signal_power = 0.0;
			self.sum_early  = ZERO;
			self.sum_prompt = ZERO;
			self.sum_late   = ZERO;

			let (result, opt_next_state) = match self.state {
				TrackingState::WaitingForInitialLockStatus{ ref mut prev_prompt, ref mut prev_test_stat } => {

					let test_stat = this_sum_prompt.norm_sqr()  / (this_input_signal_power * self.code_len_samples);

					if *prev_test_stat > SHORT_COH_THRESH_PROMOTE_TO_LONG && test_stat > SHORT_COH_THRESH_PROMOTE_TO_LONG && (prev_prompt.re > 0.0) != (this_sum_prompt.re > 0.0) { 		
						// If the signal is not present, each coherent interval has a 9.9999988871e-01 chance of staying under this threshold
						// If the signal is present,     each coherent interval has a 3.7330000000e-01 chance of staying under this threshold
						// So if the signal is present, it should only take about 10 tries to exceed this threshold
						let next_state = TrackingState::Tracking{ num_short_intervals: 1, sum_prompt_long: this_sum_prompt, input_power_long: this_input_signal_power, test_stat };
						(TrackingResult::NotReady, Some(next_state))
					} else if test_stat < SHORT_COH_THRESH_LOSS_OF_LOCK {	
						// If the signal is not present, each coherent interval has a 9.974e-04 chance of staying under this threshold
						// If the signal is present,     each coherent interval has a 4.543e-07 chance of staying under this threshold
						// If the signal is not present, we should on average only waste about 1 [sec] trying to track it
						(TrackingResult::Err(DigSigProcErr::LossOfLock), Some(TrackingState::LostLock))
					} else {
						*prev_test_stat   = test_stat;
						*prev_prompt      = this_sum_prompt;
						(TrackingResult::NotReady, None)						
					}

				},
				TrackingState::Tracking{ ref mut num_short_intervals, ref mut sum_prompt_long, ref mut input_power_long, ref mut test_stat } => {
					*num_short_intervals += 1;
					*sum_prompt_long     += this_sum_prompt;
					*input_power_long    += this_input_signal_power;

					if *num_short_intervals == 20 { 

						// Normalize the carrier at the end of every bit, which is every 20 ms
						self.carrier = self.carrier / self.carrier.norm();
		
						// Check the quality of the lock
						*test_stat = sum_prompt_long.norm_sqr() / (*input_power_long * self.code_len_samples * 20.0);
		
						// Save the value we need for the result, then reset the long accumulators
						let prompt_i:f64     = sum_prompt_long.re;
						*num_short_intervals = 0;
						*sum_prompt_long     = ZERO;
						*input_power_long    = 0.0;

						// Either return an error or the next bit
						if *test_stat < LONG_COH_THRESH_LOSS_OF_LOCK { 	
							// For a long coherent processing interval, we should be over this threshold under H0 or under this
							// threshold with H1 with a vanishingly small likelihood, i.e. this should be a very good indicator of 
							// the lock status without any need for other filtering or anything like that
							(TrackingResult::Err(DigSigProcErr::LossOfLock), Some(TrackingState::LostLock))
						} else { (TrackingResult::Ok{ prompt_i, bit_idx: sample.1}, None) }
					} 
					else if *num_short_intervals > 20 { panic!("self.num_short_intervals = {}", *num_short_intervals); }
					else                              { (TrackingResult::NotReady, None)                               }
				},
				TrackingState::LostLock => (TrackingResult::Err(DigSigProcErr::LossOfLock), None),
			};

			if let Some(next_state) = opt_next_state { self.state = next_state; }
			
			result

		} else { TrackingResult::NotReady }

		
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

		self.input_signal_power = 0.0;
		self.sum_early  = ZERO;
		self.sum_prompt = ZERO;
		self.sum_late   = ZERO;

		self.state = TrackingState::WaitingForInitialLockStatus{ prev_prompt: ZERO, prev_test_stat: 0.0 };
		
		// Leave fs and local_code as is
	}

}

pub fn new_default_tracker(prn:usize, acq_freq_hz:f64, fs:f64, bw_pll_hz:f64, bw_dll_hz:f64) -> Tracking {
	let local_code: Vec<Complex<f64>> = gps_l1_ca::signal_modulation::prn_complex(prn);
	let code_len_samples: f64 = 0.001 * fs;

	let acq_carrier_rad_per_sec = acq_freq_hz * 2.0 * consts::PI;
	let carrier_dphase_rad:f64 = acq_carrier_rad_per_sec / fs;
	let carrier     = Complex{ re: 1.0, im: 0.0};
	let carrier_inc = Complex{ re: carrier_dphase_rad.cos(), im: -carrier_dphase_rad.sin() };

	let radial_velocity_factor:f64 = (1.57542e9 + acq_freq_hz) / 1.57542e9;
	let code_phase      = 0.0;
	let code_dphase     = (radial_velocity_factor * 1.023e6) / fs;

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

	let state = TrackingState::WaitingForInitialLockStatus{ prev_prompt: ZERO, prev_test_stat: 0.0 };

	Tracking { 
		code_len_samples, state, fs, local_code, 

		// Carrier and code
		carrier, carrier_inc, carrier_dphase_rad, code_phase, code_dphase, carrier_filter, code_filter, 

		// Used during summation over the short interval
		sum_early: ZERO, sum_prompt: ZERO, sum_late: ZERO, input_signal_power: 0.0,		
	}		

}