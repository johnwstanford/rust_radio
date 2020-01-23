
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

	// Used to determine whether to transition from short to long integration
	prev_prompt: Complex<f64>,
	prev_input_power: f64,
	prev_test_stat:f64,

	// Used during summation over the long interval
	num_short_intervals: u8,
	sum_prompt_long: Complex<f64>,
	input_power_long: f64,
	pub test_stat:f64,
}

#[derive(Debug)]
pub enum TrackingState {
	WaitingForInitialLockStatus,
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

	pub fn carrier_freq_hz(&self) -> f64 { (self.carrier_dphase_rad * self.fs) / (2.0 * consts::PI) }
	pub fn carrier_phase_rad(&self) -> f64 { self.carrier.arg() }
	pub fn code_phase_samples(&self) -> f64 { self.code_phase }

	pub fn debug(&self) -> TrackingDebug {
		TrackingDebug {
			carrier_re: self.carrier.re,
			carrier_im: self.carrier.im,
			carrier_hz: (self.carrier_dphase_rad * self.fs) / (2.0 * consts::PI),
			correlation_prompt_re: self.sum_prompt.re,
			correlation_prompt_im: self.sum_prompt.im,
			test_stat: self.test_stat,
		}
	}

	fn reset_short_interval_sums(&mut self) {
		self.input_signal_power = 0.0;
		self.sum_early  = ZERO;
		self.sum_prompt = ZERO;
		self.sum_late   = ZERO;
	}

	fn reset_long_interval_sums(&mut self) {
		self.num_short_intervals = 0;
		self.sum_prompt_long     = ZERO;
		self.input_power_long    = 0.0;
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

		// If there's a new prompt value available, do correlation on it and add it to the prompt buffer
		if self.code_phase >= 1023.0 {

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

			// Match on the current state.
			match self.state {
				TrackingState::WaitingForInitialLockStatus => {

					self.test_stat = self.sum_prompt.norm_sqr()  / (self.input_signal_power * self.code_len_samples);

					if self.prev_test_stat > SHORT_COH_THRESH_PROMOTE_TO_LONG && self.test_stat > SHORT_COH_THRESH_PROMOTE_TO_LONG && (self.prev_prompt.re > 0.0) != (self.sum_prompt.re > 0.0) { 		
						// If the signal is not present, each coherent interval has a 9.9999988871e-01 chance of staying under this threshold
						// If the signal is present,     each coherent interval has a 3.7330000000e-01 chance of staying under this threshold
						// So if the signal is present, it should only take about 10 tries to exceed this threshold
						self.num_short_intervals = 1;
						self.sum_prompt_long     = self.sum_prompt;
						self.input_power_long    = self.input_signal_power;
						self.state = TrackingState::Tracking;
					} else if self.test_stat < SHORT_COH_THRESH_LOSS_OF_LOCK {	
						// If the signal is not present, each coherent interval has a 9.974e-04 chance of staying under this threshold
						// If the signal is present,     each coherent interval has a 4.543e-07 chance of staying under this threshold
						// If the signal is not present, we should on average only waste about 1 [sec] trying to track it
						self.state = TrackingState::LostLock;
						self.reset_short_interval_sums();
						return TrackingResult::Err(DigSigProcErr::LossOfLock);
					}

					self.prev_test_stat   = self.test_stat;
					self.prev_prompt      = self.sum_prompt;
					self.prev_input_power = self.input_signal_power;
				},
				TrackingState::Tracking => {
					self.num_short_intervals += 1;
					self.sum_prompt_long     += self.sum_prompt;
					self.input_power_long    += self.input_signal_power;

					if self.num_short_intervals == 20 { 

						// Normalize the carrier at the end of every bit, which is every 20 ms
						self.carrier = self.carrier / self.carrier.norm();
		
						// Check the quality of the lock
						self.test_stat = self.sum_prompt_long.norm_sqr() / (self.input_power_long * self.code_len_samples * 20.0);
		
						let prompt_i:f64 = self.sum_prompt.re;
						self.reset_short_interval_sums();
						self.reset_long_interval_sums();

						// Either return an error or the next bit
						if self.test_stat < LONG_COH_THRESH_LOSS_OF_LOCK { 	
							// For a long coherent processing interval, we should be over this threshold under H0 or under this
							// threshold with H1 with a vanishingly small likelihood, i.e. this should be a very good indicator of 
							// the lock status without any need for other filtering or anything like that
							self.state = TrackingState::LostLock;
							return TrackingResult::Err(DigSigProcErr::LossOfLock);
						} else { 
							return TrackingResult::Ok{ prompt_i, bit_idx: sample.1};
						}
					} else if self.num_short_intervals > 20 { panic!("self.num_short_intervals = {}", self.num_short_intervals); }
				},
				TrackingState::LostLock => {
					self.reset_short_interval_sums();
					self.reset_long_interval_sums();
					return TrackingResult::Err(DigSigProcErr::LossOfLock)
				},
			}

			self.reset_short_interval_sums();
		}

		TrackingResult::NotReady
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

		self.prev_prompt = ZERO;
		self.prev_input_power = 1.0;
		self.prev_test_stat = 0.0;

		self.reset_short_interval_sums();
		self.reset_long_interval_sums();

		self.state = TrackingState::WaitingForInitialLockStatus;
		
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

	Tracking { 
		code_len_samples, state: TrackingState::WaitingForInitialLockStatus, fs, local_code, 

		// Carrier and code
		carrier, carrier_inc, carrier_dphase_rad, code_phase, code_dphase, carrier_filter, code_filter, 

		// Used during summation over the short interval
		sum_early: ZERO, sum_prompt: ZERO, sum_late: ZERO, 
		input_signal_power: 0.0,
		
		// Used to determine whether to transition from short to long integration
		prev_prompt: ZERO, prev_input_power: 1.0, prev_test_stat: 0.0,
	
		// Used during summation over the long interval
		num_short_intervals: 0, sum_prompt_long: ZERO, input_power_long: 1.0, test_stat: 0.0, 
	}		

}