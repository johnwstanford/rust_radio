
use std::f64::consts;

use ::rustfft::num_complex::Complex;

use crate::{Sample, DigSigProcErr};
use crate::filters::{ScalarFilter, SecondOrderFIR};
use crate::gnss::gps_l1_ca;
use crate::utils::IntegerClock;

// Design SNR is 0.035 (-14.56 [dB])
// H0 short test_stat follows an exponential distribution w loc=1.38e-09, scale=5.00e-04
// H1 short test_stat follows a beta distribution w a=1.26e+01, b=1.25e+02, loc=-1.81e-03, scale=1.20e-01

// H0 long test_stat follows an exponential distribution w loc=2.27e-09, scale=2.52e-05
// H1 long test_stat follows a beta distribution w a=2.07e+02, b=2.25e+06, loc=-6.96e-04, scale=1.03e+02

// Design scripts in Python repo under commit c61f35a9
// gnss/l1_ca_long_snr_simulation.py used to simulate long coherent test stats
// gnss/l1_ca_snr_simulation.py used to simulate short coherent test stats
// stats/describe.py fits distributions to the JSON output of the other two 
//   scripts and ranks the results by best fit

pub const DEFAULT_FILTER_B1:f64 = 0.5;
pub const DEFAULT_FILTER_B2:f64 = 0.5;
pub const DEFAULT_FILTER_B3:f64 = 0.5;
pub const DEFAULT_FILTER_B4:f64 = 0.5;

pub const SHORT_COH_THRESH_PROMOTE_TO_LONG:f64 = 0.008;
pub const SHORT_COH_THRESH_LOSS_OF_LOCK:f64    = 5.0e-7;
pub const LONG_COH_THRESH_LOSS_OF_LOCK:f64     = 0.001;

pub const SYMBOL_LEN_SEC:f64 = 1.0e-3;

const ZERO:Complex<f64> = Complex{ re: 0.0, im: 0.0 };

// Lock detection
pub struct Tracking<A: ScalarFilter, B: ScalarFilter> {
	code_len_samples: f64,
	pub prn:usize,
	pub state: TrackingState,
	pub fs:f64,
	pub local_code:Vec<Complex<f64>>,

	sv_tow_sec_inner:IntegerClock,
	sv_tow_sec_outer:IntegerClock,

	// Carrier and code
	carrier: Complex<f64>,
	carrier_inc: Complex<f64>,
	carrier_dphase_rad: f64,
	code_phase: f64,
	code_dphase: f64,

	carrier_filter: A,
	code_filter: B,

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

#[cfg(debug_assertions)]
#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct TrackingDebug {
	pub carrier_re:f64,
	pub carrier_im:f64,
	pub carrier_hz:f64,
	pub correlation_prompt_re:f64,
	pub correlation_prompt_im:f64,
	pub test_stat:f64,
}

impl<A: ScalarFilter, B: ScalarFilter> Tracking<A, B> {

	pub fn carrier_freq_hz(&self) -> f64 { (self.carrier_dphase_rad * self.fs) / (2.0 * consts::PI) }
	pub fn carrier_phase_rad(&self) -> f64 { self.carrier.arg() }
	pub fn code_phase_samples(&self) -> f64 { self.code_phase * (self.fs / 1.023e6) }
	pub fn code_dphase(&self) -> f64 { self.code_dphase }
	pub fn test_stat(&self) -> f64 { match self.state {
		TrackingState::Tracking{ num_short_intervals:_, sum_prompt_long:_, input_power_long:_, test_stat } => test_stat,
		_ => 0.0,
	}}

	pub fn sv_time_of_week(&self) -> f64 { self.sv_tow_sec_outer.time() }
	pub fn reset_clock(&mut self, t:f64) {
		self.sv_tow_sec_outer.reset(t);
		self.sv_tow_sec_inner.reset(t);
	}

	#[cfg(debug_assertions)]
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
	pub fn apply(&mut self, sample:&Sample) -> TrackingResult {
		self.sv_tow_sec_outer.inc();

		// Increment the carrier and code phase
		self.carrier = self.carrier * self.carrier_inc;
		self.code_phase += self.code_dphase;

		// Remove the carrier from the new sample and accumulate the power sum
		let x = sample.val * self.carrier;
		self.input_signal_power += x.norm_sqr();

		// Integrate early, prompt, and late sums
	    let e_idx:usize = if self.code_phase < 0.5 { 1022 } else { (self.code_phase - 0.5).floor() as usize };
	    
	    self.sum_early  += self.local_code[e_idx%1023] * x;
	    self.sum_prompt += self.local_code[(self.code_phase.floor() as usize)%1023] * x;
	    self.sum_late   += self.local_code[(e_idx+1)%1023] * x;			
		
		if self.code_phase >= 1023.0 {
			// End of a 1-ms short coherent cycle
			self.sv_tow_sec_inner.inc();
			self.sv_tow_sec_outer.reset(self.sv_tow_sec_inner.time());

			// Update carrier tracking; carrier_error has units [radians]
			let carrier_error = if self.sum_prompt.re == 0.0 { 0.0 } else { (self.sum_prompt.im / self.sum_prompt.re).atan() };	
			self.carrier_dphase_rad += self.carrier_filter.apply(carrier_error);
			self.carrier_inc = Complex{ re: self.carrier_dphase_rad.cos(), im: -self.carrier_dphase_rad.sin() };
	
			#[cfg(debug_assertions)]
			eprintln!("PRN {} carrier update: carrier=({:.3e})+i({:.3e}), err={:.3e} [rad], dphase={:.3e} [rad/sample]", 
				self.prn, self.carrier.re, self.carrier.im, carrier_error, self.carrier_dphase_rad);

			// Update code tracking
			// TODO: try other phase detectors
			self.code_phase -= 1023.0;
			let code_error:f64 = {
				let e:f64 = self.sum_early.norm();
				let l:f64 = self.sum_late.norm();
				if l+e == 0.0 { 0.0 } else { 0.5 * (l-e) / (l+e) }
			};
			self.code_dphase += self.code_filter.apply(code_error);
			self.sv_tow_sec_outer.set_clock_rate(self.code_dphase * (self.fs.powi(2) / 1.023e6));

			#[cfg(debug_assertions)]
			eprintln!("PRN {} code update: e={:.6e}, p={:.6e}, l={:.6e}, dphase={:.6e} [chips/sample]", 
				self.prn, self.sum_early.norm(), self.sum_prompt.norm(), self.sum_late.norm(), self.code_dphase);

			let (result, opt_next_state) = match self.state {
				TrackingState::WaitingForInitialLockStatus{ ref mut prev_prompt, ref mut prev_test_stat } => {

					let test_stat = self.sum_prompt.norm_sqr()  / (self.input_signal_power * self.code_len_samples);

					if *prev_test_stat > SHORT_COH_THRESH_PROMOTE_TO_LONG && test_stat > SHORT_COH_THRESH_PROMOTE_TO_LONG && (prev_prompt.re > 0.0) != (self.sum_prompt.re > 0.0) { 		
						// If the signal is not present, each coherent interval has a 9.9999988871e-01 chance of staying under this threshold
						// If the signal is present,     each coherent interval has a 3.7330000000e-01 chance of staying under this threshold
						// So if the signal is present, it should only take about 10 tries to exceed this threshold
						let next_state = TrackingState::Tracking{ num_short_intervals: 1, sum_prompt_long: self.sum_prompt, input_power_long: self.input_signal_power, test_stat };
						(TrackingResult::NotReady, Some(next_state))
					} else if test_stat < SHORT_COH_THRESH_LOSS_OF_LOCK {	
						// If the signal is not present, each coherent interval has a 9.974e-04 chance of staying under this threshold
						// If the signal is present,     each coherent interval has a 4.543e-07 chance of staying under this threshold
						// If the signal is not present, we should on average only waste about 1 [sec] trying to track it
						(TrackingResult::Err(DigSigProcErr::LossOfLock), Some(TrackingState::LostLock))
					} else {
						*prev_test_stat   = test_stat;
						*prev_prompt      = self.sum_prompt;
						(TrackingResult::NotReady, None)						
					}

				},
				TrackingState::Tracking{ ref mut num_short_intervals, ref mut sum_prompt_long, ref mut input_power_long, ref mut test_stat } => {
					*num_short_intervals += 1;
					*sum_prompt_long     += self.sum_prompt;
					*input_power_long    += self.input_signal_power;

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
						} else { (TrackingResult::Ok{ prompt_i, bit_idx: sample.idx}, None) }
					} 
					else if *num_short_intervals > 20 { panic!("self.num_short_intervals = {}", *num_short_intervals); }
					else                              { (TrackingResult::NotReady, None)                               }
				},
				TrackingState::LostLock => (TrackingResult::Err(DigSigProcErr::LossOfLock), None),
			};

			// Reset the short integration accumulators for the next cycle
			self.input_signal_power = 0.0;
			self.sum_early  = ZERO;
			self.sum_prompt = ZERO;
			self.sum_late   = ZERO;

			// Transition state if a state transition is required
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

pub fn new_default_tracker(prn:usize, acq_freq_hz:f64, fs:f64) -> Tracking<SecondOrderFIR, SecondOrderFIR> {
	let local_code: Vec<Complex<f64>> = gps_l1_ca::signal_modulation::prn_complex(prn);
	let code_len_samples: f64 = 0.001 * fs;

	let acq_carrier_rad_per_sec = acq_freq_hz * 2.0 * consts::PI;
	let carrier_dphase_rad:f64 = acq_carrier_rad_per_sec / fs;
	let carrier     = Complex{ re: 1.0, im: 0.0};
	let carrier_inc = Complex{ re: carrier_dphase_rad.cos(), im: -carrier_dphase_rad.sin() };

	let radial_velocity_factor:f64 = (1.57542e9 + acq_freq_hz) / 1.57542e9;
	let code_phase      = 0.0;
	let code_dphase     = (radial_velocity_factor * 1.023e6) / fs;

	// FIR coefficients for both filters have units of [1 / samples]
	// Prototyped in Python repo on commit ba5ce609149 under controls/pll_state_space_0_3tap_fir.py
	let (b1, b2, b3, b4) = (DEFAULT_FILTER_B1, DEFAULT_FILTER_B2, DEFAULT_FILTER_B3, DEFAULT_FILTER_B4);
	let a0 = (b1*b2*b3*b4) / SYMBOL_LEN_SEC;
	let a1 = -((b1+b2)*b3*b4 + (b3+b4)*b1*b2) / SYMBOL_LEN_SEC;
	let a2 = (b3*b4 + b1*b2 + (b1+b2)*(b3+b4) - 1.0) / SYMBOL_LEN_SEC;

	#[cfg(debug_assertions)]
	eprintln!("Tracker filter coeffs: a0={:.1}/fs, a1={:.1}/fs, a2={:.1}/fs", a0, a1, a2);

	let carrier_filter = SecondOrderFIR::new(a0/fs, a1/fs, a2/fs);
	let code_filter    = SecondOrderFIR::new(a0/fs, a1/fs, a2/fs);

	let state = TrackingState::WaitingForInitialLockStatus{ prev_prompt: ZERO, prev_test_stat: 0.0 };

	Tracking { 
		code_len_samples, prn, state, fs, local_code, 

		sv_tow_sec_inner: IntegerClock::new(1000.0),
		sv_tow_sec_outer: IntegerClock::new(fs),

		// Carrier and code
		carrier, carrier_inc, carrier_dphase_rad, code_phase, code_dphase, carrier_filter, code_filter, 

		// Used during summation over the short interval
		sum_early: ZERO, sum_prompt: ZERO, sum_late: ZERO, input_signal_power: 0.0,		
	}		

}