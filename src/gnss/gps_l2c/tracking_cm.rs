
use std::f64::consts;

use ::rustfft::num_complex::Complex;

use crate::{Sample, DigSigProcErr};
use crate::filters::{ScalarFilter, SecondOrderFIR};
use crate::utils::IntegerClock;

// Design SNR is 0.015, -18.24 [dB]
// H0 test_stat for CM follows an exponential distribution w loc=1.99e-09, scale=1.23e-05
// H1 test_stat for CM follows a beta distribution w a=1.79e+01, b=2.93e+01, loc=1.27e-04, scale=2.18e-03
// Designed with scripts in Python repo commit 8602d2a8c
// gnss/data/prn15_l2_cm.json        			Data file with test CM code
// gnss/l2c_snr_simulation.py  		 			Simulation used to generate distribution
// stats/describe.py 					 		Used to fit simulation results to distributions
// gnss/l2c_tracking_test_stat_thresholds.py 	Used to analyze false alarm and miss rates

pub const DEFAULT_FILTER_B1:f64 = 0.5;
pub const DEFAULT_FILTER_B2:f64 = 0.5;
pub const DEFAULT_FILTER_B3:f64 = 0.5;
pub const DEFAULT_FILTER_B4:f64 = 0.5;

// False alarm rate predicted to be one in 3.91e+10
// Miss rate predicted to be one in 1.52e+08
pub const TEST_STAT_THRESH_CM:f64 = 0.0003;

pub const INITIAL_LOCK_ATTEMPTS:usize = 20;

pub const SYMBOL_LEN_SEC:f64 = 20.0e-3;

const ZERO:Complex<f64> = Complex{ re: 0.0, im: 0.0 };

pub struct Tracking<A: ScalarFilter, B: ScalarFilter> {
	code_len_samples: f64,
	pub prn:usize,
	pub state: TrackingState,
	pub fs:f64,
	pub local_code:Vec<Complex<f64>>,

	last_test_stat:f64,

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

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum TrackingState {
	WaitingForInitialLockStatus(usize),
	Tracking,
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
	pub fn test_stat(&self) -> f64 { self.last_test_stat }

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
	    let e_idx:usize = if self.code_phase < 0.5 { 20459 } else { (self.code_phase - 0.5).floor() as usize };
	    
	    self.sum_early  += self.local_code[e_idx%20460] * x;
	    self.sum_prompt += self.local_code[(self.code_phase.floor() as usize)%20460] * x;
	    self.sum_late   += self.local_code[(e_idx+1)%20460] * x;			
		
		if self.code_phase >= 20460.0 {

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
			self.code_phase -= 20460.0;
			let code_error:f64 = {
				let e:f64 = self.sum_early.norm();
				let l:f64 = self.sum_late.norm();
				if l+e == 0.0 { 0.0 } else { 0.5 * (l-e) / (l+e) }
			};
			self.code_dphase += self.code_filter.apply(code_error);
			self.sv_tow_sec_outer.set_clock_rate(self.code_dphase * (self.fs.powi(2) / 1.023e6));

			self.last_test_stat = self.sum_prompt.norm_sqr()  / (self.input_signal_power * self.code_len_samples);

			#[cfg(debug_assertions)]
			eprintln!("PRN {} code update: e={:.6e}, p={:.6e}, l={:.6e}, dphase={:.6e} [chips/sample]\n   test_stat={:.6}", 
				self.prn, self.sum_early.norm(), self.sum_prompt.norm(), self.sum_late.norm(), self.code_dphase, self.last_test_stat);

			let (result, opt_next_state) = match self.state {
				TrackingState::WaitingForInitialLockStatus(mut tries_so_far) => if self.last_test_stat > TEST_STAT_THRESH_CM {
					(TrackingResult::NotReady, Some(TrackingState::Tracking))
				} else {
					tries_so_far += 1;
					if tries_so_far >= INITIAL_LOCK_ATTEMPTS {
						(TrackingResult::Err(DigSigProcErr::LossOfLock), None)
					} else {
						(TrackingResult::NotReady, None)
					}
				},
				TrackingState::Tracking => {

					// Normalize the carrier at the end of every symbol, which is every 20 ms
					self.carrier = self.carrier / self.carrier.norm();
	
					// Save the value we need for the result, then reset the long accumulators
					// TODO: determine whether or not this applies to L2C
					let prompt_i:f64 = self.sum_prompt.re;

					// Either return an error or the next bit
					if self.last_test_stat < TEST_STAT_THRESH_CM { 	
						// For a long coherent processing interval, we should be over this threshold under H0 or under this
						// threshold with H1 with a vanishingly small likelihood, i.e. this should be a very good indicator of 
						// the lock status without any need for other filtering or anything like that
						(TrackingResult::Err(DigSigProcErr::LossOfLock), Some(TrackingState::LostLock))
					} else { (TrackingResult::Ok{ prompt_i, bit_idx: sample.idx }, None) }

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

		// The frequency here is changed to 1227.6 MHz
		// The chips still come as the same rate as L1.  It's just that each symbol is 20x more chips
		let radial_velocity_factor:f64 = (1.2276e9 + acq_freq_hz) / 1.2276e9;
		self.code_phase = 0.0;
		self.code_dphase = (radial_velocity_factor * 1.023e6) / self.fs;

		self.carrier_filter.initialize();
		self.code_filter.initialize();

		self.input_signal_power = 0.0;
		self.sum_early  = ZERO;
		self.sum_prompt = ZERO;
		self.sum_late   = ZERO;

		self.state = TrackingState::WaitingForInitialLockStatus(0);
		
		// Leave fs and local_code as is
	}

}

pub fn new_default_tracker(prn:usize, acq_freq_hz:f64, fs:f64) -> Tracking<SecondOrderFIR, SecondOrderFIR> {
	// Create CM code and resample
	let mut local_code:Vec<Complex<f64>> = vec![];
	for chip in super::signal_modulation::cm_code(prn).iter() {
		// We're just tracking the CM code right now and it's interleaved with the CL code, hence the zero after each chip
		local_code.push(if *chip { Complex{ re:1.0, im:0.0} } else { Complex{ re:-1.0, im:0.0} });
		local_code.push(ZERO);
	}

	let code_len_samples:f64 = fs * super::L2_CM_PERIOD_SEC as f64;		// [samples/sec] * [sec]

	let acq_carrier_rad_per_sec = acq_freq_hz * 2.0 * consts::PI;
	let carrier_dphase_rad:f64 = acq_carrier_rad_per_sec / fs;
	let carrier     = Complex{ re: 1.0, im: 0.0};
	let carrier_inc = Complex{ re: carrier_dphase_rad.cos(), im: -carrier_dphase_rad.sin() };

	// The frequency here is changed to 1227.6 MHz
	// The chips still come as the same rate as L1.  It's just that each symbol is 20x more chips
	let radial_velocity_factor:f64 = (1.2276e9 + acq_freq_hz) / 1.2276e9;
	let code_phase      = 0.0;
	let code_dphase     = (radial_velocity_factor * 1.023e6) / fs;	

	// FIR coefficients for both filters have units of [1 / samples]
	let (b1, b2, b3, b4) = (DEFAULT_FILTER_B1, DEFAULT_FILTER_B2, DEFAULT_FILTER_B3, DEFAULT_FILTER_B4);
	let a0 = (b1*b2*b3*b4) / SYMBOL_LEN_SEC;
	let a1 = -((b1+b2)*b3*b4 + (b3+b4)*b1*b2) / SYMBOL_LEN_SEC;
	let a2 = (b3*b4 + b1*b2 + (b1+b2)*(b3+b4) - 1.0) / SYMBOL_LEN_SEC;

	#[cfg(debug_assertions)]
	eprintln!("Tracker filter coeffs: a0={:.1}/fs, a1={:.1}/fs, a2={:.1}/fs", a0, a1, a2);

	let carrier_filter = SecondOrderFIR::new(a0/fs, a1/fs, a2/fs);
	let code_filter    = SecondOrderFIR::new(a0/fs, a1/fs, a2/fs);

	let state = TrackingState::WaitingForInitialLockStatus(0);

	Tracking { 
		code_len_samples, prn, state, fs, local_code, 

		sv_tow_sec_inner: IntegerClock::new(50.0),		// 1000 [Hz] symbol rate for L1, 50 [Hz] symbol rate for L2
		sv_tow_sec_outer: IntegerClock::new(fs),		// Sample rate is still provided

		last_test_stat: 0.0,

		// Carrier and code
		carrier, carrier_inc, carrier_dphase_rad, code_phase, code_dphase, carrier_filter, code_filter, 

		// Used during summation over the short interval
		sum_early: ZERO, sum_prompt: ZERO, sum_late: ZERO, input_signal_power: 0.0,		
	}		

}