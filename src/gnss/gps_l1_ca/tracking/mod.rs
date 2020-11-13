
use std::f64::consts;

use rustfft::num_complex::Complex;

use crate::Sample;
use crate::{DigSigProcErr as DSPErr};

use crate::block::{BlockFunctionality, BlockResult};

use crate::filters::{ScalarFilter, FirstOrderFIR, SecondOrderFIR, ThirdOrderFIR};
use crate::gnss::common::acquisition::AcquisitionResult;
use crate::gnss::common::tracking::TrackReport;
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

	last_acq_result:AcquisitionResult,

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
	Tracking{ num_short_intervals: u8, filter_rate:u8, cycles_since_upgrade: u8,
		sum_prompt_long: Complex<f64>, sum_prompt_medium: Complex<f64>, 
		input_power_long: f64, test_stat:f64 },
	LostLock,
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct TrackingDebug {
	pub prn:usize,
	pub carrier_re:f64,
	pub carrier_im:f64,
	pub carrier_hz:f64,
	pub correlation_prompt_re:f64,
	pub correlation_prompt_im:f64,
	pub test_stat:f64,
}

impl<A:ScalarFilter, B:ScalarFilter> BlockFunctionality<AcquisitionResult, (), Sample, TrackReport> for Tracking<A, B> {

	fn control(&mut self, acq_result:&AcquisitionResult) -> Result<(), &'static str> {
		self.initialize(acq_result.doppler_hz);
		self.last_acq_result = acq_result.clone();
		Ok(())
	}

	fn apply(&mut self, sample:&Sample) -> BlockResult<TrackReport> {
		if sample.idx >= self.last_acq_result.sample_idx + self.last_acq_result.code_phase {
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

				// Update code tracking
				self.code_phase -= 1023.0;
				let code_error:f64 = {
					let e:f64 = self.sum_early.norm();
					let l:f64 = self.sum_late.norm();
					if l+e == 0.0 { 0.0 } else { 0.5 * (l-e) / (l+e) }
				};
				self.code_dphase += self.code_filter.apply(code_error);
				self.sv_tow_sec_outer.set_clock_rate(self.code_dphase * (self.fs.powi(2) / 1.023e6));

				let (result, opt_next_state) = match self.state {

					TrackingState::WaitingForInitialLockStatus{ ref mut prev_prompt, ref mut prev_test_stat } => {

						// Update carrier tracking; carrier_error has units [radians]
						let carrier_error = if self.sum_prompt.re == 0.0 { 0.0 } else { (self.sum_prompt.im / self.sum_prompt.re).atan() };	
						self.carrier_dphase_rad += self.carrier_filter.apply(carrier_error);
						self.carrier_inc = Complex{ re: self.carrier_dphase_rad.cos(), im: -self.carrier_dphase_rad.sin() };
				
						let test_stat = self.sum_prompt.norm_sqr()  / (self.input_signal_power * self.code_len_samples);

						if *prev_test_stat > SHORT_COH_THRESH_PROMOTE_TO_LONG && test_stat > SHORT_COH_THRESH_PROMOTE_TO_LONG && (prev_prompt.re > 0.0) != (self.sum_prompt.re > 0.0) { 		
							// If the signal is not present, each coherent interval has a 9.9999988871e-01 chance of staying under this threshold
							// If the signal is present,     each coherent interval has a 3.7330000000e-01 chance of staying under this threshold
							// So if the signal is present, it should only take about 10 tries to exceed this threshold
							let next_state = TrackingState::Tracking{ num_short_intervals: 1, filter_rate: 1, cycles_since_upgrade: 0,
								sum_prompt_long: self.sum_prompt, 
								sum_prompt_medium: self.sum_prompt, input_power_long: self.input_signal_power, test_stat };
							(BlockResult::NotReady, Some(next_state))
						} else if test_stat < SHORT_COH_THRESH_LOSS_OF_LOCK {	
							// If the signal is not present, each coherent interval has a 9.974e-04 chance of staying under this threshold
							// If the signal is present,     each coherent interval has a 4.543e-07 chance of staying under this threshold
							// If the signal is not present, we should on average only waste about 1 [sec] trying to track it
							(BlockResult::Err(DSPErr::LossOfLock), Some(TrackingState::LostLock))
						} else {
							*prev_test_stat   = test_stat;
							*prev_prompt      = self.sum_prompt;
							(BlockResult::NotReady, None)						
						}

					},
					TrackingState::Tracking{ ref mut num_short_intervals, ref mut filter_rate, ref mut cycles_since_upgrade,
						ref mut sum_prompt_long, ref mut sum_prompt_medium, 
						ref mut input_power_long, ref mut test_stat } => {

						*num_short_intervals  += 1;
						*cycles_since_upgrade += 1;
						*sum_prompt_long      += self.sum_prompt;
						*sum_prompt_medium    += self.sum_prompt * self.sum_prompt.re.signum();
						*input_power_long     += self.input_signal_power;

						if *num_short_intervals % *filter_rate == 0 {
							// Update carrier tracking; carrier_error has units [radians]
							let carrier_error = if sum_prompt_medium.re == 0.0 { 0.0 } else { (sum_prompt_medium.im / sum_prompt_medium.re).atan() };	
							self.carrier_dphase_rad += self.carrier_filter.apply(carrier_error);
							self.carrier_inc = Complex{ re: self.carrier_dphase_rad.cos(), im: -self.carrier_dphase_rad.sin() };

							*sum_prompt_medium = ZERO;

							if *cycles_since_upgrade > 20 {
								// Upgrade medium coherent tracking
								let opt_next_filter:Option<(u8, f64)> = match *filter_rate {
									 1 => Some(( 2, 0.50)),
									 2 => Some(( 4, 0.25)),
									 4 => Some(( 5, 0.20)),
									 5 => Some((10, 0.10)),
									10 => Some((20, 0.05)),
									 _ => None
								};

								if let Some((next_rate, next_scale)) = opt_next_filter {
									*filter_rate = next_rate;
									self.carrier_filter.scale_coeffs(next_scale);
								}

								*cycles_since_upgrade = 0;
							}
						}
				
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
								// eprintln!("Loss of Lock from Long Coherent State");
								(BlockResult::Err(DSPErr::LossOfLock), Some(TrackingState::LostLock))
							} else { 
								let v = TrackReport { id: self.prn, prompt_i, sample_idx: sample.idx,
									test_stat: self.test_stat(), freq_hz: self.carrier_freq_hz() };
								(BlockResult::Ready(v), None) 
							}
						} 
						else if *num_short_intervals > 20 { panic!("self.num_short_intervals = {}", *num_short_intervals); }
						else                              { (BlockResult::NotReady, None)                                  }
					},
					TrackingState::LostLock => (BlockResult::Err(DSPErr::LossOfLock), None),
				};

				// Reset the short integration accumulators for the next cycle
				self.input_signal_power = 0.0;
				self.sum_early  = ZERO;
				self.sum_prompt = ZERO;
				self.sum_late   = ZERO;

				// Transition state if a state transition is required
				if let Some(next_state) = opt_next_state { self.state = next_state; }
				
				result

			} else { BlockResult::NotReady }
		} else {
			BlockResult::NotReady
		}
	}

}

impl<A: ScalarFilter, B: ScalarFilter> Tracking<A, B> {

	pub fn carrier_freq_hz(&self) -> f64 { (self.carrier_dphase_rad * self.fs) / (2.0 * consts::PI) }
	pub fn carrier_phase_rad(&self) -> f64 { self.carrier.arg() }
	pub fn code_phase_samples(&self) -> f64 { self.code_phase * (self.fs / 1.023e6) }
	pub fn code_dphase(&self) -> f64 { self.code_dphase }
	pub fn test_stat(&self) -> f64 { match self.state {
		TrackingState::Tracking{ num_short_intervals:_, filter_rate:_, cycles_since_upgrade:_,
			sum_prompt_long:_, sum_prompt_medium:_, input_power_long:_, test_stat } => test_stat,
		_ => 0.0,
	}}

	pub fn sv_time_of_week(&self) -> f64 { self.sv_tow_sec_outer.time() }
	pub fn reset_clock(&mut self, t:f64) {
		self.sv_tow_sec_outer.reset(t);
		self.sv_tow_sec_inner.reset(t);
	}

	pub fn debug(&self) -> TrackingDebug {
		TrackingDebug {
			prn: self.prn,
			carrier_re: self.carrier.re,
			carrier_im: self.carrier.im,
			carrier_hz: (self.carrier_dphase_rad * self.fs) / (2.0 * consts::PI),
			correlation_prompt_re: self.sum_prompt.re,
			correlation_prompt_im: self.sum_prompt.im,
			test_stat: self.test_stat(),
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

		self.input_signal_power = 0.0;
		self.sum_early  = ZERO;
		self.sum_prompt = ZERO;
		self.sum_late   = ZERO;

		self.state = TrackingState::WaitingForInitialLockStatus{ prev_prompt: ZERO, prev_test_stat: 0.0 };
		
		// Leave fs and local_code as is
	}

}

pub fn new_tracker<T: ScalarFilter, F: Fn(f64, f64) -> T>(prn:usize, acq_freq_hz:f64, fs:f64, 
	alpha_carrier:f64, alpha_code:f64, f:F) -> Tracking<T, T> {
	
	let local_code: Vec<Complex<f64>> = gps_l1_ca::signal_modulation::prn_complex(prn);
	let code_len_samples: f64 = 0.001 * fs;

	let acq_carrier_rad_per_sec = acq_freq_hz * 2.0 * consts::PI;
	let carrier_dphase_rad:f64 = acq_carrier_rad_per_sec / fs;
	let carrier     = Complex{ re: 1.0, im: 0.0};
	let carrier_inc = Complex{ re: carrier_dphase_rad.cos(), im: -carrier_dphase_rad.sin() };

	let radial_velocity_factor:f64 = (1.57542e9 + acq_freq_hz) / 1.57542e9;
	let code_phase      = 0.0;
	let code_dphase     = (radial_velocity_factor * 1.023e6) / fs;

	let carrier_filter      = f(alpha_carrier, SYMBOL_LEN_SEC);
	let code_filter         = f(alpha_code,    SYMBOL_LEN_SEC);

	let state = TrackingState::WaitingForInitialLockStatus{ prev_prompt: ZERO, prev_test_stat: 0.0 };

	Tracking { 
		code_len_samples, prn, state, fs, local_code, 

		last_acq_result: Default::default(),

		sv_tow_sec_inner: IntegerClock::new(1000.0),
		sv_tow_sec_outer: IntegerClock::new(fs),

		// Carrier and code
		carrier, carrier_inc, carrier_dphase_rad, code_phase, code_dphase, carrier_filter, code_filter, 

		// Used during summation over the short interval
		sum_early: ZERO, sum_prompt: ZERO, sum_late: ZERO, input_signal_power: 0.0,		
	}		
}

pub fn new_1st_order_tracker(prn:usize, acq_freq_hz:f64, fs:f64, alpha_carrier:f64, alpha_code:f64) -> Tracking<FirstOrderFIR, FirstOrderFIR> {

	new_tracker(prn, acq_freq_hz, fs, alpha_carrier, alpha_code, |alpha, dt| {
	
		let alpha_lim:f64 = 
			if      0.667 > alpha { 0.667 } 
			else if alpha > 0.95  { 0.95  } 
			else                  { alpha };

		let k:f64 = (alpha_lim.powi(2) - (0.667_f64).powi(2)).sqrt();
		
		let a0 = (-0.29696 - 0.667*k.powi(2)) / dt;
		let a1 = (  0.3333 +   1.0*k.powi(2)) / dt;
		
		FirstOrderFIR::new(a0/fs, a1/fs)

	})

}

pub fn new_2nd_order_tracker(prn:usize, acq_freq_hz:f64, fs:f64, alpha_carrier:f64, alpha_code:f64) -> Tracking<SecondOrderFIR, SecondOrderFIR> {

	new_tracker(prn, acq_freq_hz, fs, alpha_carrier, alpha_code, |alpha, dt| {
	
		let alpha_lim:f64 = 
			if        0.5 > alpha { 0.5   } 
			else if alpha > 0.95  { 0.95  } 
			else                  { alpha };

		let k:f64 = (alpha_lim.powi(2) - (0.5_f64).powi(2)).sqrt();
		
		let a0 = (0.0625 + 0.5*k.powi(2) + k.powi(4)) / dt;
		let a1 = (  -0.5 - 2.0*k.powi(2))             / dt;
		let a2 = (   0.5 + 2.0*k.powi(2))             / dt;
		
		SecondOrderFIR::new(a0/fs, a1/fs, a2/fs)

	})

}

pub fn new_3rd_order_tracker(prn:usize, acq_freq_hz:f64, fs:f64, alpha_carrier:f64, alpha_code:f64) -> Tracking<ThirdOrderFIR, ThirdOrderFIR> {

	new_tracker(prn, acq_freq_hz, fs, alpha_carrier, alpha_code, |alpha, dt| {
	
		let alpha_lim:f64 = 
			if        0.4 > alpha { 0.4   } 
			else if alpha > 0.95  { 0.95  } 
			else                  { alpha };

		let k:f64 = (alpha_lim.powi(2) - (0.4_f64).powi(2)).sqrt();
		
		let a0 = (-0.01024 - 0.128*k.powi(2) - 0.4*k.powi(4)) / dt;
		let a1 = (   0.128 +  0.96*k.powi(2) +     k.powi(4)) / dt;
		let a2 = (  -0.128 -   2.4*k.powi(2))                 / dt;
		let a3 = (    0.6  +   2.0*k.powi(2))                 / dt;
		eprintln!("a0={:.3}, a1={:.3}, a2={:.3}, a3={:.3}", a0, a1, a2, a3);

		ThirdOrderFIR::new(a0/fs, a1/fs, a2/fs, a3/fs)

	})

}