
use std::f64::consts;

use ::rustfft::num_complex::Complex;

use crate::{Sample, DigSigProcErr};
use crate::filters::{ScalarFilter, SecondOrderFIR};

pub const DEFAULT_FILTER_B1:f64 = 0.5;
pub const DEFAULT_FILTER_B2:f64 = 0.5;
pub const DEFAULT_FILTER_B3:f64 = 0.5;
pub const DEFAULT_FILTER_B4:f64 = 0.5;

pub const TEST_STAT_THRESH_CL:f64 = 0.00000075;

pub const SYMBOL_LEN_SEC:f64 = 1.5;

// These constants include chips from both symbols, i.e. with interleaved zeros where the chips from the other symbol would go
pub const CHIPS_PER_SEC:f64 = 1.023e6;
pub const CM_LEN_CHIPS:usize = 20460;
pub const CL_LEN_CHIPS:usize = 1534500;

pub const L2_CARRIER_HZ:f64 = 1.2276e9;
pub const FILTER_CYCLES_PER_CL_SYMBOL:usize = 26;

const ZERO:Complex<f64> = Complex{ re: 0.0, im: 0.0 };

pub struct Tracking<A: ScalarFilter, B: ScalarFilter> {
	pub prn:usize,
	pub state: TrackingState,
	pub fs:f64,
	pub local_cl_code:Vec<Complex<f64>>,
	pub local_cm_code:Vec<Complex<f64>>,

	last_test_stat:f64,

	// Carrier and code
	carrier: Complex<f64>,
	carrier_inc: Complex<f64>,
	carrier_dphase_rad: f64,
	code_phase: f64,
	code_dphase: f64,

	carrier_filter: A,
	code_filter: B,

	// Used during summation over CM symbol interval (data demodulation)
	sum_prompt_cm: Complex<f64>,
	num_samples_cm: usize,

	// Used during summation over the short interval (filter processing)
	cycle_start_chips: [f64; FILTER_CYCLES_PER_CL_SYMBOL],
	next_start_index: usize,
	sum_early:  Complex<f64>,
	sum_prompt: Complex<f64>,
	sum_late:   Complex<f64>,

	// Used during summation over the long interval (lock evaluation)
	sum_prompt_long: Complex<f64>,
	input_signal_power: f64,
	test_stat_period_len_samples: f64,

}

#[derive(Debug, Copy, Clone, PartialEq)]
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
	pub fn code_phase_samples(&self) -> f64 { self.code_phase * (self.fs / CHIPS_PER_SEC) }
	pub fn code_dphase(&self) -> f64 { self.code_dphase }
	pub fn test_stat(&self) -> f64 { self.last_test_stat }

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
	pub fn apply(&mut self, sample:&Sample) -> TrackingResult {

		// Increment the carrier and code phase
		self.carrier = self.carrier * self.carrier_inc;
		self.code_phase += self.code_dphase;

		// Remove the carrier from the new sample and accumulate the power sum
		let x = sample.val * self.carrier;
		self.input_signal_power += x.norm_sqr();

		// Integrate early, prompt, and late sums for filter processing
	    let e_idx:usize = if self.code_phase < 0.5 { CL_LEN_CHIPS-1 } else { (self.code_phase - 0.5).floor() as usize };
	    
	    self.sum_early  += self.local_cl_code[e_idx%CL_LEN_CHIPS] * x;
	    self.sum_prompt += self.local_cl_code[(self.code_phase.floor() as usize)%CL_LEN_CHIPS] * x;
	    self.sum_late   += self.local_cl_code[(e_idx+1)%CL_LEN_CHIPS] * x;			
		
		// Integrate long prompt for lock evaluation
	    self.sum_prompt_long += self.local_cl_code[(self.code_phase.floor() as usize)%CL_LEN_CHIPS] * x;

	    // Integrate CM prompt for data demodulation
	    self.sum_prompt_cm += self.local_cm_code[(self.code_phase.floor() as usize)%CM_LEN_CHIPS] * x;
	    self.num_samples_cm += 1;

		if (self.next_start_index > 0 && self.code_phase >= self.cycle_start_chips[self.next_start_index]) || (self.next_start_index == 0 && self.code_phase < self.cycle_start_chips[1]) {
			// End of a short coherent cycle
			// TODO: consider making it possible to change the filter rate while tracking
			self.next_start_index = (self.next_start_index + 1) % FILTER_CYCLES_PER_CL_SYMBOL;

			// Update carrier tracking; carrier_error has units [radians]
			let carrier_error = if self.sum_prompt.re == 0.0 { 0.0 } else { (self.sum_prompt.im / self.sum_prompt.re).atan() };	
			self.carrier_dphase_rad += self.carrier_filter.apply(carrier_error);
			self.carrier_inc = Complex{ re: self.carrier_dphase_rad.cos(), im: -self.carrier_dphase_rad.sin() };
	
			// Update code tracking
			let code_error:f64 = {
				let e:f64 = self.sum_early.norm();
				let l:f64 = self.sum_late.norm();
				if l+e == 0.0 { 0.0 } else { 0.5 * (l-e) / (l+e) }
			};
			self.code_dphase += self.code_filter.apply(code_error);

			// Normalize the carrier at the end of every short coherent cycle
			self.carrier = self.carrier / self.carrier.norm();
	
			// Reset the short integration accumulators for the next cycle
			self.sum_early  = ZERO;
			self.sum_prompt = ZERO;
			self.sum_late   = ZERO;

		}

		let opt_next_state = if self.code_phase >= (CL_LEN_CHIPS as f64) {
			// End of a 1.5-sec CL symbol; perform lock evaluation and state transition (if applicable)

			// Reset code phase
			self.code_phase -= CL_LEN_CHIPS as f64;

			// Calculate test statistic
			self.last_test_stat = self.sum_prompt_long.norm_sqr()  / (self.input_signal_power * self.test_stat_period_len_samples);

			// Reset accumulators for the next long coherent interval
			self.input_signal_power = 0.0;
			self.sum_prompt_long = ZERO;

			// Perform processing based on state
			match self.state {
				// TODO: consider adding a usize to WaitingForInitialLockStatus to keep track of how long we've been trying,
				// then maybe declare a loss of lock if this gets too high
				TrackingState::WaitingForInitialLockStatus => 
					if self.last_test_stat > TEST_STAT_THRESH_CL { Some(TrackingState::Tracking) } else { None },
				TrackingState::Tracking => 
					if self.last_test_stat < TEST_STAT_THRESH_CL { Some(TrackingState::LostLock) } else { None },
				TrackingState::LostLock => 
					None,
			}

		} else { None };
		
		// Transition state if a state transition is required
		if let Some(next_state) = opt_next_state { self.state = next_state; }
		
	    if self.num_samples_cm >= CM_LEN_CHIPS && self.state == TrackingState::Tracking {

	    	let prompt_i:f64 = self.sum_prompt_cm.re;
	    	
	    	self.num_samples_cm -= CM_LEN_CHIPS;
	    	self.sum_prompt_cm = ZERO;
	    	
	    	TrackingResult::Ok{ prompt_i, bit_idx: sample.idx }

	    } 
	    else if self.state == TrackingState::LostLock { TrackingResult::Err(DigSigProcErr::LossOfLock) }
	    else { TrackingResult::NotReady }

	}

	pub fn initialize(&mut self, acq_freq_hz:f64) {

		let acq_carrier_rad_per_sec = acq_freq_hz * 2.0 * consts::PI;
		self.carrier            = Complex{ re: 1.0, im: 0.0};
		self.carrier_dphase_rad = acq_carrier_rad_per_sec / self.fs;

		// The frequency here is changed to 1227.6 MHz
		// The chips still come as the same rate as L1.  It's just that each symbol is more chips
		let radial_velocity_factor:f64 = (L2_CARRIER_HZ + acq_freq_hz) / L2_CARRIER_HZ;
		self.code_phase = 0.0;
		self.code_dphase = (radial_velocity_factor * CHIPS_PER_SEC) / self.fs;

		self.carrier_filter.initialize();
		self.code_filter.initialize();

		self.input_signal_power = 0.0;
		self.sum_early  = ZERO;
		self.sum_prompt = ZERO;
		self.sum_late   = ZERO;

		self.state = TrackingState::WaitingForInitialLockStatus;
		
		// Leave fs and local_code as is
	}

}

pub fn new_default_tracker(prn:usize, acq_freq_hz:f64, fs:f64) -> Tracking<SecondOrderFIR, SecondOrderFIR> {
	// Create CM code and resample
	let mut local_cl_code:Vec<Complex<f64>> = vec![];
	for chip in super::signal_modulation::cl_code(prn).iter() {
		local_cl_code.push(if *chip { Complex{ re:1.0, im:0.0} } else { Complex{ re:-1.0, im:0.0} });
		local_cl_code.push(ZERO);
	}

	// 3.3.2.4 of IS-GPS-200K makes it sound like the first CM chip comes before the first CL chip but
	// it's not completely clear and it seems to work better the other way around
	let mut local_cm_code:Vec<Complex<f64>> = vec![];
	for chip in super::signal_modulation::cm_code(prn).iter() {
		local_cm_code.push(ZERO);
		local_cm_code.push(if *chip { Complex{ re:1.0, im:0.0} } else { Complex{ re:-1.0, im:0.0} });
	}

	let test_stat_period_len_samples:f64 = fs * SYMBOL_LEN_SEC;		// [samples/sec] * [sec]

	let acq_carrier_rad_per_sec = acq_freq_hz * 2.0 * consts::PI;
	let carrier_dphase_rad:f64 = acq_carrier_rad_per_sec / fs;
	let carrier     = Complex{ re: 1.0, im: 0.0};
	let carrier_inc = Complex{ re: carrier_dphase_rad.cos(), im: -carrier_dphase_rad.sin() };

	// The frequency here is changed to 1227.6 MHz
	// The chips still come as the same rate as L1.  It's just that each symbol is more chips
	let radial_velocity_factor:f64 = (L2_CARRIER_HZ + acq_freq_hz) / L2_CARRIER_HZ;
	let code_phase      = 0.0;
	let code_dphase     = (radial_velocity_factor * CHIPS_PER_SEC) / fs;	

	// FIR coefficients for both filters have units of [1 / samples]
	let filter_rate_hz:f64 = (FILTER_CYCLES_PER_CL_SYMBOL as f64) / SYMBOL_LEN_SEC;
	let (b1, b2, b3, b4) = (DEFAULT_FILTER_B1, DEFAULT_FILTER_B2, DEFAULT_FILTER_B3, DEFAULT_FILTER_B4);
	let a0 = (b1*b2*b3*b4) * filter_rate_hz;
	let a1 = -((b1+b2)*b3*b4 + (b3+b4)*b1*b2) * filter_rate_hz;
	let a2 = (b3*b4 + b1*b2 + (b1+b2)*(b3+b4) - 1.0) * filter_rate_hz;
	
	// [chips / symbol] / [cycles / symbol] = [chips / cycle]
	let l2_chips_per_filter_cycle:f64 = (CL_LEN_CHIPS as f64) / (FILTER_CYCLES_PER_CL_SYMBOL as f64);
	let mut cycle_start_chips:[f64; FILTER_CYCLES_PER_CL_SYMBOL] = [0.0; FILTER_CYCLES_PER_CL_SYMBOL];
	for i in 0..cycle_start_chips.len() {
		cycle_start_chips[i] = (i as f64)*l2_chips_per_filter_cycle;
	}

	let carrier_filter = SecondOrderFIR::new(a0/fs, a1/fs, a2/fs);
	let code_filter    = SecondOrderFIR::new(a0/fs, a1/fs, a2/fs);

	let state = TrackingState::WaitingForInitialLockStatus;

	Tracking { 
		prn, state, fs, local_cl_code, local_cm_code,

		last_test_stat: 0.0,

		// Carrier and code
		carrier, carrier_inc, carrier_dphase_rad, code_phase, code_dphase, carrier_filter, code_filter, 
		cycle_start_chips, next_start_index: 1,

		// Used during summation over CM symbol interval (data demodulation)
		sum_prompt_cm: ZERO, num_samples_cm: 0,

		// Used during summation over the short interval (filter processing)
		sum_early: ZERO, sum_prompt: ZERO, sum_late: ZERO, 

		// Used during summation over the long interval (lock evaluation)
		sum_prompt_long: ZERO, input_signal_power: 0.0, test_stat_period_len_samples
	}		

}