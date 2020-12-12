
use num_complex::Complex;
use num_traits::Zero;

use rustfft::FFTplanner;

use serde::{Serialize, Deserialize};

pub mod basic_pcps;
pub mod fast_pcps;
pub mod two_stage_pcps;

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct AcquisitionResult {
	pub id:usize,
	pub sample_idx:usize,
	pub doppler_hz:f64,
	pub doppler_step_hz:f64,
	pub code_phase:usize,
	pub mf_response:Complex<f64>,
	pub mf_len:usize,
	pub input_power_total:f64,
}

impl AcquisitionResult {

	pub fn test_statistic(&self) -> f64 { self.mf_response.norm_sqr() / (self.input_power_total * (self.mf_len as f64)) }

}

pub fn make_acquisition(symbol:Vec<Complex<f64>>, fs:f64, prn:usize, n_coarse:usize, n_fine:usize, test_statistic_threshold:f64, n_skip:usize) -> fast_pcps::Acquisition {

	let len_fft:usize = symbol.len();

	// Forward FFT
	let mut local_code_time_domain: Vec<Complex<f64>> = symbol.clone();
	let mut fft_out: Vec<Complex<f64>> = vec![Complex::zero(); len_fft];
	let mut planner = FFTplanner::new(false);
	let fft = planner.plan_fft(len_fft);
	fft.process(&mut local_code_time_domain, &mut fft_out);

	let local_code_freq_domain: Vec<Complex<f64>> = (&fft_out).into_iter().map(|p| p.conj() ).collect();

	// Inverse FFT
	let mut inv_planner = FFTplanner::new(true);
	let ifft = inv_planner.plan_fft(len_fft);
	let mut ifft_out: Vec<Complex<f64>> = vec![Complex::zero(); len_fft];
	ifft.process(&mut fft_out, &mut ifft_out);

	let buffer:Vec<Complex<f64>> = vec![Complex::zero()];	// Because we're starting last_sample_idx at zero

	fast_pcps::Acquisition{ fs, prn, test_statistic_threshold, n_coarse, n_fine, 
		buffer, len_fft, fft, local_code_freq_domain, fft_out, ifft, ifft_out, 
		skip_count: 0, last_sample_idx: 0, fast_freq_inc: -fs / len_fft as f64, n_skip }
}


