
extern crate rustfft;

use self::rustfft::FFTplanner;
use self::rustfft::num_complex::Complex;
use self::rustfft::num_traits::Zero;

pub mod basic_pcps;

#[derive(Debug)]
pub struct AcquisitionResult {
	pub doppler_hz:i16,
	pub code_phase:usize,
	pub test_statistic:f64,
}

pub trait Acquisition {
	fn apply(&mut self, sample:Complex<f64>) -> Option<AcquisitionResult>;
}

pub fn make_acquisition(symbol:Vec<i8>, fs:f64, doppler_step:usize, doppler_max:i16, test_statistic_threshold:f64) -> basic_pcps::Acquisition {

	let len_fft:usize = symbol.len();
	let doppler_freqs:Vec<i16> = (-doppler_max..doppler_max).step_by(doppler_step as usize).collect();

	// Forward FFT
	let mut local_code_time_domain: Vec<Complex<f64>> = symbol.into_iter().map(|b| Complex{ re: b as f64, im: 0.0 }).collect();
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

	let buffer:Vec<Complex<f64>> = vec![];

	basic_pcps::Acquisition{ fs, test_statistic_threshold, doppler_freqs, buffer, len_fft, fft, local_code_freq_domain, fft_out, ifft, ifft_out, skip_count: 0 }
}