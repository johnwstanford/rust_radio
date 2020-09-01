
use std::f64::consts;
use std::sync::Arc;

use rustfft::FFT;
use rustfft::FFTplanner;
use rustfft::num_complex::Complex;
use rustfft::num_traits::Zero;

use serde::{Serialize, Deserialize};

use crate::Sample;

pub struct MatchedFilterResult {
	pub doppler_hz:f64,
	pub input_power_total:f64,
	pub response:Vec<Complex<f64>>
}

#[derive(Serialize, Deserialize)]
pub struct MatchedFilterTestStatResult {
	pub max_idx:usize,
	pub test_stat:f64,
}

impl MatchedFilterResult {

	pub fn test_statistic(&self) -> MatchedFilterTestStatResult { 
		// Find the best result
		let (max_idx, max_response) = (&self.response).into_iter().enumerate().max_by_key(|(_, resp)| (resp.norm_sqr() * 10000.0) as usize ).unwrap();

		let test_stat:f64 = max_response.norm_sqr() / (self.input_power_total * (self.response.len() as f64));

		MatchedFilterTestStatResult{ max_idx, test_stat }
	}

}

pub struct MatchedFilter {
	// Specified during struct creation
	pub fs:f64, pub freq_shift:f64,

	// Derived from arguments to struct creation that remain constant after being calculated once
	pub len_fft:usize,
	pub carrier_inc:Complex<f64>,
	pub symbol_freq_domain:Vec<Complex<f64>>,

	// Updated on every sample
	pub buffer:Vec<Complex<f64>>,
	pub carrier:Complex<f64>,

	// Used once the buffer is full
	pub fft:Arc<dyn FFT<f64>>,
	pub fft_out:  Vec<Complex<f64>>,
	pub ifft:Arc<dyn FFT<f64>>,
	pub ifft_out: Vec<Complex<f64>>,
}

impl MatchedFilter {

	pub fn new(symbol:Vec<i8>, fs:f64, freq_shift:f64) -> Self {

		let len_fft:usize = symbol.len();
		let phase_step_rad:f64 = (-2.0 * consts::PI * freq_shift) / fs;
		let carrier_inc = Complex{ re: phase_step_rad.cos(), im: phase_step_rad.sin() };

		// Forward FFT
		let mut symbol_time_domain: Vec<Complex<f64>> = symbol.into_iter().map(|b| Complex{ re: b as f64, im: 0.0 }).collect();
		let mut fft_out: Vec<Complex<f64>> = vec![Complex::zero(); len_fft];
		let mut planner = FFTplanner::new(false);
		let fft = planner.plan_fft(len_fft);
		fft.process(&mut symbol_time_domain, &mut fft_out);

		let symbol_freq_domain: Vec<Complex<f64>> = (&fft_out).into_iter().map(|p| p.conj() ).collect();

		// Inverse FFT
		let mut inv_planner = FFTplanner::new(true);
		let ifft = inv_planner.plan_fft(len_fft);
		let mut ifft_out: Vec<Complex<f64>> = vec![Complex::zero(); len_fft];
		ifft.process(&mut fft_out, &mut ifft_out);

		let buffer:Vec<Complex<f64>> = vec![];
		let carrier = Complex{ re: 1.0, im: 0.0 };

		Self { fs, freq_shift, len_fft, carrier_inc, symbol_freq_domain, buffer, carrier, fft, fft_out, ifft, ifft_out}
	}

	pub fn apply(&mut self, sample:&Sample) -> Option<MatchedFilterResult> {
		self.buffer.push(sample.val * self.carrier);
		self.carrier = self.carrier * self.carrier_inc;

		if self.buffer.len() >= self.len_fft {

			// Normalize carrier
			self.carrier = self.carrier / self.carrier.norm();

			// Drain signal from buffer and find total power
			let mut signal:Vec<Complex<f64>> = self.buffer.drain(..self.len_fft).collect();

			let input_power_total:f64 = signal.iter().map(|c| c.re*c.re + c.im*c.im).sum();

			// Run the forward FFT
			self.fft.process(&mut signal, &mut self.fft_out);

			// Perform multiplication in the freq domain, which is convolution in the time domain
			let mut convolution_freq_domain:Vec<Complex<f64>> = (&self.fft_out).into_iter()
				.zip((&self.symbol_freq_domain).into_iter())
				.map( |(a,b)| a*b )
				.collect();

			// Run the inverse FFT to get correlation in the time domain
			self.ifft.process(&mut convolution_freq_domain, &mut self.ifft_out);
			
			let ans = MatchedFilterResult {
				response: self.ifft_out.iter().map(|c| c / (self.len_fft as f64)).collect(),
				doppler_hz: self.freq_shift,
				input_power_total
			};

			Some(ans)

		} else {
			// Buffer isn't full yet, so there's no result to return
			None
		}

	}
}

