
use std::f64::consts;
use std::sync::Arc;

use rustfft::FFT;
use rustfft::FFTplanner;
use rustfft::num_complex::Complex;
use rustfft::num_traits::Zero;

use crate::Sample;

pub struct MatchedFilterResult {
	pub doppler_hz:f64,
	pub code_phase:usize,
	pub mf_response:Complex<f64>,
	pub mf_len:usize,
	pub input_power_total:f64
}

impl MatchedFilterResult {

	pub fn test_statistic(&self) -> f64 { self.mf_response.norm_sqr() / (self.input_power_total * (self.mf_len as f64)) }

}

pub struct MatchedFilter {
	pub fs:f64, pub freq_shift:f64,
	pub buffer:Vec<Complex<f64>>,
	pub len_fft:usize,
	pub fft:Arc<dyn FFT<f64>>,
	pub symbol_freq_domain:Vec<Complex<f64>>,
	pub fft_out:  Vec<Complex<f64>>,
	pub ifft:Arc<dyn FFT<f64>>,
	pub ifft_out: Vec<Complex<f64>>,
	pub last_sample_idx: usize,
}

impl MatchedFilter {

	pub fn new(symbol:Vec<i8>, fs:f64, freq_shift:f64) -> Self {

		let len_fft:usize = symbol.len();

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

		let buffer:Vec<Complex<f64>> = vec![Complex::zero()];	// Because we're starting last_sample_idx at zero

		Self { fs, freq_shift,
			buffer, len_fft, fft, symbol_freq_domain, fft_out, ifft, ifft_out,
			last_sample_idx: 0 }
	}

	pub fn provide_sample(&mut self, sample:&Sample) -> Result<(), &str> {
		if sample.idx > self.last_sample_idx {
			self.buffer.push(sample.val);
			self.last_sample_idx = sample.idx;
		}
		Ok(())
	}

	pub fn block_for_result(&mut self) -> Result<Option<MatchedFilterResult>, &str> {
		if self.buffer.len() >= self.len_fft {

			let signal:Vec<Complex<f64>> = self.buffer.drain(..self.len_fft).collect();

			// Try acquiring an SV
			let input_power_total:f64 = signal.iter().map(|c| c.re*c.re + c.im*c.im).sum();

			let mut best_match = MatchedFilterResult{ doppler_hz: self.freq_shift, code_phase: 0, mf_response: Complex{re: 0.0, im: 0.0}, 
				mf_len: self.len_fft, input_power_total };

			// Wipe the carrier off the input signal
			let phase_step_rad:f64 = (-2.0 * consts::PI * self.freq_shift) / self.fs;	// TODO: cache on struct creation
			let mut doppler_wiped_time_domain:Vec<Complex<f64>> = (0..(signal.len()))
				.map(|idx| {
					let phase = phase_step_rad * (idx as f64);
					signal[idx] * Complex{ re: phase.cos(), im: phase.sin() }
				}).collect();

			// Run the forward FFT
			self.fft.process(&mut doppler_wiped_time_domain, &mut self.fft_out);

			// Perform multiplication in the freq domain, which is convolution in the time domain
			let mut convolution_freq_domain:Vec<Complex<f64>> = (&self.fft_out).into_iter()
				.zip((&self.symbol_freq_domain).into_iter())
				.map( |(a,b)| a*b )
				.collect();

			// Run the inverse FFT to get correlation in the time domain
			self.ifft.process(&mut convolution_freq_domain, &mut self.ifft_out);
			self.ifft_out = self.ifft_out.iter().map(|c| c / (self.len_fft as f64)).collect();

			// Find the best result from this frequency
			for (idx, mf_response) in (&self.ifft_out).into_iter().enumerate() {

				// Compare the best result from this frequency to the best result overall
				if best_match.mf_response.norm_sqr() < mf_response.norm_sqr() {
					best_match.code_phase  = idx;
					best_match.mf_response = *mf_response;
				}

			}

			// Return the best match if it meets the threshold
			Ok(Some(best_match))

		} else {
			// Buffer isn't full yet, so there's no result to return
			Ok(None)
		}

	}
}

