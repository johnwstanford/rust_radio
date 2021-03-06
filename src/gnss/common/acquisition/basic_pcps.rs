
use std::f64::consts;
use std::sync::Arc;

use rustfft::FFT;
use rustfft::FFTplanner;
use rustfft::num_complex::Complex;
use rustfft::num_traits::Zero;

use crate::{Sample, DigSigProcErr as DSPErr};
use crate::block::{BlockFunctionality, BlockResult};

use super::AcquisitionResult;

pub struct Acquisition {
	pub fs:f64,
	pub prn:usize,
	pub test_statistic_threshold:f64,
	pub doppler_freqs:Vec<f64>,
	pub buffer:Vec<Complex<f64>>,
	pub len_fft:usize,
	pub fft:Arc<dyn FFT<f64>>,
	pub local_code_freq_domain:Vec<Complex<f64>>,
	pub fft_out:  Vec<Complex<f64>>,
	pub ifft:Arc<dyn FFT<f64>>,
	pub ifft_out: Vec<Complex<f64>>,
	pub last_sample_idx: usize,
}

impl BlockFunctionality<(), (), Sample, AcquisitionResult> for Acquisition {

	fn control(&mut self, _:&()) -> Result<(), &'static str> {
		Ok(())
	}

	fn apply(&mut self, input:&Sample) -> BlockResult<AcquisitionResult> {
		self.provide_sample(input).unwrap();
		match self.block_for_result() {
			Ok(Some(result)) => BlockResult::Ready(result),
			Ok(None)         => BlockResult::NotReady,
			Err(e)           => BlockResult::Err(e)
		}
	}

}

impl Acquisition {

	pub fn new(symbol:Vec<Complex<f64>>, fs:f64, prn:usize, test_statistic_threshold:f64, doppler_freqs:Vec<f64>) -> Self {

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

		Self { fs, prn, test_statistic_threshold, doppler_freqs,
			buffer, len_fft, fft, local_code_freq_domain, fft_out, ifft, ifft_out,
			last_sample_idx: 0 }
	}

	pub fn provide_sample(&mut self, sample:&Sample) -> Result<(), DSPErr> {
		if sample.idx > self.last_sample_idx {
			self.buffer.push(sample.val);
			self.last_sample_idx = sample.idx;
		}
		Ok(())
	}

	pub fn block_for_result(&mut self) -> Result<Option<super::AcquisitionResult>, DSPErr> {
		if self.buffer.len() >= self.len_fft {

			let signal:Vec<Complex<f64>> = self.buffer.drain(..self.len_fft).collect();

			// Try acquiring an SV
			let input_power_total:f64 = signal.iter().map(|c| c.re*c.re + c.im*c.im).sum();

			// Based on the assumption that the spacing is equal between frequencies
			// Zero indicates that there is no step because there's only one frequency
			let doppler_step_hz:f64 = if self.doppler_freqs.len() > 1 { self.doppler_freqs[1] - self.doppler_freqs[0] } else { self.doppler_freqs[0] };
			
			let mut best_match = super::AcquisitionResult{ id: self.prn, sample_idx: self.last_sample_idx,
				doppler_hz: 0.0, doppler_step_hz, code_phase: 0, mf_response: Complex{re: 0.0, im: 0.0}, 
				mf_len: self.len_fft, input_power_total };

			// Try every frequency and update best_match every time we find a new best
			for freq in self.doppler_freqs.iter() {
				// Wipe the carrier off the input signal
				let phase_step_rad:f64 = (-2.0 * consts::PI * (*freq)) / self.fs;			
				let mut doppler_wiped_time_domain:Vec<Complex<f64>> = (0..(signal.len()))
					.map(|idx| {
						let phase = phase_step_rad * (idx as f64);
						signal[idx] * Complex{ re: phase.cos(), im: phase.sin() }
					}).collect();

				// Run the forward FFT
				self.fft.process(&mut doppler_wiped_time_domain, &mut self.fft_out);

				// Perform multiplication in the freq domain, which is convolution in the time domain
				let mut convolution_freq_domain:Vec<Complex<f64>> = (&self.fft_out).into_iter()
					.zip((&self.local_code_freq_domain).into_iter())
					.map( |(a,b)| a*b )
					.collect();

				// Run the inverse FFT to get correlation in the time domain
				self.ifft.process(&mut convolution_freq_domain, &mut self.ifft_out);
				self.ifft_out = self.ifft_out.iter().map(|c| c / (self.len_fft as f64)).collect();

				// Find the best result from this frequency
				for (idx, mf_response) in (&self.ifft_out).into_iter().enumerate() {

					// Compare the best result from this frequency to the best result overall
					if best_match.mf_response.norm_sqr() < mf_response.norm_sqr() {
						best_match.doppler_hz = *freq;
						best_match.code_phase  = idx;
						best_match.mf_response = *mf_response;
					}

				}

			}

			// Return the best match if it meets the threshold
			if best_match.test_statistic() > self.test_statistic_threshold { Ok(Some(best_match)) }
			else { Ok(None) }

		} else {
			// Buffer isn't full yet, so there's no result to return
			Ok(None)
		}

	}

}

