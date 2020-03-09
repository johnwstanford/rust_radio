
extern crate rustfft;

use std::f64::consts;
use std::sync::Arc;

use self::rustfft::FFT;
use self::rustfft::num_complex::Complex;

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

impl super::Acquisition for Acquisition {

	fn provide_sample(&mut self, sample:(Complex<f64>, usize)) -> Result<(), &str> {
		if sample.1 > self.last_sample_idx {
			self.buffer.push(sample.0);
			self.last_sample_idx = sample.1;
		}
		Ok(())
	}

	fn block_for_result(&mut self, prn:usize) -> Result<Option<super::AcquisitionResult>, &str> {
		if self.buffer.len() >= self.len_fft && prn == self.prn {

			let signal:Vec<Complex<f64>> = self.buffer.drain(..self.len_fft).collect();

			// Try acquiring an SV
			let input_power_total:f64 = signal.iter().map(|c| c.re*c.re + c.im*c.im).sum();

			// Based on the assumption that the spacing is equal between frequencies
			// Zero indicates that there is no step because there's only one frequency
			let doppler_step_hz:f64 = if self.doppler_freqs.len() > 1 { self.doppler_freqs[1] - self.doppler_freqs[0] } else { self.doppler_freqs[0] };
			
			let mut best_match = super::AcquisitionResult{ doppler_hz: 0.0, doppler_step_hz, code_phase: 0, mf_response: Complex{re: 0.0, im: 0.0}, 
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

