
use std::f64::consts;
use std::sync::Arc;

use rustfft::FFT;
use rustfft::num_complex::Complex;

use crate::Sample;

pub struct Acquisition {
	pub fs:f64,
	pub prn:usize,
	pub test_statistic_threshold:f64,
	pub n_coarse:usize, pub n_fine:usize,
	pub buffer:Vec<Complex<f64>>,
	pub len_fft:usize,
	pub fft:Arc<dyn FFT<f64>>,
	pub local_code_freq_domain:Vec<Complex<f64>>,
	pub fft_out:  Vec<Complex<f64>>,
	pub ifft:Arc<dyn FFT<f64>>,
	pub ifft_out: Vec<Complex<f64>>,
	pub skip_count: usize,
	pub last_sample_idx: usize,
	pub fast_freq_inc:f64,
	pub n_skip:usize,
}

impl super::Acquisition for Acquisition {

	fn provide_sample(&mut self, sample:&Sample) -> Result<(), &str> {
		if sample.idx > self.last_sample_idx {
			self.buffer.push(sample.val);
			self.last_sample_idx = sample.idx;
		}
		Ok(())
	}

	fn block_for_result(&mut self) -> Result<Option<super::AcquisitionResult>, &str> {
		if self.buffer.len() >= self.len_fft {
			self.skip_count += 1;
			if self.skip_count > self.n_skip {
				self.skip_count = 0;

				let signal:Vec<Complex<f64>> = self.buffer.drain(..self.len_fft).collect();

				// Try acquiring an SV
				let input_power_total:f64 = signal.iter().map(|c| c.re*c.re + c.im*c.im).sum();

				let mut best_match = super::AcquisitionResult{ doppler_hz: 0.0, doppler_step_hz: (self.fast_freq_inc.abs()) / (self.n_fine as f64),
					code_phase: 0, mf_response: Complex{re: 0.0, im: 0.0}, mf_len: self.len_fft, input_power_total };

				// Try every frequency and update best_match every time we find a new best
				for fine_idx in 0..self.n_fine {
					let base_freq:f64 = (fine_idx as f64 * self.fast_freq_inc) / (self.n_fine as f64);

					// Wipe the carrier off the input signal
					let phase_step_rad:f64 = (-2.0 * consts::PI * base_freq) / self.fs;			
					let mut doppler_wiped_time_domain:Vec<Complex<f64>> = (0..(signal.len()))
						.map(|idx| {
							let phase = phase_step_rad * (idx as f64);
							signal[idx] * Complex{ re: phase.cos(), im: phase.sin() }
						}).collect();

					// Run the forward FFT
					self.fft.process(&mut doppler_wiped_time_domain, &mut self.fft_out);

					for coarse_idx in (-(self.n_coarse as i32))..=(self.n_coarse as i32) {
						// Use the frequency shift theorem to shift the signal by integer multiples of self.fast_freq_inc
						let mut input_signal_freq_domain = self.fft_out.clone();
						if coarse_idx > 0 {
							for _ in 0..coarse_idx {  
								let x = input_signal_freq_domain.pop().unwrap();
								input_signal_freq_domain.insert(0, x);
							}
						}
						else if coarse_idx < 0 {
							for _ in 0..(-coarse_idx) {  
								let x = input_signal_freq_domain.remove(0);
								input_signal_freq_domain.push(x);
							}

						}

						// Perform multiplication in the freq domain, which is convolution in the time domain
						let mut convolution_freq_domain:Vec<Complex<f64>> = input_signal_freq_domain.into_iter()
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
								best_match.doppler_hz  = base_freq + ((coarse_idx as f64)*self.fast_freq_inc);
								best_match.code_phase  = idx;
								best_match.mf_response = *mf_response;
							}

						}

					}

				}

				// Return the best match if it meets the threshold
				if best_match.test_statistic() > self.test_statistic_threshold { Ok(Some(best_match)) }
				else { Ok(None) }

			} else {
				// Clear the buffer for next time
				self.buffer.drain(..self.len_fft);

				Ok(None)
			}

		} else {
			// Buffer isn't full yet, so there's no result to return
			Ok(None)
		}

	}

}

