
extern crate rustfft;

use std::f64::consts;
use std::sync::Arc;

use self::rustfft::FFT;
use self::rustfft::num_complex::Complex;

const N_SKIP:usize = 9;

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
			self.skip_count += 1;
			if self.skip_count >= N_SKIP {
				self.skip_count = 0;

				let signal:Vec<Complex<f64>> = self.buffer.drain(..self.len_fft).collect();

				// Try acquiring an SV
				let input_power_total:f64 = signal.iter().map(|c| c.re*c.re + c.im*c.im).sum();
				let input_power_avg:f64 = input_power_total / (self.len_fft as f64);

				let mut best_match = super::AcquisitionResult{ doppler_hz: 0.0, code_phase: 0, test_statistic: 0.0 };

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

						// Normalize, enumerate, and sort the magnitudes
						let mut magnitudes:Vec<(usize, f64)> = (&self.ifft_out).into_iter()
							.map(|c| (c.re*c.re + c.im*c.im))
							.enumerate().collect();
						magnitudes.sort_by(|a,b| b.1.partial_cmp(&a.1).unwrap() );

						// Compare the best result from this iteration to the overall best result
						let (best_idx_this_doppler, best_test_stat_raw_this_doppler) = magnitudes[0];
						let best_test_stat_this_doppler:f64 = best_test_stat_raw_this_doppler / (input_power_avg * (self.len_fft as f64) * (self.len_fft as f64));
						if best_match.test_statistic < best_test_stat_this_doppler {
							best_match.doppler_hz = base_freq + ((coarse_idx as f64)*self.fast_freq_inc);
							best_match.code_phase = best_idx_this_doppler;
							best_match.test_statistic = best_test_stat_this_doppler;
						}

					}

				}

				// Return the best match if it meets the threshold
				if best_match.test_statistic > self.test_statistic_threshold { Ok(Some(best_match)) }
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

