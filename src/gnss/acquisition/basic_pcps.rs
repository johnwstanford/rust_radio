
extern crate rustfft;

use std::f64::consts;
use std::sync::Arc;

use self::rustfft::FFT;
use self::rustfft::num_complex::Complex;

const N_SKIP:usize = 9;

pub struct Acquisition {
	pub fs:f64,
	pub test_statistic_threshold:f64,
	pub doppler_freqs:Vec<i16>,
	pub buffer:Vec<Complex<f64>>,
	pub len_fft:usize,
	pub fft:Arc<dyn FFT<f64>>,
	pub local_code_freq_domain:Vec<Complex<f64>>,
	pub fft_out:  Vec<Complex<f64>>,
	pub ifft:Arc<dyn FFT<f64>>,
	pub ifft_out: Vec<Complex<f64>>,
	pub skip_count: usize,
}

impl super::Acquisition for Acquisition {

	fn provide_sample(&mut self, sample:Complex<f64>) -> Result<(), &str> {
		self.buffer.push(sample);
		Ok(())
	}

	fn block_for_result(&mut self, prn:usize) -> Result<Option<super::AcquisitionResult>, &str> {
		if self.buffer.len() >= self.len_fft {
			self.skip_count += 1;
			if self.skip_count >= N_SKIP {
				self.skip_count = 0;

				// Try acquiring an SV
				let input_power_total:f64 = self.buffer.iter().map(|c| c.re*c.re + c.im*c.im).sum();
				let input_power_avg:f64 = input_power_total / (self.len_fft as f64);

				let mut best_match = super::AcquisitionResult{ doppler_hz: 0, code_phase: 0, test_statistic: 0.0 };

				// Try every frequency and update best_match every time we find a new best
				for freq in self.doppler_freqs.iter() {
					// Wipe the carrier off the input signal
					let phase_step_rad:f64 = (-2.0 * consts::PI * (*freq as f64)) / self.fs;			
					let mut doppler_wiped_time_domain:Vec<Complex<f64>> = (0..(self.buffer.len()))
						.map(|idx| {
							let phase = phase_step_rad * (idx as f64);
							self.buffer[idx] * Complex{ re: phase.cos(), im: phase.sin() }
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

					// Normalize, enumerate, and sort the magnitudes
					let mut magnitudes:Vec<(usize, f64)> = (&self.ifft_out).into_iter()
						.map(|c| (c.re*c.re + c.im*c.im))
						.enumerate().collect();
					magnitudes.sort_by(|a,b| b.1.partial_cmp(&a.1).unwrap() );

					// Compare the best result from this iteration to the overall best result
					let (best_idx_this_doppler, best_test_stat_raw_this_doppler) = magnitudes[0];
					let best_test_stat_this_doppler:f64 = best_test_stat_raw_this_doppler / (input_power_avg * (self.len_fft as f64) * (self.len_fft as f64));
					if best_match.test_statistic < best_test_stat_this_doppler {
						best_match.doppler_hz = *freq;
						best_match.code_phase = best_idx_this_doppler;
						best_match.test_statistic = best_test_stat_this_doppler;
					}
				}

				// Clear the buffer for next time
				self.buffer.clear();

				// Return the best match if it meets the threshold
				if best_match.test_statistic > self.test_statistic_threshold { Ok(Some(best_match)) }
				else { Ok(None) }

			} else {
				// Clear the buffer for next time
				self.buffer.clear();

				Ok(None)
			}

		} else {
			// Buffer isn't full yet, so there's no result to return
			Ok(None)
		}

	}

}

