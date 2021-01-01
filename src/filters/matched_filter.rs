
use num_complex::Complex;
use num_traits::Zero;

use crate::fourier_analysis::{Direction, FFT};

pub struct MatchedFilter {
	n: usize,
	fwd:  FFT,
	inv: FFT,
	waveform_freq_domain_conj: Vec<Complex<f64>>,
	filter_power: f64,
}

impl MatchedFilter {

	pub fn new(waveform_time_domain:&[Complex<f64>]) -> Self {
		let n = waveform_time_domain.len();
		let mut fwd  = FFT::new(n, Direction::Forward);
		let inv = FFT::new(n, Direction::Inverse);

		let filter_power:f64 = waveform_time_domain.iter().map(|x| x.norm_sqr()).sum();
		let waveform_freq_domain:Vec<Complex<f64>> = fwd.execute(&waveform_time_domain);
		let waveform_freq_domain_conj = waveform_freq_domain.into_iter().map(|x| x.conj()).collect();

		MatchedFilter { n, fwd, inv, waveform_freq_domain_conj, filter_power }
	}

	pub fn simple_pulse(freq_hz:f64, n:usize, n_pad:usize, sample_rate_sps:f64) -> Self {
		let rad_per_sec = freq_hz * 2.0 * std::f64::consts::PI;
		let rad_per_samp = rad_per_sec / sample_rate_sps;
		let mut waveform:Vec<Complex<f64>> = (0..n).map(|i| {
			let phase:f64 = (i as f64) * rad_per_samp;
			Complex{ re: phase.cos(), im: phase.sin() }
		}).collect();

		while waveform.len() < n_pad {
			waveform.push(Complex::zero());
		}

		Self::new(&waveform)

	}

	pub fn len(&self) -> usize {
		self.n
	}

	pub fn apply(&mut self, signal_time_domain:&[Complex<f64>]) -> Result<MatchedFilterResponse, &'static str> {
		if signal_time_domain.len() != self.n {
			Err("Wrong-sized input for matched filter")
		} else {
			let signal_power:f64 = signal_time_domain.iter().map(|x| x.norm_sqr()).sum();

			let signal_freq_domain:Vec<Complex<f64>> = self.fwd.execute(signal_time_domain);

			let correlation_freq_domain:Vec<Complex<f64>> = signal_freq_domain.iter().zip(self.waveform_freq_domain_conj.iter()).map(|(a,b)| a*b).collect();
			let correlation_time_domain:Vec<Complex<f64>> = self.inv.execute(&correlation_freq_domain);

			Ok(MatchedFilterResponse{ correlation_time_domain:correlation_time_domain.clone(), signal_power, filter_power: self.filter_power })
		}
	}

}

pub struct MatchedFilterResponse {
	pub correlation_time_domain: Vec<Complex<f64>>,
	pub signal_power: f64,
	pub filter_power: f64,
}

impl MatchedFilterResponse {

	pub fn test_stat_at_idx(&self, k:usize) -> f64 {
		self.correlation_time_domain[k].norm_sqr() / (self.signal_power * self.filter_power)
	}

	// Useful if you want to use a different signal power for some reason.  For example, signal power might 
	// change significantly from one window to another and you want to filter it to provide better comparison
	// between windows
	pub fn test_stat_at_idx_w_power(&self, k:usize, signal_power:f64) -> f64 {
		self.correlation_time_domain[k].norm_sqr() / (signal_power * self.filter_power)
	}

	pub fn best_test_stat(&self) -> (f64, usize) {
		let mut best = (0.0, 0);

		for (idx, term) in self.correlation_time_domain.iter().enumerate() {
			let norm_sqr = term.norm_sqr();

			if best.0 < norm_sqr {
				best = (norm_sqr, idx);
			}
		}

		let (best_norm_sqr, best_idx) = best;
		(best_norm_sqr / (self.signal_power * self.filter_power), best_idx)
	}

}