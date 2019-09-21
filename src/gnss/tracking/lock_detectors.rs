
extern crate rustfft;

use std::collections::VecDeque;

use self::rustfft::num_complex::Complex;

pub fn cn0_svn_estimator(prompt_buffer:&VecDeque<(Complex<f64>, usize)>, coh_integration_time_s:f64) -> f64 {
	let n:f64 = prompt_buffer.len() as f64;
	let p_sig:f64 = {
		let sum:f64 = prompt_buffer.into_iter().map(|(c, _)| c.re.abs() ).sum();
		(sum / n).powi(2)
	};
	let p_tot:f64 = {
		let sum:f64 = prompt_buffer.into_iter().map(|(c, _)| c.re*c.re + c.im*c.im).sum();
		sum / n
	};
	let snr = p_sig / (p_tot - p_sig);
	10.0 * snr.log10() - 10.0 * coh_integration_time_s.log10()
}

pub fn carrier_lock_detector(prompt_buffer:&VecDeque<(Complex<f64>, usize)>) -> f64 {
    let tmp_sum_i:f64 = prompt_buffer.into_iter().map(|(c, _)| c.re).sum();
    let tmp_sum_q:f64 = prompt_buffer.into_iter().map(|(c, _)| c.im).sum();
    let nbp:f64 = tmp_sum_i * tmp_sum_i + tmp_sum_q * tmp_sum_q;
    let nbd:f64 = tmp_sum_i * tmp_sum_i - tmp_sum_q * tmp_sum_q;
    nbd / nbp
}
