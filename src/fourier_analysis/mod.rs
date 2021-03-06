
use std::sync::Arc;

use num_complex::Complex;
use num_traits::Zero;

use rustfft::{FFT as FFTtrait};
use rustfft::FFTplanner;

use crate::types::even_odd_iter::EvenOddIter;

#[derive(Clone, Copy, PartialEq)]
pub enum Direction { Forward = -1, Inverse = 1 }

// Simpler interface around RustFFT
pub struct FFT {
	input: Vec<Complex<f64>>,
	output: Vec<Complex<f64>>,
	fft: Arc<dyn FFTtrait<f64>>,
}

impl FFT {

	pub fn new(n:usize, direction:Direction) -> Self {
		let input: Vec<Complex<f64>>  = vec![Complex::zero(); n];
		let output: Vec<Complex<f64>> = vec![Complex::zero(); n];
		let mut planner = FFTplanner::new(direction == Direction::Inverse);
		let fft = planner.plan_fft(n);
		Self{ input, output, fft }

	}

	pub fn execute(&mut self, data:&[Complex<f64>]) -> Vec<Complex<f64>> {
		self.input.clone_from_slice(data);
		self.fft.process(&mut self.input, &mut self.output);
		self.output.clone()
	}

}

pub fn fft(x: &[Complex<f64>])  -> Vec<Complex<f64>> { fft_k(EvenOddIter::from(x), -1.0) }
pub fn ifft(x: &[Complex<f64>]) -> Vec<Complex<f64>> { fft_k(EvenOddIter::from(x), 1.0).iter().map(|c| c / (x.len() as f64)).collect() }

fn fft_k(mut x:EvenOddIter<Complex<f64>>, sign:f64) -> Vec<Complex<f64>> {
    let n = x.len();
    if n%2 == 0 {
	    let ek = fft_k(x.even(), sign);
	    let ok = fft_k(x.odd(),  sign);
	    let pairs:Vec<(&Complex<f64>, Complex<f64>)> = (0..(n / 2)).map(|k| {
		    let ex = Complex{re: 0.0, im: (sign * 2.0 * std::f64::consts::PI * (k as f64)) / (n as f64)}.exp();
		    (&ek[k], ex * ok[k])
	    }).collect();

	    let first_half  = pairs.iter().map(|(ek, eok)| *ek + eok);
	    let second_half = pairs.iter().map(|(ek, eok)| *ek - eok);
	    first_half.chain(second_half).collect()
    }
    else if n == 1 { vec![x.next().unwrap().clone()]              }
	else           { panic!("Can only run FFT/IFFT with powers of 2"); }    
}

#[test]
fn test_fft_and_ifft() {
	let x_time_usize:Vec<usize> = (0..8).collect();
	let x_time:Vec<Complex<f64>> = x_time_usize.iter().map(|x| Complex{re: *x as f64, im: 0.0}).collect();
	let x_freq:Vec<Complex<f64>> = fft(&x_time);

	let x_time_usize_p:Vec<usize> = ifft(&x_freq).iter().map(|c| c.re.round() as usize ).collect();

	for (a,b) in x_time_usize.iter().zip(x_time_usize_p.iter()) {
		assert_eq!(a, b);
	}
}
