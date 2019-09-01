
use std::collections::VecDeque;

pub struct FIR {
	coeffs: Vec<f64>,
	buffer: VecDeque<f64>,
}

impl FIR {

	pub fn apply(&mut self, x:&f64) -> f64 {
		self.buffer.push_front(*x);
		while self.buffer.len() > self.coeffs.len() {
			self.buffer.pop_back();
		}
		(&self.buffer).into_iter().zip((&self.coeffs).into_iter()).map(|(a,b)| a*b).sum()
	}

}

pub fn new_fir(coeffs: Vec<f64>) -> FIR {

	let buffer:VecDeque<f64> = VecDeque::with_capacity(coeffs.len());

	FIR{ coeffs, buffer }

}