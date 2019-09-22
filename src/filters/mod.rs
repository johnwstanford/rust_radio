
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

pub struct SecondOrderFIR {
	pub b0: f64,
	pub b1: f64,
	pub x0: f64,
	pub x1: f64,
}

pub fn new_second_order_fir(b0: f64, b1: f64) -> SecondOrderFIR {
	SecondOrderFIR{ b0, b1, x0: 0.0, x1: 0.0}
}

impl SecondOrderFIR {

	pub fn apply(&mut self, x:f64) -> f64 {
		self.x1 = self.x0;
		self.x0 = x;
		self.b0*self.x0 + self.b1*self.x1
	}

}