
pub trait ScalarFilter {

	fn apply(&mut self, x:f64) -> f64;
	fn initialize(&mut self);

}

pub struct FirstOrderFIR { pub b0: f64, pub b1: f64,
							pub x0: f64, pub x1: f64 }

impl FirstOrderFIR {

	pub fn new(b0: f64, b1: f64) -> Self { Self { b0, b1, x0: 0.0, x1: 0.0} }

}

impl ScalarFilter for FirstOrderFIR {

	fn apply(&mut self, x:f64) -> f64 {
		self.x0 = self.x1;
		self.x1 = x;
		self.b0*self.x0 + self.b1*self.x1
	}

	fn initialize(&mut self) {
		self.x0 = 0.0;
		self.x1 = 0.0;
	}

}

pub struct SecondOrderFIR { pub b0: f64, pub b1: f64, pub b2: f64,
						   pub x0: f64, pub x1: f64, pub x2: f64 }

impl SecondOrderFIR {

	pub fn new(b0:f64, b1:f64, b2:f64) -> Self { Self{b0, b1, b2, x0: 0.0, x1: 0.0, x2: 0.0} }

}

impl ScalarFilter for SecondOrderFIR {

	fn apply(&mut self, x:f64) -> f64 {
		self.x0 = self.x1;
		self.x1 = self.x2;
		self.x2 = x;
		self.b0*self.x0 + self.b1*self.x1 + self.b2*self.x2
	}

	fn initialize(&mut self) {
		self.x0 = 0.0;
		self.x1 = 0.0;
		self.x2 = 0.0;
	}

}