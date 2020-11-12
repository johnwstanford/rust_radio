
pub mod matched_filter;

pub trait ScalarFilter {

	fn apply(&mut self, x:f64) -> f64;
	fn initialize(&mut self);
	fn scale_coeffs(&mut self, new_scale:f64);

}

pub struct FirstOrderFIR { pub b0: f64, pub b1: f64,
						   pub x0: f64, pub x1: f64,
						   pub scale: f64 }

impl FirstOrderFIR {

	pub fn new(b0: f64, b1: f64) -> Self { Self { b0, b1, x0: 0.0, x1: 0.0, scale: 1.0} }

}

impl ScalarFilter for FirstOrderFIR {

	fn apply(&mut self, x:f64) -> f64 {
		self.x0 = self.x1;
		self.x1 = x;
		(self.b0*self.x0 + self.b1*self.x1)*self.scale
	}

	fn initialize(&mut self) {
		self.x0 = 0.0;
		self.x1 = 0.0;
		self.scale = 1.0;
	}

	fn scale_coeffs(&mut self, new_scale:f64) {
		self.scale = new_scale;
	}

}

pub struct SecondOrderFIR { pub b0: f64, pub b1: f64, pub b2: f64,
						    pub x0: f64, pub x1: f64, pub x2: f64,
						    pub scale: f64  }

impl SecondOrderFIR {

	pub fn new(b0:f64, b1:f64, b2:f64) -> Self { Self{b0, b1, b2, x0: 0.0, x1: 0.0, x2: 0.0, scale:1.0} }

}

impl ScalarFilter for SecondOrderFIR {

	fn apply(&mut self, x:f64) -> f64 {
		self.x0 = self.x1;
		self.x1 = self.x2;
		self.x2 = x;
		(self.b0*self.x0 + self.b1*self.x1 + self.b2*self.x2)*self.scale
	}

	fn initialize(&mut self) {
		self.x0 = 0.0;
		self.x1 = 0.0;
		self.x2 = 0.0;
		self.scale = 1.0;
	}

	fn scale_coeffs(&mut self, new_scale:f64) {
		self.scale = new_scale;
	}
	
}

pub struct ThirdOrderFIR { pub b0: f64, pub b1: f64, pub b2: f64, pub b3: f64,
						   pub x0: f64, pub x1: f64, pub x2: f64, pub x3: f64,
						   pub scale: f64  }

impl ThirdOrderFIR {

	pub fn new(b0:f64, b1:f64, b2:f64, b3:f64) -> Self { 
		Self{b0, b1, b2, b3, x0: 0.0, x1: 0.0, x2: 0.0, x3: 0.0, scale: 1.0} 
	}

}

impl ScalarFilter for ThirdOrderFIR {

	fn apply(&mut self, x:f64) -> f64 {
		self.x0 = self.x1;
		self.x1 = self.x2;
		self.x2 = self.x3;
		self.x3 = x;
		(self.b0*self.x0 + self.b1*self.x1 + self.b2*self.x2 + self.b3*self.x3)*self.scale
	}

	fn initialize(&mut self) {
		self.x0 = 0.0;
		self.x1 = 0.0;
		self.x2 = 0.0;
		self.x3 = 0.0;
		self.scale = 1.0;
	}

	fn scale_coeffs(&mut self, new_scale:f64) {
		self.scale = new_scale;
	}
	
}