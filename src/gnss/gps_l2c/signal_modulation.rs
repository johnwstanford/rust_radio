
const CM_INITIAL_STATE:[[bool; 27]; 2] = [
	[true, true, true, true, false, false, false, true, false, true,  false, false, false, false, true, true, true,  true,  true,  true,  false, true,  true, false, true, false, false],		// PRN 01
	[true, true, true, true, false, true,  true,  true, false, false, false, false, false, false, true, true, false, false, false, false, false, false, true, true,  true, false, true ]		// PRN 02
	];

pub struct ModularShiftRegister {
	pub state: [bool; 27],
}

impl ModularShiftRegister {
	
	pub fn shift(&mut self) -> bool {
		let current_output:bool = self.state[26];

		self.state[26] = self.state[25];
		self.state[25] = self.state[24];
		self.state[24] = self.state[23] ^ current_output;
		self.state[23] = self.state[22] ^ current_output;
		self.state[22] = self.state[21] ^ current_output;
		self.state[21] = self.state[20] ^ current_output;
		self.state[20] = self.state[19];
		self.state[19] = self.state[18];
		self.state[18] = self.state[17] ^ current_output;
		self.state[17] = self.state[16];
		self.state[16] = self.state[15] ^ current_output;
		self.state[15] = self.state[14];
		self.state[14] = self.state[13] ^ current_output;
		self.state[13] = self.state[12];
		self.state[12] = self.state[11];
		self.state[11] = self.state[10] ^ current_output;
		self.state[10] = self.state[ 9];
		self.state[ 9] = self.state[ 8];
		self.state[ 8] = self.state[ 7] ^ current_output;
		self.state[ 7] = self.state[ 6];
		self.state[ 6] = self.state[ 5] ^ current_output;
		self.state[ 5] = self.state[ 4];
		self.state[ 4] = self.state[ 3];
		self.state[ 3] = self.state[ 2] ^ current_output;
		self.state[ 2] = self.state[ 1];
		self.state[ 1] = self.state[ 0];
		self.state[ 0] = current_output;

		current_output
	}

}

pub fn cm_code(prn:usize) -> [bool; 10230] {
	if prn >= 1 && prn <= 2 {
		let mut ans:[bool; 10230] = [false; 10230];
		let mut shift_reg = ModularShiftRegister{ state: CM_INITIAL_STATE[prn-1] };
		for idx in 0..10230 { ans[idx] = shift_reg.shift(); }
		ans
	} else {
		panic!("Invalid PRN number for CM code generation");
	}
}