
use std::collections::VecDeque;
use ::DigSigProcErr;

const POS_PATTERN:[bool; 8] = [true,  false, false, false, true,  false, true,  true ];
const NEG_PATTERN:[bool; 8] = [false, true,  true,  true,  false, true,  false, false];

pub struct PreambleDetector {
	buffer:VecDeque<bool>,
	current_bit:usize,
	inverse_sense:Option<bool>,
	preamble_location:Option<usize>,
}

pub fn new_preamble_detector() -> PreambleDetector {
	PreambleDetector{ buffer: VecDeque::new(), current_bit: 0, inverse_sense: None, preamble_location: None }
}

impl PreambleDetector {

	pub fn initialize(&mut self) {
		self.buffer.clear();
		self.current_bit = 0;
		self.inverse_sense = None;
		self.preamble_location = None;
	}

	pub fn apply(&mut self, b:bool) {
		self.buffer.push_back(b);
		self.current_bit += 1;

		// Limit the buffer size to 30
		while self.buffer.len() > 30 { self.buffer.pop_front().unwrap(); }

		// TODO: consider adding a flag to indicate whether the preamble has been lost after being found the first time
		if self.buffer.len() == 30 {
			let first_eight:Vec<bool> = self.buffer.iter().map(|b| *b).take(8).collect();
			if first_eight == POS_PATTERN {
				let whole_word:Vec<bool> = self.buffer.iter().map(|b| *b).collect();
				if super::parity_check(&whole_word, false, false) {
					self.inverse_sense = Some(false);
					self.preamble_location = Some((self.current_bit - 30)%300);
				}
			} 
			else if first_eight == NEG_PATTERN {
				let whole_word:Vec<bool> = self.buffer.iter().map(|b| !b).collect();
				if super::parity_check(&whole_word, false, false) {
					self.inverse_sense = Some(true);
					self.preamble_location = Some((self.current_bit - 30)%300);
				}
			}
		}
	}

	pub fn get_result(&self) -> Result<usize, DigSigProcErr> {
		match self.preamble_location {
			Some(x) => Ok(x),
			None    => Err(DigSigProcErr::InvalidTelemetryData),
		}
	}

	pub fn is_inverse_sense(&self) -> Result<bool, DigSigProcErr> {
		match self.inverse_sense {
			Some(b) => Ok(b),
			None    => Err(DigSigProcErr::InvalidTelemetryData),
		}
	}
}

