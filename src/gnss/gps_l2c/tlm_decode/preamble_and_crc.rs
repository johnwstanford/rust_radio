
use super::error_detection;

#[derive(Debug)]
pub struct PreambleAndCrc {
	buffer: Vec<bool>,
	state: State
}

#[derive(Debug)]
enum State {
	Initial,
	Valid{ is_inverse:bool },
}

impl PreambleAndCrc {
	
	pub fn new() -> Self {
		Self{ buffer: vec![], state: State::Initial }
	}

	pub fn apply(&mut self, b:bool) -> Option<Vec<bool>> {

		let (opt_next_state, opt_ans) = match self.state {
			State::Initial => {
				self.buffer.push(b);

				// TODO: consider using a VecDeque for better performance
				while self.buffer.len() > 300 { self.buffer.remove(0); }

				if self.buffer.len() == 300 {
					// See if this is a valid preamble + CRC
					if self.buffer[0..8] == [true, false, false, false, true, false, true, true] && error_detection::is_subframe_crc_ok(&self.buffer) {
						let msg:Vec<bool> = self.buffer.drain(..).take(276).collect();
						(Some(State::Valid{ is_inverse: false }), Some(msg))
					}
					else if self.buffer[0..8] == [false, true, true, true, false, true, false, false] {
						let mut inverse_buffer:Vec<bool> = self.buffer.drain(..).map(|x| !x).collect();
						if error_detection::is_subframe_crc_ok(&inverse_buffer) {
							let msg:Vec<bool> = inverse_buffer.drain(..).take(276).collect();
							(Some(State::Valid{ is_inverse: true }), Some(msg))	
						}
						else { (None, None) }
					}
					else { (None, None) }
				} else { (None, None) }
			},
			State::Valid{ is_inverse } => {
				self.buffer.push(b ^ is_inverse);
				if self.buffer.len() == 300 {
					if error_detection::is_subframe_crc_ok(&self.buffer) {
						// We passed the CRC check, so no state transition is necessary; just return the current message without the CRC
						let msg:Vec<bool> = self.buffer.drain(..).take(276).collect();
						(None, Some(msg))
					} else {
						// We failed the CRC check, so go back to the initial state and start over
						(Some(State::Initial), None)
					}
				} else {
					(None, None)
				}
			}
		};

		// Perform state transition if necessary
		if let Some(next_state) = opt_next_state {
			self.state = next_state;
		}

		opt_ans
	}

}