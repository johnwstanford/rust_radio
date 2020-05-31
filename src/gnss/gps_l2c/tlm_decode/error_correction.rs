
const ERROR_THRES:usize = 5;

struct History {
	current_state: u8,
	prev_states: Vec<u8>,
	cumulative_error: usize,
}

impl History {
	
	fn new(s0:u8) -> Self { Self {current_state: s0, prev_states: vec![], cumulative_error: 0} }

	fn transition(&self, next_bit:bool, expected_values:(bool, bool)) -> Self {

		// Calculate error
		let mut cumulative_error = self.cumulative_error;
		let (g1, g2) = cnav_fec(self.current_state);
		let (g1_exp, g2_exp) = expected_values;
		if g1 != g1_exp { cumulative_error += 1; }
		if g2 != g2_exp { cumulative_error += 1; }

		// Update state
		let current_state:u8 = (self.current_state / 2) + if next_bit { 64 } else { 0 };

		// Update history
		let mut prev_states = self.prev_states.clone();
		prev_states.push(self.current_state);

		Self{ current_state, prev_states, cumulative_error }
	}

}

fn cnav_fec(x:u8) -> (bool, bool) {

	let bit6:bool = (x & 0x40) != 0;
	let bit5:bool = (x & 0x20) != 0;
	let bit4:bool = (x & 0x10) != 0;
	let bit3:bool = (x & 0x08) != 0;

	// Both G1 and G2 skip bit2

	let bit1:bool = (x & 0x02) != 0;
	let bit0:bool = (x & 0x01) != 0;

	let g1:bool = bit6 ^ bit5 ^ bit4 ^ bit3 ^        bit0;
	let g2:bool = bit6 ^        bit4 ^ bit3 ^ bit1 ^ bit0;

	(g1, g2)

}

pub fn decode(symbols:Vec<bool>) -> Option<Vec<bool>> {
	let mut best_solutions:Vec<History> = (0..128).map(|i| History::new(i)).collect();

	for chunk in symbols.chunks_exact(2) {
		let mut new_solutions:Vec<History> = vec![];
		for h in &best_solutions { 
			let if_true  = h.transition(true,  (chunk[0], chunk[1]));
			let if_false = h.transition(false, (chunk[0], chunk[1]));

			if if_true.cumulative_error  < ERROR_THRES { new_solutions.push(if_true);  }
			if if_false.cumulative_error < ERROR_THRES { new_solutions.push(if_false); }
		}

		if new_solutions.len() == 0 { return None; }

		best_solutions = new_solutions;

	}

	best_solutions.iter().min_by_key(|h| h.cumulative_error).map(|h| h.prev_states.iter().map(|s| s%2 == 1).collect())

}