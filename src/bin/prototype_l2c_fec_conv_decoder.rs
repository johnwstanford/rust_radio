
#[derive(Debug, Clone)]
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

const ERROR_THRES:usize = 5;

fn main() {
	
	let symbols:Vec<bool> = vec![true, true, false, true, true, true, false, false, true, true, false, true, false, true, true, true, true, false, true, true, false, false, false, false, false, false, false, true, false, true, true, true, false, true, false, false, false, true, true, false, true, false, false, false, true, false, true, false, false, true, false, true, true, true, true, true, false, false, true, false, false, true, true, true, false, false, true, false, true, false, false, false, false, false, false, true, true, true, true, true, false, true, true, false, false, true, false, true, false, false, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, false, false, true, false, false, true, false, true, false, false, true, true, false, true, false, false, false, false, false, false, true, false, true, true, false, true, true, true, true, true, true, false, true, true, false, false, false, true, true, true, true, false, false, true, false, false, true, true, false, false, false, true, false, false, false, false, true, true, true, false, true, true, true, false, true, false, false, false, false, true, true, true, false, true, false, false, false, false, true, false, true, true, false, false, false, false, true, true];

	let mut best_solutions:Vec<History> = (0..128).map(|i| History::new(i)).collect();

	for chunk in symbols.chunks_exact(2) {
		let mut new_solutions:Vec<History> = vec![];
		for h in &best_solutions { 
			let if_true  = h.transition(true,  (chunk[0], chunk[1]));
			let if_false = h.transition(false, (chunk[0], chunk[1]));

			if if_true.cumulative_error  < ERROR_THRES { new_solutions.push(if_true);  }
			if if_false.cumulative_error < ERROR_THRES { new_solutions.push(if_false); }
		}

		best_solutions = new_solutions;

	}

	let best_solution = best_solutions.iter().min_by_key(|h| h.cumulative_error).unwrap().clone();

	// Expect 1010000000110111010000011100001000010010000001111111111111111100010101000000100111111101111100100110
	for s in best_solution.prev_states.clone() {
		print!("{}", s%2);
	}
	print!("\n");



}