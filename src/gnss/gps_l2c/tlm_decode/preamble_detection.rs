
use crate::DigSigProcErr;

#[derive(PartialEq, Clone, Copy)]
enum BitLocation {
	Uninitialized,
	AlwaysTrue,
	AlwaysFalse,
	Variable
}

impl BitLocation {
	
	fn apply(&mut self, b:bool) {
		// If this location has already changed more than once, then there's nothing to update
		if *self == BitLocation::Variable { return; }

		// If this location is uninitialize, then initialize it
		if *self == BitLocation::Uninitialized {
			*self = if b { BitLocation::AlwaysTrue } else { BitLocation::AlwaysFalse };
		} else {
			if *self == BitLocation::AlwaysTrue  &&  b { return; }
			if *self == BitLocation::AlwaysFalse && !b { return; }

			*self = BitLocation::Variable;
		}
	}

}

pub struct PreambleDetector {
	bit_locations: [BitLocation; 300],
	current_idx: usize,
}

impl PreambleDetector {
	
	pub fn new() -> Self {

		Self { bit_locations:[BitLocation::Uninitialized; 300], current_idx: 0 }
	}

	pub fn apply(&mut self, data:&[bool]) -> Result<Option<(usize, i8)>, DigSigProcErr> {
		for b in data {
			self.bit_locations[self.current_idx % 300].apply(*b);
			self.current_idx += 1;
		}

		if self.current_idx > 300 {
			self.current_idx -= 300;

			// After each time we wrap back around, try to detect the preamble
			let mut possible_pos_locs:Vec<usize> = vec![];
			let mut possible_neg_locs:Vec<usize> = vec![];

			for i in 0..300 {
				if self.bit_locations[(i+0)%300] == BitLocation::AlwaysTrue  && 
				   self.bit_locations[(i+1)%300] == BitLocation::AlwaysFalse && 
				   self.bit_locations[(i+2)%300] == BitLocation::AlwaysFalse && 
				   self.bit_locations[(i+3)%300] == BitLocation::AlwaysFalse && 
				   self.bit_locations[(i+4)%300] == BitLocation::AlwaysTrue  && 
				   self.bit_locations[(i+5)%300] == BitLocation::AlwaysFalse && 
				   self.bit_locations[(i+6)%300] == BitLocation::AlwaysTrue  && 
				   self.bit_locations[(i+7)%300] == BitLocation::AlwaysTrue { possible_pos_locs.push(i); }

				if self.bit_locations[(i+0)%300] == BitLocation::AlwaysFalse && 
				   self.bit_locations[(i+1)%300] == BitLocation::AlwaysTrue  && 
				   self.bit_locations[(i+2)%300] == BitLocation::AlwaysTrue  && 
				   self.bit_locations[(i+3)%300] == BitLocation::AlwaysTrue  && 
				   self.bit_locations[(i+4)%300] == BitLocation::AlwaysFalse && 
				   self.bit_locations[(i+5)%300] == BitLocation::AlwaysTrue  && 
				   self.bit_locations[(i+6)%300] == BitLocation::AlwaysFalse && 
				   self.bit_locations[(i+7)%300] == BitLocation::AlwaysFalse { possible_neg_locs.push(i); }
			}

			if possible_pos_locs.len() == 1 && possible_neg_locs.len() == 0 {
				Ok(Some((possible_pos_locs[0],  1)))
			} else if possible_pos_locs.len() == 0 && possible_neg_locs.len() == 1 {
				Ok(Some((possible_neg_locs[0], -1)))
			} else if possible_pos_locs.len() == 0 && possible_neg_locs.len() == 0 {
				Err(DigSigProcErr::InvalidTelemetryData("No possible preamble locations found"))
			} else {
				Ok(None)
			}

		} else {
			Ok(None)
		}
	}

}