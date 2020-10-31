
use std::collections::VecDeque;
use std::marker::PhantomData;

use crate::block::{BlockFunctionality as BlkFunc, BlockResult};


pub struct RotatingSplitAndMerge<T: Clone, U, A: BlkFunc<(), bool, T, U>> {
	rotation_interval:usize,
	rotation_count:usize,
	next_to_activate:usize,
	max_active:Option<usize>,
	blocks:Vec<(bool, A)>,
	output_buffer:VecDeque<U>,
	pt: PhantomData<T>,
}

impl<T: Clone, U, A: BlkFunc<(), bool, T, U>> RotatingSplitAndMerge<T, U, A> {

	pub fn from_iter<I: Iterator<Item=A>>(iter:I, rotation_interval:usize, max_active:Option<usize>) -> Self {
		let blocks = iter.map(|a| (false, a)).collect();
		Self { rotation_interval, rotation_count: 0, next_to_activate:0, max_active, blocks, 
			output_buffer: VecDeque::new(), pt: PhantomData }
	}

	// This implementation is based on the assumption that the component blocks with usually output NotReady and the
	// output buffer won't be filled faster than it's emptied.  This function can be used to check this assumption
	pub fn output_buffer_len(&self) -> usize { self.output_buffer.len() }

	fn activate_next(&mut self) {
		let n:usize = self.blocks.len();
		for idx in 0..n {
			if !self.blocks[(self.next_to_activate + idx)%n].0 {
				// eprintln!("Activating block at index {}", self.next_to_activate + idx);
				self.blocks[(self.next_to_activate + idx)%n].0 = true;
				self.next_to_activate = self.next_to_activate + idx + 1;
				self.next_to_activate %= n;
				return;
			}			
		}
	}

	fn rotate(&mut self) {

		// Deactivate blocks without an active track
		let mut total_locked = 0;
		for (is_active, blk) in self.blocks.iter_mut() {
			if blk.control(&()).unwrap() {
				total_locked += 1; 
			} else {
				*is_active = false; 				
			}
		}

		// As long as we're below our maximum, activate at least one
		// At some point, I might add configuration that allows you to activate more than one in this case
		if total_locked < self.max_active.unwrap_or(self.blocks.len()) {
			self.activate_next();
		}
	}

}

impl<T: Clone, U, A: BlkFunc<(), bool, T, U>> BlkFunc<(), (usize, usize), T, U> for RotatingSplitAndMerge<T, U, A> {

	// The control response is just the number of blocks with a lock and the number that are active
	fn control(&mut self, _:&()) -> Result<(usize, usize), &'static str> {
		let mut total_active = 0;
		let mut total_locked = 0;
		// We don't actually need to mutate the blocks, but the BlockFunctionality trait is essentially reserving
		// the right to mutate the block for us and that's why we need iter_mut()
		for (is_active, blk) in self.blocks.iter_mut() {
			if *is_active        { total_active += 1; }
			if blk.control(&())? { total_locked += 1; }
		}
		Ok((total_locked, total_active))
	}

	fn apply(&mut self, input:&T) -> BlockResult<U> {
		self.rotation_count += 1;
		if self.rotation_count >= self.rotation_interval {
			self.rotate();
			self.rotation_count = 0;
		}

		for (is_active, blk) in self.blocks.iter_mut() {
			// Only provide the input to blocks that are active
			if *is_active {
				match blk.apply(input) {
					BlockResult::NotReady => (),
					BlockResult::Ready(u) => self.output_buffer.push_back(u),
					BlockResult::Err(e)   => return BlockResult::Err(e)
				}
			}
		}

		match self.output_buffer.pop_front() {
			Some(u) => BlockResult::Ready(u),
			None    => BlockResult::NotReady,
		}
	}

}

impl<T: Clone, U, A: BlkFunc<(), bool, T, U>> BlkFunc<(), (usize, usize), T, Vec<U>> for RotatingSplitAndMerge<T, U, A> {

	// The control response is just the number of blocks with a lock and the number that are active
	fn control(&mut self, _:&()) -> Result<(usize, usize), &'static str> {
		let mut total_active = 0;
		let mut total_locked = 0;
		// We don't actually need to mutate the blocks, but the BlockFunctionality trait is essentially reserving
		// the right to mutate the block for us and that's why we need iter_mut()
		for (is_active, blk) in self.blocks.iter_mut() {
			if *is_active        { total_active += 1; }
			if blk.control(&())? { total_locked += 1; }
		}
		Ok((total_locked, total_active))
	}

	fn apply(&mut self, input:&T) -> BlockResult<Vec<U>> {
		self.rotation_count += 1;
		if self.rotation_count >= self.rotation_interval {
			self.rotate();
			self.rotation_count = 0;
		}

		for (is_active, blk) in self.blocks.iter_mut() {
			// Only provide the input to blocks that are active
			if *is_active {
				match blk.apply(input) {
					BlockResult::NotReady => (),
					BlockResult::Ready(u) => self.output_buffer.push_back(u),
					BlockResult::Err(e)   => return BlockResult::Err(e)
				}
			}
		}

		if self.output_buffer.len() == 0 {
			BlockResult::NotReady
		} else {
			BlockResult::Ready(self.output_buffer.drain(..).collect())
		}
	}

}

pub struct SplitAndMerge<C: Clone, D, T: Clone, U, A: BlkFunc<C, D, T, U>> {
	blocks:Vec<A>,
	output_buffer:VecDeque<U>,
	pc: PhantomData<C>,
	pd: PhantomData<D>,
	pt: PhantomData<T>,
}

impl<C: Clone, D, T: Clone, U, A: BlkFunc<C, D, T, U>> SplitAndMerge<C, D, T, U, A> {

	pub fn from_iter<I: Iterator<Item=A>>(iter:I) -> Self {
		let blocks = iter.collect();
		Self { blocks, output_buffer: VecDeque::new(), pc: PhantomData, pd: PhantomData, pt: PhantomData }
	}

	// This implementation is based on the assumption that the component blocks with usually output NotReady and the
	// output buffer won't be filled faster than it's emptied.  This function can be used to check this assumption
	pub fn output_buffer_len(&self) -> usize { self.output_buffer.len() }

}

impl<C: Clone, D, T: Clone, U, A: BlkFunc<C, D, T, U>> BlkFunc<C, Vec<D>, T, U> for SplitAndMerge<C, D, T, U, A> {

	// The control input is applied to all blocks and a vector of all responses is returned
	fn control(&mut self, c:&C) -> Result<Vec<D>, &'static str> {
		let mut ans = vec![];
		for blk in self.blocks.iter_mut() {
			ans.push(blk.control(c)?);
		}
		Ok(ans)
	}

	fn apply(&mut self, input:&T) -> BlockResult<U> {
		for blk in self.blocks.iter_mut() {
			match blk.apply(input) {
				BlockResult::NotReady => (),
				BlockResult::Ready(u) => self.output_buffer.push_back(u),
				BlockResult::Err(e)   => return BlockResult::Err(e)
			}
		}

		match self.output_buffer.pop_front() {
			Some(u) => BlockResult::Ready(u),
			None    => BlockResult::NotReady,
		}
	}

}