
use std::marker::PhantomData;

use crate::block::{BlockFunctionality as BlkFunc, BlockResult};

pub mod acquire_and_track;
pub mod split_and_merge;

pub struct Series<C: Clone, D, T: Clone, U: Clone, V, A: BlkFunc<C, D, T, U>, B:BlkFunc<C, D, U, V>> {
	left: A,
	right: B,
	pub left_control: bool,
	pc: PhantomData<C>,
	pd: PhantomData<D>,
	pt: PhantomData<T>,
	pu: PhantomData<U>,
	pv: PhantomData<V>,
}

impl<C: Clone, D, T: Clone, U: Clone, V, A: BlkFunc<C, D, T, U>, B:BlkFunc<C, D, U, V>> Series<C, D, T, U, V, A, B> {

	pub fn new(left:A, right:B, left_control:bool) -> Self {
		Self { left, right, left_control, pc:PhantomData, pd:PhantomData, pt:PhantomData, pu:PhantomData, pv:PhantomData }
	}

}

impl<C: Clone, D, T: Clone, U: Clone, V, A: BlkFunc<C, D, T, U>, B:BlkFunc<C, D, U, V>> BlkFunc<C, D, T, V> for Series<C, D, T, U, V, A, B> {

	fn control(&mut self, control:&C) -> Result<D, &'static str> {
		if self.left_control { self.left.control(control)  }
		else                 { self.right.control(control) }
	}

	fn apply(&mut self, input:&T) -> BlockResult<V> {
		match self.left.apply(input) {
			BlockResult::Ready(u) => self.right.apply(&u),
			BlockResult::NotReady => BlockResult::NotReady,
			BlockResult::Err(e)   => BlockResult::Err(e)
		}
	}

}

pub struct SeriesLeftControl<C: Clone, D, T: Clone, U: Clone, V, A: BlkFunc<C, D, T, U>, B:BlkFunc<C, D, U, V>> {
	left: A,
	right: B,
	pc: PhantomData<C>,
	pd: PhantomData<D>,
	pt: PhantomData<T>,
	pu: PhantomData<U>,
	pv: PhantomData<V>,
}

impl<C: Clone, D, T: Clone, U: Clone, V, A: BlkFunc<C, D, T, U>, B:BlkFunc<C, D, U, V>> SeriesLeftControl<C, D, T, U, V, A, B> {

	pub fn new(left:A, right:B) -> Self {
		Self { left, right, pc:PhantomData, pd:PhantomData, pt:PhantomData, pu:PhantomData, pv:PhantomData }
	}

}

impl<C: Clone, D, T: Clone, U: Clone, V, A: BlkFunc<C, D, T, U>, B:BlkFunc<C, D, U, V>> BlkFunc<C, D, T, V> for SeriesLeftControl<C, D, T, U, V, A, B> {

	fn control(&mut self, control:&C) -> Result<D, &'static str> {
		self.left.control(control)
	}

	fn apply(&mut self, input:&T) -> BlockResult<V> {
		match self.left.apply(input) {
			BlockResult::Ready(u) => self.right.apply(&u),
			BlockResult::NotReady => BlockResult::NotReady,
			BlockResult::Err(e)   => BlockResult::Err(e)
		}
	}

}

