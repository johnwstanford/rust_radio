
use std::marker::PhantomData;

use crate::{DigSigProcErr as DSPErr};
use crate::block::{BlockFunctionality as BlkFunc, BlockResult};


pub struct AcquireAndTrack<T: Clone, U: Clone, V, A: BlkFunc<(), (), T, U>, B:BlkFunc<U, (), T, V>> {
	pub acq: A,
	pub trk: B,
	pub awaiting_acq: bool,
	pt: PhantomData<T>,
	pu: PhantomData<U>,
	pv: PhantomData<V>,
}

impl<T: Clone, U: Clone, V, A: BlkFunc<(), (), T, U>, B:BlkFunc<U, (), T, V>> AcquireAndTrack<T, U, V, A, B> {

	pub fn new(acq:A, trk:B) -> Self {
		Self { acq, trk, awaiting_acq: true, pt: PhantomData, pu: PhantomData, pv: PhantomData }
	}

}

impl<T: Clone, U: Clone, V, A: BlkFunc<(), (), T, U>, B:BlkFunc<U, (), T, V>> BlkFunc<(), bool, T, V> for AcquireAndTrack<T, U, V, A, B> {

	fn control(&mut self, _:&()) -> Result<bool, &'static str> {
		// Is this block actively tracking?
		Ok(!self.awaiting_acq)
	}

	fn apply(&mut self, input:&T) -> BlockResult<V> {
		if self.awaiting_acq {
			match self.acq.apply(input) {
				BlockResult::Ready(u) => {
					// Successful acquisition
					self.trk.control(&u).unwrap();
					self.awaiting_acq = false;
					BlockResult::NotReady
				},
				BlockResult::NotReady => BlockResult::NotReady,
				BlockResult::Err(e)   => BlockResult::Err(e)
			}
		} else {
			match self.trk.apply(input) {
				BlockResult::Ready(v) => BlockResult::Ready(v),
				BlockResult::Err(DSPErr::LossOfLock) => {
					self.awaiting_acq = true;
					BlockResult::NotReady
				},
				BlockResult::Err(e) => panic!("Error other than LossOfLock: {:?}", e),
				BlockResult::NotReady => BlockResult::NotReady,
			}
		}
	}

}

