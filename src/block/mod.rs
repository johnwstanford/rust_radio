
use tokio::sync::mpsc;
use tokio::task::JoinHandle;

use crate::{DigSigProcErr as DSPErr};

pub mod block_tree_sync_static;

/* TODO: Resurrect these async blocks at some point; The intent is to have hierarchies of blocks that fall into
several categories: synchronous with static typing, synchronous with dynamic typing, and asynchronous.  I might
divide the async ones down futher when I get there.  I wanted to do asynchronous first, but then I realized that
there's a lot of work to do creating synchronous structures first.  The different types could end up complementing
each other well.  Synchronous blocks of both types could be contained in async blocks. */
// pub mod acquire_and_track;
// pub mod series;

pub enum BlockResult<U> {
	NotReady,
	Ready(U),
	Err(DSPErr)
}

impl<U> BlockResult<U> {

	pub fn unwrap(self) -> U { 
		match self {
			Self::Ready(u) => u,
			_ => panic!("Called unwrap on something other than BlockResult::Ready")
		}
	}

}

// A type that implements BlockFunctionality consumes instances of T and 
// produces either Ok(Some(U)) if an output is ready, Ok(None) if an output
// is not ready, or an Err(_) if the operation fails
pub trait BlockFunctionality<C: Clone, D, T: Clone, U> {

	// Requiring immutable references to the input and control values and requiring them
	// to implement Clone is the least restrictive thing because if the block needs an owned
	// value, it can clone it (if sent to another thread, for example)
	fn control(&mut self, control:&C) -> Result<D, &'static str>;
	fn apply(&mut self, input:&T) -> BlockResult<U>;

}

pub struct Block<C: 'static + Send, D: 'static + Send, T: 'static + Send, U: 'static + Send> {
	pub tx_control:  mpsc::Sender<C>,
	pub rx_response: mpsc::Receiver<D>,
	pub tx_input:    mpsc::Sender<T>,
	pub rx_output:   mpsc::Receiver<U>,
	pub handles:     Vec<JoinHandle<Result<(), &'static str>>>,
}


impl<C: Send + Sync + Clone, D: Send + Sync, T: Send + Sync + Clone, U: Send + Sync> Block<C, D, T, U> {

	pub async fn apply(&mut self, input:T) -> Result<(), &'static str> {
		self.tx_input.send(input).await.map_err(|_| "Unable to send block input")
	}

	pub fn try_recv(&mut self) -> Result<U, &'static str> {
		self.rx_output.try_recv().map_err(|_| "Unable to receive block output")
	}

	pub fn from<B: 'static + BlockFunctionality<C, D, T, U> + Send + Sync>(b:B) -> Self {
		
		// TODO: make these channel capacities configurable
		let (     tx_control, mut rx_control) = mpsc::channel::<C>(10);
		let (mut tx_response,    rx_response) = mpsc::channel::<D>(10);
		let (       tx_input,   mut rx_input) = mpsc::channel::<T>(4000);
		let (  mut tx_output,      rx_output) = mpsc::channel::<U>(100);

		let handle:JoinHandle<Result<(), &'static str>> = tokio::spawn(async move {

			let mut owned_b = b;

		    'rx: while let Some(t) = rx_input.recv().await {

		    	// Interleaving control handling with input handling prevents us from having to
		    	// use a mutex to protect the state
		    	if let Ok(c) = rx_control.try_recv() {
		    		let d:D = owned_b.control(&c)?;
		    		tx_response.send(d).await.map_err(|_| "Unable to send control response")?;
		    	}

				match owned_b.apply(&t) {
					BlockResult::Ready(u) => tx_output.send(u).await.map_err(|_| "Unable to send output")?,
					BlockResult::NotReady => (),
					BlockResult::Err(e)   => {
						eprintln!("Error in block: {:?}", e);
						break 'rx;
					}
				}

		    }			

		    Ok(())
        });

        let handles = vec![handle];

		Block{ tx_control, rx_response, tx_input, rx_output, handles }
	}

	pub async fn shutdown(self) -> Result<(), &'static str> {
		
		let Block{ tx_control, rx_response:_, tx_input, rx_output:_, handles } = self;
		
		drop(tx_control);
		drop(tx_input);
		
		for handle in handles {
			handle.await.unwrap()?;
		}

		Ok(())
	}

}

