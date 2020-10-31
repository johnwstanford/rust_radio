
/*use std::sync::{Arc, Mutex};

use tokio::sync::mpsc;
use tokio::task::JoinHandle;

use super::{Block, TrackResult};

#[cfg(test)]
mod tests;

/* The case where one block acquires a signal, then the input switches to another block, which
tracks the signal is a common enough use case that it's worth special handling.  The entire block
consumes T, which can go to either the acquisition or tracking block.  The tracking block produces
type U, which goes to the outside.  The acquisition block internally produces V, which initializes
the tracking block. */
impl<C: Send, T: Send, U: Send> Block<C, T, U> {

	pub fn acquire_and_track<V: Send>(acq:Self, trk:Block<U, T, TrackResult<V>>) -> Block<C, T, V> {

		let (   tx_control,  mut rx_control) = mpsc::channel::<C>(10);
		let (     tx_input,    mut rx_input) = mpsc::channel::<T>(10);
		let (mut tx_output,       rx_output) = mpsc::channel::<V>(10);

		let Block { tx_control: mut ctrl_acq, tx_input:mut tx_acq, rx_output:mut rx_acq, handles: mut handles_acq } = acq;
		let Block { tx_control: mut ctrl_trk, tx_input:mut tx_trk, rx_output:mut rx_trk, handles: mut handles_trk } = trk;

		let awaiting_acq_mutex = Arc::new(Mutex::new(true));
    	let awaiting_acq_0 = awaiting_acq_mutex.clone();
    	let awaiting_acq_1 = awaiting_acq_mutex.clone();
    	let awaiting_acq_2 = awaiting_acq_mutex.clone();

		let handle_input:JoinHandle<Result<(), &'static str>> = tokio::spawn(async move {

		    while let Some(t) = rx_input.recv().await {
		    	// For each input of type T, which is the input of both the acquisition and tracking blocks (e.g. Sample)

		    	// First, see if there's a control message for the whole AAT block, which is of type C, which is the control
		    	// type for the acquisition block, relay it to the acquisition block
		    	if let Ok(c) = rx_control.try_recv() { ctrl_acq.send(c).await.map_err(|_| "AAT failure to relay control message")?; }

		    	let awaiting_acq:bool = *(awaiting_acq_0.lock().unwrap());

		    	if awaiting_acq {
		    		tx_acq.send(t).await.map_err(|_| "AAT failure to relay input to ACQ")?;
		    	} else {
		    		tx_trk.send(t).await.map_err(|_| "AAT failure to relay input to TRK")?;
		    	}

		    }

		    Ok(())
        });

		let handle_acq:JoinHandle<Result<(), &'static str>> = tokio::spawn(async move {

		    while let Some(u) = rx_acq.recv().await {
		    	// For each acquisition output of type U, which is the control type of the tracking block

		    	ctrl_trk.send(u).await.map_err(|_| "AAT failure to send ACQ message to TRK control")?;

		    	*(awaiting_acq_1.lock().unwrap()) = false;

		    }

		    Ok(())
        });

		let handle_trk:JoinHandle<Result<(), &'static str>> = tokio::spawn(async move {

		    'trk_loop: while let Some(opt_v) = rx_trk.recv().await {
		    	match opt_v {
		    		TrackResult::NotReady   => (),
		    		TrackResult::Ready(v)   => tx_output.send(v).await.map_err(|_| "AAT failure to send TRK message to output")?,
		    		TrackResult::LossOfLock => *(awaiting_acq_2.lock().unwrap()) = true,
		    		TrackResult::Err(_)     => break 'trk_loop,
		    	}
		    }

		    Ok(())
        });

		// Combined the handles into a new vector of handles that the new block with be responsible for
        let mut handles = vec![handle_input, handle_acq, handle_trk];
        handles.append(&mut handles_acq);
        handles.append(&mut handles_trk);

		Block{ tx_control, tx_input, rx_output, handles }

	}

}*/