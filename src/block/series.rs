
/*use tokio::sync::mpsc;
use tokio::task::JoinHandle;

use super::Block;

impl<C: Send + Clone, T: Send, U: Send> Block<C, T, U> {

	// Both blocks need to take the same control type, so this will need to usually
	// be something pretty simple like () to indicate a reset command
	pub fn connect<V: Send>(self, other:Block<C, U, V>) -> Block<C, T, V> {

		let (   tx_control,  mut rx_control) = mpsc::channel::<C>(10);
		let (     tx_input,    mut rx_input) = mpsc::channel(10);
		let (mut tx_output,       rx_output) = mpsc::channel(10);

		let Block { tx_control: mut ctrl_0, tx_input:mut tx_0, rx_output:mut rx_0, handles: mut handles_0 } = self;
		let Block { tx_control: mut ctrl_1, tx_input:mut tx_1, rx_output:mut rx_1, handles: mut handles_1 } = other;

		// Create a new thread connecting rx_input to tx_0
		let handle_input:JoinHandle<Result<(), &'static str>> = tokio::spawn(async move {

		    while let Some(t) = rx_input.recv().await {
		    	// Interleaving control handling with input handling prevents us from having to
		    	// use a mutex to protect the state
		    	if let Ok(c) = rx_control.try_recv() {
		    		let c0 = c.clone();
		    		ctrl_0.send(c0).await.map_err(|_| "Unable to send control value")?;
		    		ctrl_1.send(c ).await.map_err(|_| "Unable to send control value")?;
		    	}

		    	tx_0.send(t).await.map_err(|_| "Unable to send output")?;
		    }			

		    Ok(())
        });

		// Create a new thread connecting rx_0 to tx_1
		let handle_01:JoinHandle<Result<(), &'static str>> = tokio::spawn(async move {

		    while let Some(t) = rx_0.recv().await {
		    	tx_1.send(t).await.map_err(|_| "Unable to send output")?;
		    }			

		    Ok(())
        });

		// Create a new thread connecting rx_1 to tx_output
		let handle_output:JoinHandle<Result<(), &'static str>> = tokio::spawn(async move {

		    while let Some(t) = rx_1.recv().await {
		    	tx_output.send(t).await.map_err(|_| "Unable to send output")?;
		    }			

		    Ok(())
        });

		// Combined the handles into a new vector of handles that the new block with be responsible for
        let mut handles = vec![handle_input, handle_01, handle_output];
        handles.append(&mut handles_0);
        handles.append(&mut handles_1);

		Block{ tx_control, tx_input, rx_output, handles }

	}

	
}*/