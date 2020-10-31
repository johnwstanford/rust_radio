
use crate::block::{Block, BlockFunctionality, BlockResult};
use crate::block::acquire_and_track::TrackResult;

struct MockAcq {
	pub mod_val: u32
}

impl BlockFunctionality<(), (), u32, usize> for MockAcq {

	fn control(&mut self, control:()) -> Result<(), &'static str> {
		Ok(control)
	}

	fn apply(&mut self, input:u32) -> BlockResult<usize> {
		if input % self.mod_val == 0 {
			BlockResult::Ready(input as usize)
		} else {
			BlockResult::NotReady
		}
	}

}

#[derive(Debug)]
struct MockTrack {
	pub last_acq: usize
}

impl BlockFunctionality<usize, (), u32, TrackResult<f32>> for MockTrack {

	fn control(&mut self, control:usize) -> Result<(), &'static str> {
		self.last_acq = control;
		Ok(())
	}

	fn apply(&mut self, input:u32) -> BlockResult<TrackResult<f32>> {
		if input as usize > 2*self.last_acq {
			BlockResult::Ready(TrackResult::LossOfLock)
		} else {
			BlockResult::Ready(TrackResult::Ready(input as f32))
		}
	}

}

#[tokio::test(threaded_scheduler)]
async fn acquire_and_track() {

	let acq = Block::from(MockAcq{ mod_val: 7 });
	let trk = Block::from(MockTrack{ last_acq: 0});

	let mut aat = Block::acquire_and_track(acq, trk);
	let mut results:Vec<f32> = vec![];

	for sample in 0..55 {
		// Provide inputs
		aat.tx_input.send(sample).await.unwrap();

		// Receive all available outputs
		while let Ok(output) = aat.rx_output.try_recv() {
			results.push(output);
		}

		std::thread::sleep(std::time::Duration::from_millis(1));
	}

	assert_eq!(results, vec![8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 
		30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0, 37.0, 38.0, 39.0, 40.0, 41.0, 42.0, 50.0, 51.0, 52.0, 53.0]);

	aat.shutdown().await.unwrap();

}