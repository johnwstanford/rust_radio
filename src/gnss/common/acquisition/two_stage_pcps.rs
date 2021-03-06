
use num_complex::Complex;

use crate::{Sample, DigSigProcErr as DSPErr};
use crate::block::{BlockFunctionality, BlockResult};

use super::AcquisitionResult;

pub struct Acquisition {
	pub fs: f64,
	pub test_statistic_threshold: f64,
	pub stage_two_resolution_hz: f64,
	state:     State,
	stage_one: super::fast_pcps::Acquisition,
	stage_two: super::basic_pcps::Acquisition,
}

enum State {
	StageOne,
	StageTwo{ current_freq_hz:f64, current_step_hz:f64, last_code_phase:usize },
}

impl BlockFunctionality<(), (), Sample, AcquisitionResult> for Acquisition {

	fn control(&mut self, _:&()) -> Result<(), &'static str> {
		Ok(())
	}

	fn apply(&mut self, input:&Sample) -> BlockResult<AcquisitionResult> {
		self.provide_sample(input).unwrap();
		match self.block_for_result() {
			Ok(Some(result)) => BlockResult::Ready(result),
			Ok(None)         => BlockResult::NotReady,
			Err(e)           => BlockResult::Err(e)
		}
	}

}

impl Acquisition {
	pub fn new(symbol:Vec<Complex<f64>>, fs:f64, prn:usize, n_coarse:usize, n_fine:usize, stage_two_resolution_hz:f64, test_statistic_threshold:f64, n_skip:usize) -> Acquisition {
		let state = State::StageOne;
		let stage_one = super::make_acquisition(symbol.clone(), fs, prn, n_coarse, n_fine, test_statistic_threshold, n_skip);
		let stage_two = super::basic_pcps::Acquisition::new(symbol, fs, prn, 0.0, vec![]);
		Acquisition{ fs, test_statistic_threshold, stage_two_resolution_hz, state, stage_one, stage_two }
	}

	pub fn prn(&self) -> usize { self.stage_one.prn }

	pub fn provide_sample(&mut self, sample:&Sample) -> Result<(), DSPErr> { match self.state {
		State::StageOne => self.stage_one.provide_sample(sample),
		State::StageTwo{ current_freq_hz:_, current_step_hz:_, last_code_phase:_ } => self.stage_two.provide_sample(sample),
	}}

	pub fn block_for_result(&mut self) -> Result<Option<super::AcquisitionResult>, DSPErr> {
		let (next_state, ans) = match self.state {
			State::StageOne => {
				match self.stage_one.block_for_result() {
					Ok(Some(acq)) => { 
						let current_range_hz:f64 = 0.5 * acq.doppler_step_hz;
						
						self.stage_two.doppler_freqs.clear();
						self.stage_two.doppler_freqs.push(acq.doppler_hz - 0.5*current_range_hz);
						self.stage_two.doppler_freqs.push(acq.doppler_hz + 0.5*current_range_hz);
						/*eprintln!("PRN {}: Stage one acq at {:.1} +/- {:.1} [Hz] and {} [samples] and {:.6}, trying {:.1} and {:.1} [Hz]", self.stage_two.prn, 
							acq.doppler_hz, current_range_hz, acq.code_phase, acq.test_statistic(),
							acq.doppler_hz - 0.5*current_range_hz, 
							acq.doppler_hz + 0.5*current_range_hz);*/
						
						(State::StageTwo{ current_freq_hz: acq.doppler_hz, current_step_hz: current_range_hz, last_code_phase: acq.code_phase }, Ok(None))
					},
					_ => (State::StageOne, Ok(None))
				}
			},
			State::StageTwo{ current_freq_hz, current_step_hz, last_code_phase } => {
				match self.stage_two.block_for_result() {
					Ok(Some(acq)) => {
						// The test statistic threshold is set to zero for stage two, so we'll always get a result after the first complete
						// symbol and we'll compare it to the threshold here.
						if acq.test_statistic() > self.test_statistic_threshold { 
							// If acquisition failed here, determine whether or not another refinement is needed
							if acq.doppler_step_hz <= self.stage_two_resolution_hz {
								// If we've met the step threshold, then we're done
								(State::StageOne, Ok(Some(acq))) 
							} else {
								// Otherwise, make another refinement
								// TODO: consider comparing the code phase of this acquisition to the previous one to make sure it's within a few chips
								let current_step_hz:f64 = 0.5 * acq.doppler_step_hz;
								
								self.stage_two.doppler_freqs.clear();
								self.stage_two.doppler_freqs.push(acq.doppler_hz - 0.5*current_step_hz);
								self.stage_two.doppler_freqs.push(acq.doppler_hz + 0.5*current_step_hz);
								/*eprintln!("PRN {}: Stage two acq at {:.1} +/- {:.1} [Hz] and {} [samples] and {:.6}, trying {:.1} and {:.1} [Hz]", self.stage_two.prn, 
									acq.doppler_hz, current_step_hz, acq.code_phase, acq.test_statistic(),
									acq.doppler_hz - 0.5*current_step_hz, 
									acq.doppler_hz + 0.5*current_step_hz);*/

								(State::StageTwo{ current_freq_hz: acq.doppler_hz, current_step_hz, last_code_phase: acq.code_phase }, Ok(None))
							}
						}
						else { 
							// If acquisition failed here, go back to stage one and start over
							(State::StageOne, Ok(None))      
						}
					},
					_ => (State::StageTwo{ current_freq_hz, current_step_hz, last_code_phase }, Ok(None))
				}
			},
		};

		self.state = next_state;

		ans
	}
}

