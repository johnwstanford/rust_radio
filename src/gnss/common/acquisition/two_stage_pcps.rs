
extern crate rustfft;

use self::rustfft::num_complex::Complex;

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
	StageTwo,
}

impl Acquisition {
	pub fn new(symbol:Vec<i8>, fs:f64, prn:usize, n_coarse:usize, n_fine:usize, stage_two_resolution_hz:f64, test_statistic_threshold:f64, n_skip:usize) -> Acquisition {
		let state = State::StageOne;
		let stage_one = super::make_acquisition(symbol.clone(), fs, prn, n_coarse, n_fine, test_statistic_threshold, n_skip);
		let stage_two = super::make_basic_acquisition(symbol, fs, prn, 0.0, vec![]);
		Acquisition{ fs, test_statistic_threshold, stage_two_resolution_hz, state, stage_one, stage_two }
	}
}

impl super::Acquisition for Acquisition {

	fn provide_sample(&mut self, sample:(Complex<f64>, usize)) -> Result<(), &str> { match self.state {
		State::StageOne => self.stage_one.provide_sample(sample),
		State::StageTwo => self.stage_two.provide_sample(sample),
	}}

	fn block_for_result(&mut self) -> Result<Option<super::AcquisitionResult>, &str> {
		let (next_state, ans) = match self.state {
			State::StageOne => {
				match self.stage_one.block_for_result() {
					Ok(Some(super::AcquisitionResult{doppler_hz, doppler_step_hz, code_phase:_, mf_response:_, mf_len:_, input_power_total:_})) => { 
						self.stage_two.doppler_freqs.clear();

						let mut freq:f64 = doppler_hz - (0.5*doppler_step_hz);
						while freq < doppler_hz + (0.5*doppler_step_hz) {
							self.stage_two.doppler_freqs.push(freq);
							freq += self.stage_two_resolution_hz;
						}
						
						(State::StageTwo, Ok(None))
					},
					_ => (State::StageOne, Ok(None))
				}
			},
			State::StageTwo => {
				match self.stage_two.block_for_result() {
					Ok(Some(acq)) => {
						// The test statistic threshold is set to zero for stage two, so we'll always get a result after the first complete
						// symbol and we'll compare it to the threshold here.  Either way, we return to stage one 
						if acq.test_statistic() > self.test_statistic_threshold { (State::StageOne, Ok(Some(acq))) }
						else                                                    { (State::StageOne, Ok(None))      }
					},
					_ => (State::StageTwo, Ok(None))
				}
			},
		};

		self.state = next_state;

		ans
	}

}