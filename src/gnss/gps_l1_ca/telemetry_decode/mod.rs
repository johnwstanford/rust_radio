
#![allow(non_snake_case)]

use std::collections::VecDeque;

use crate::DigSigProcErr;
use crate::block::{BlockFunctionality, BlockResult};
use crate::gnss::common::tracking::TrackReport;

/*	GPS Telemetry Decoding Pipeline:
	- Preamble detector
	- Subframe group
	- Subframe decode
*/

const SUBFRAME_SIZE_W_PARITY_BITS:usize = 300;
const SUBFRAME_SIZE_DATA_ONLY_BITS:usize = 240;

mod preamble_detector;

pub mod subframe;

fn parity_check(word:&Vec<bool>, last_D29:bool, last_D30:bool) -> bool {
	if word.len() != 30 { panic!("Word length must be 30 bits"); }

	let d:Vec<bool> = word.iter().take(24).map(|b| b ^ last_D30).collect();

	let mut parity:Vec<bool> = vec![];
	parity.push(last_D29 ^ d[0] ^ d[1] ^ d[2] ^ d[4] ^ d[5] ^ d[9]  ^ d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[16] ^ d[17] ^ d[19] ^ d[22]);
	parity.push(last_D30 ^ d[1] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[17] ^ d[18] ^ d[20] ^ d[23]);
	parity.push(last_D29 ^ d[0] ^ d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[7]  ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[18] ^ d[19] ^ d[21]);
	parity.push(last_D30 ^ d[1] ^ d[3] ^ d[4] ^ d[5] ^ d[7] ^ d[8]  ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[19] ^ d[20] ^ d[22]);
	parity.push(last_D30 ^ d[0] ^ d[2] ^ d[4] ^ d[5] ^ d[6] ^ d[8]  ^ d[9]  ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[20] ^ d[21] ^ d[23]);
	parity.push(last_D29 ^ d[2] ^ d[4] ^ d[5] ^ d[7] ^ d[8] ^ d[9]  ^ d[10] ^ d[12] ^ d[14] ^ d[18] ^ d[21] ^ d[22] ^ d[23]);

	word.iter().skip(24).zip(parity.iter()).map(|(a,b)| a == b).fold(true, |a,b| a & b)
}

fn data_recover(subframe:[bool; SUBFRAME_SIZE_W_PARITY_BITS]) -> Result<[bool; SUBFRAME_SIZE_DATA_ONLY_BITS], DigSigProcErr> {
	let mut ans:[bool; SUBFRAME_SIZE_DATA_ONLY_BITS] = [false; SUBFRAME_SIZE_DATA_ONLY_BITS];

	if !parity_check(&subframe[  0..30 ].to_vec(), false,         false) ||
	   !parity_check(&subframe[ 30..60 ].to_vec(), subframe[28],  subframe[29]) ||
	   !parity_check(&subframe[ 60..90 ].to_vec(), subframe[58],  subframe[59]) ||
	   !parity_check(&subframe[ 90..120].to_vec(), subframe[88],  subframe[89]) ||
	   !parity_check(&subframe[120..150].to_vec(), subframe[118], subframe[119]) ||
	   !parity_check(&subframe[150..180].to_vec(), subframe[148], subframe[149]) ||
	   !parity_check(&subframe[180..210].to_vec(), subframe[178], subframe[179]) ||
	   !parity_check(&subframe[210..240].to_vec(), subframe[208], subframe[209]) ||
	   !parity_check(&subframe[240..270].to_vec(), subframe[238], subframe[239]) ||
	   !parity_check(&subframe[270..300].to_vec(), subframe[268], subframe[269])
	   { return Err(DigSigProcErr::InvalidTelemetryData("Bad parity check")); }

	for bit_idx in 0..24 { ans[bit_idx] = subframe[bit_idx]; }
	for sf_idx in 1..10 {
		for bit_idx in 0..24 { ans[(24*sf_idx)+bit_idx] = subframe[(30*sf_idx)+bit_idx] ^ subframe[(30*sf_idx)-1]; }
	}

	Ok(ans)
}

pub struct TelemetryDecoder {
	detector: preamble_detector::PreambleDetector,
	detection_buffer:VecDeque<(bool, usize)>,
	state: TelemetryDecoderState,
	idx_buffer: VecDeque<usize>
}

impl BlockFunctionality<(), bool, TrackReport, (usize, subframe::Subframe, usize)> for TelemetryDecoder {

	fn control(&mut self, _:&()) -> Result<bool, &'static str> {
		Ok(true)
	}

	fn apply(&mut self, input:&TrackReport) -> BlockResult<(usize, subframe::Subframe, usize)> {
		let bit = (input.prompt_i > 0.0, input.sample_idx);
		match self.apply_sample(bit) {
			TelemetryDecoderResult::NotReady => BlockResult::NotReady,
			TelemetryDecoderResult::Err(e)   => BlockResult::Err(e),
			TelemetryDecoderResult::Ok(sf, _, last_idx) => BlockResult::Ready((input.id, sf, last_idx))
		}
	}

}

pub enum TelemetryDecoderResult {
	NotReady,
	Ok(subframe::Subframe, [bool; SUBFRAME_SIZE_DATA_ONLY_BITS], usize),
	Err(DigSigProcErr),
}

impl TelemetryDecoder {

	pub fn new() -> TelemetryDecoder {
		TelemetryDecoder{ detector: preamble_detector::new_preamble_detector(), 
						  detection_buffer: VecDeque::new(),
						  state: TelemetryDecoderState::LookingForPreamble,
						  idx_buffer: VecDeque::new() }
	}

	pub fn initialize(&mut self) {
		self.detector.initialize();
		self.detection_buffer.clear();
		self.idx_buffer.clear();
		self.state = TelemetryDecoderState::LookingForPreamble;
	}

	/// Takes a bit tuple in the form of a boolean representing a bit and a usize representing the sample index where this symbol ended.
	/// Returns a TelemetryDecoderResult
	pub fn apply_sample(&mut self, bit:(bool, usize)) -> TelemetryDecoderResult {
		self.idx_buffer.push_back(bit.1);
		if self.idx_buffer.len() == 3 {
			let d_idx0 = self.idx_buffer[1] - self.idx_buffer[0];
			let d_idx1 = self.idx_buffer[2] - self.idx_buffer[1];
			if d_idx1 > 2*d_idx0 {
				self.initialize();
				return TelemetryDecoderResult::NotReady;
			}
			self.idx_buffer.pop_front();
		} else if self.idx_buffer.len() > 3 {
			// Should never happen but if it does, we want to initialize
			self.initialize();
			return TelemetryDecoderResult::NotReady;
		}

		match self.state {
			TelemetryDecoderState::LookingForPreamble => {
				self.detector.apply(bit.0);
				self.detection_buffer.push_back(bit);
				match (self.detector.get_result(), self.detector.is_inverse_sense()) {
					(Ok(bit_locations), Ok(is_inverse_sense)) => {
						// Preamble detected
						self.state = TelemetryDecoderState::DecodingSubframes{ is_inverse_sense };

						// Drop bits to get to the start of the next subframe
						for _ in 0..bit_locations { self.detection_buffer.pop_front(); }
						
						// TODO account for the fact that there might be a few subframes available in the buffer; for now, just return it next method call
						TelemetryDecoderResult::NotReady
					},
					(_, _) => {
						// Preamble not yet detected, don't change state
						// TODO: panic or return Err if one if Ok(_) but not the other
						TelemetryDecoderResult::NotReady
					}
				}
			},
			TelemetryDecoderState::DecodingSubframes{ is_inverse_sense } => {
				self.detection_buffer.push_back(bit);

				if self.detection_buffer.len() >= SUBFRAME_SIZE_W_PARITY_BITS {
					let mut next_subframe = [false; SUBFRAME_SIZE_W_PARITY_BITS];
					let last_idx:usize = match self.detection_buffer.get(SUBFRAME_SIZE_W_PARITY_BITS-1) {
						Some((_, idx)) => *idx,
						None => panic!("Thought we had enough bits in the buffer, but didn't")
					};

					// Unload the detection buffer
					for i in 0..SUBFRAME_SIZE_W_PARITY_BITS {
						match self.detection_buffer.pop_front() {
							Some((b, _)) => next_subframe[i] = b ^ is_inverse_sense,
							None => return TelemetryDecoderResult::Err(DigSigProcErr::InvalidTelemetryData("Not enough bits in detection_buffer")),
						}
					}

					// Parity-check the whole subframe and return the actual data without the parity bits
					match data_recover(next_subframe) {
						Ok(bits) => {
							// If the bits passed the parity check, try to actually decode the data
							match subframe::decode(bits) {
								Ok(sf) => TelemetryDecoderResult::Ok(sf, bits, last_idx),
								Err(e) => TelemetryDecoderResult::Err(e)		
							}
						},
						Err(e) => TelemetryDecoderResult::Err(e)
					}

				} else { TelemetryDecoderResult::NotReady }

			},
		}
	}
}

enum TelemetryDecoderState {
	LookingForPreamble,
	DecodingSubframes{ is_inverse_sense:bool },
}