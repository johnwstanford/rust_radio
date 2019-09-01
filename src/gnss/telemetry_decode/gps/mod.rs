
#![allow(non_snake_case)]

extern crate num_complex;

use std::collections::VecDeque;
use self::num_complex::Complex;
use ::gnss::tracking;
use ::DigSigProcErr;

/*	GPS Telemetry Decoding Pipeline:
	- Preamble detector
	- Subframe group
	- Subframe decode
*/

const SUBFRAME_SIZE_W_PARITY_BITS:usize = 300;
const SUBFRAME_SIZE_DATA_ONLY_BITS:usize = 240;

mod preamble_detector;
pub mod l1_ca_subframe;

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
	   { return Err(DigSigProcErr::InvalidTelemetryData); }

	for bit_idx in 0..24 { ans[bit_idx] = subframe[bit_idx]; }
	for sf_idx in 1..10 {
		for bit_idx in 0..24 { ans[(24*sf_idx)+bit_idx] = subframe[(30*sf_idx)+bit_idx] ^ subframe[(30*sf_idx)-1]; }
	}

	Ok(ans)
}

pub fn get_subframes<T:Iterator<Item=(Complex<f64>, usize)>>(trk:&mut tracking::Tracking<T>) -> Result<Vec<([bool; SUBFRAME_SIZE_DATA_ONLY_BITS], usize)>, DigSigProcErr> {

	let mut detector = preamble_detector::new_preamble_detector();
	let mut detection_buffer:VecDeque<(bool, usize)> = VecDeque::new();

	// Determine subframe locations
	while let Ok((prompt, prompt_idx)) = trk.next() {
		let b:bool = prompt.re > 0.0;
		detector.apply(b);
		detection_buffer.push_back((b, prompt_idx));
		if detector.get_result().is_ok() { break; }
	}

	match (detector.get_result(), detector.is_inverse_sense()) {
		(Ok(bit_locations), Ok(is_inverse_sense)) => {
			// Drop bits to get to the start of the next subframe
			for _ in 0..bit_locations { detection_buffer.pop_front(); }
			let mut ans:Vec<([bool; SUBFRAME_SIZE_DATA_ONLY_BITS], usize)> = vec![];

			// Keep reading from the tracker until it runs out
			while let Ok((prompt, prompt_idx)) = trk.next() {
				detection_buffer.push_back((prompt.re > 0.0, prompt_idx));

				while detection_buffer.len() >= SUBFRAME_SIZE_W_PARITY_BITS {
					let mut next_subframe = [false; SUBFRAME_SIZE_W_PARITY_BITS];
					if let Some((b, first_idx)) = detection_buffer.pop_front() {
						next_subframe[0] = b ^ is_inverse_sense;
						for i in 1..SUBFRAME_SIZE_W_PARITY_BITS {
							match detection_buffer.pop_front() {
								Some((b, _)) => next_subframe[i] = b ^ is_inverse_sense,
								None => return Ok(ans),
							}
						}
						ans.push((data_recover(next_subframe)?, first_idx));			
					} else { return Ok(ans) }
				}

			}

			Ok(ans)

		},
		(_, _) => Err(DigSigProcErr::InvalidTelemetryData),
	}

}