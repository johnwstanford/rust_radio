
use ::serde::{Serialize, Deserialize};

use crate::utils::bools_to_int;
use crate::DigSigProcErr;

#[derive(Debug, Serialize, Deserialize)]
pub struct Body {
	week_num: u16,
	l1_health: bool,
	l2_health: bool,
	l5_health: bool,
}

impl Body {
	
	pub fn new(bits:&[bool]) -> Result<Self, DigSigProcErr> {
		if bits.len() == 238 {
			let week_num  = bools_to_int::to_u16(&bits[0..13])?;

			// For these flags, false indicates "Signal OK" and true indicates "Signal bad or unavailable" (IS-GPS-200K, section 30.3.3.1.1.2)
			let l1_health = bits[13];
			let l2_health = bits[14];
			let l5_health = bits[15];

			Ok(Self{ week_num, l1_health, l2_health, l5_health })

		} else {
			Err(DigSigProcErr::InvalidTelemetryData("Expected a bool slice of length 238 in type10::Body::new"))
		}
	}

}