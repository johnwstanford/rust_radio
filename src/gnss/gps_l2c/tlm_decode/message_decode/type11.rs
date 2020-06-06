
use ::serde::{Serialize, Deserialize};

use crate::utils::bools_to_int;
use crate::DigSigProcErr;

#[derive(Debug, Serialize, Deserialize)]
pub struct Body {
	t_oe: f64	
}

impl Body {
	
	pub fn new(bits:&[bool]) -> Result<Self, DigSigProcErr> {
		if bits.len() == 238 {
			// Scale factors given in IS-GPS-200K, Table 30-I
			let t_oe = (bools_to_int::to_u16(&bits[0..11])? as f64) * 300.0;

			Ok(Self{ t_oe })

		} else {
			Err(DigSigProcErr::InvalidTelemetryData("Expected a bool slice of length 238 in type11::Body::new"))
		}
	}

}