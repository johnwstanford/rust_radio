
use ::serde::{Serialize, Deserialize};

use crate::utils::bools_to_int;
use crate::DigSigProcErr;

#[derive(Debug, Serialize, Deserialize)]
pub struct Message {
	pub prn: u8,
	pub type_id:u8,
	pub time_of_week_truncated:u32,
	pub alert_flag:bool,
	pub body:MessageBody,
}

#[derive(Debug, Serialize, Deserialize)]
pub enum MessageBody {
	Type10(type10::Body),
	Type11(type11::Body),
	Unknown
}

pub mod type10;
pub mod type11;

impl Message {

	pub fn new(bits:&[bool]) -> Result<Self, DigSigProcErr> {

		if bits.len() == 276 {
			let prn                    = bools_to_int::to_u8( &bits[ 8..14])?;
			let type_id                = bools_to_int::to_u8( &bits[14..20])?;
			let time_of_week_truncated = bools_to_int::to_u32(&bits[20..37])?;
			let alert_flag             = bits[38];
			let body = match type_id {
				10 => MessageBody::Type10(type10::Body::new(&bits[38..])?),
				11 => MessageBody::Type11(type11::Body::new(&bits[38..])?),
				_  => MessageBody::Unknown,
			};
			Ok(Self{ prn, type_id, time_of_week_truncated, alert_flag, body })
		} else {
			Err(DigSigProcErr::InvalidTelemetryData("Expected a 276-bit message with CRC removed but got a different size"))
		}

	}

}

