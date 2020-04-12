
extern crate byteorder;
extern crate serde;

use self::serde::{Serialize, Deserialize};
use ::DigSigProcErr;
use ::utils;

pub mod subframe1;
pub mod subframe2;
pub mod subframe3;
pub mod subframe4;
pub mod subframe5;

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub struct Subframe {
	pub time_of_week_truncated:u32,
	pub subframe_id:u8,
	pub body:SubframeBody,
}

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub enum SubframeBody {
	Subframe1(subframe1::Body),
	Subframe2(subframe2::Body),
	Subframe3(subframe3::Body),
	Subframe4(subframe4::Body),
	Subframe5(subframe5::Body),
}

impl Subframe {

	pub fn time_of_week(&self) -> f64 { (self.time_of_week_truncated as f64) * 6.0 }

}

pub fn decode(bits:[bool; 240]) -> Result<Subframe, DigSigProcErr> {
	let time_of_week_truncated:u32 = utils::bool_slice_to_u32(&bits[24..41]);
	let subframe_id:u8 = utils::bool_slice_to_u8(&bits[43..46]);

	let body = match subframe_id {
		1 => SubframeBody::Subframe1(subframe1::Body::new(&bits)?),
		2 => SubframeBody::Subframe2(subframe2::Body::new(&bits)?),
		3 => SubframeBody::Subframe3(subframe3::Body::new(&bits)?),
		4 => SubframeBody::Subframe4(subframe4::Body::new(&bits)?),
		5 => SubframeBody::Subframe5(subframe5::Body::new(&bits)?),
		_ => return Err(DigSigProcErr::InvalidTelemetryData("Subframe number other than 1 through 5")),
	};

	Ok(Subframe{ time_of_week_truncated, subframe_id, body })
}