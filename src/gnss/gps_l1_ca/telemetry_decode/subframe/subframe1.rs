
use ::serde::{Serialize, Deserialize};

use crate::DigSigProcErr;
use crate::utils::bools_to_int;

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
#[allow(non_camel_case_types)]
pub enum CodeOnL2 {
	Reserved,
	P_Code,
	CA_Code,
}

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub struct Body {
	pub week_number:u16, 
	pub code_on_l2:CodeOnL2, 
	pub ura_index:u8, 
	pub sv_health:u8, 
	pub iodc:u16, 
	pub t_gd:f64, 
	pub t_oc:u32, 
	pub a_f2:f64, 
	pub a_f1:f64, 
	pub a_f0:f64
}

impl Body {
	
	// TODO: change this to take a &[u8; 30]
	pub fn new(bits:&[bool; 240]) -> Result<Body, DigSigProcErr> {
		let week_number:u16 = bools_to_int::to_u16(&bits[48..58])?;
		let code_on_l2 = match (bits[58], bits[59]) {
			(false, false) => CodeOnL2::Reserved,
			(false, true ) => CodeOnL2::P_Code,
			(true,  false) => CodeOnL2::CA_Code,
			(true,  true ) => return Err(DigSigProcErr::InvalidTelemetryData("Invalid code_on_l2 field in subframe 1")),
		};
		let ura_index:u8 =  bools_to_int::to_u8(&bits[60..64])?;
		let sv_health:u8 =  bools_to_int::to_u8(&bits[64..70])?;
		let iodc:u16     =  bools_to_int::to_u16(&[&bits[70..72], &bits[168..176]].concat())?;
		let t_gd:f64     = (bools_to_int::to_i8(&bits[160..168])? as f64) * (2.0_f64).powi(-31);
		let t_oc:u32     =  bools_to_int::to_u32(&bits[176..192])? * 16_u32;
		let a_f2:f64     = (bools_to_int::to_i8(&bits[192..200])? as f64) * (2.0_f64).powi(-55);
		let a_f1:f64     = (bools_to_int::to_i16(&bits[200..216])? as f64) * (2.0_f64).powi(-43);
		let a_f0:f64     = (bools_to_int::to_i32(&bits[216..238])? as f64) * (2.0_f64).powi(-31);

		Ok(Body{ week_number, code_on_l2, ura_index, sv_health, iodc, t_gd, t_oc, a_f2, a_f1, a_f0 })		
	}

}