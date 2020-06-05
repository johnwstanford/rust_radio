
use ::serde::{Serialize, Deserialize};

use crate::DigSigProcErr;
use crate::utils::bools_to_int;

// TODO: think about whether Copy and Clone are really necessary
#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub struct Body {
	pub cic:f64, 
	pub omega0:f64, 
	pub cis:f64, 
	pub i0:f64, 
	pub crc:f64, 
	pub omega:f64, 
	pub omega_dot:f64, 
	pub iode:u8, 
	pub idot:f64
}

impl Body {

	pub fn new(bits:&[bool; 240]) -> Result<Body, DigSigProcErr> {
		let cic:f64       = (bools_to_int::bool_slice_to_i16(&bits[ 48..64 ]) as f64) * (2.0_f64).powi(-29);
		let omega0:f64    = (bools_to_int::bool_slice_to_i32(&bits[ 64..96 ]) as f64) * (2.0_f64).powi(-31);
		let cis:f64       = (bools_to_int::bool_slice_to_i16(&bits[ 96..112]) as f64) * (2.0_f64).powi(-29);
		let i0:f64        = (bools_to_int::bool_slice_to_i32(&bits[112..144]) as f64) * (2.0_f64).powi(-31);
		let crc:f64       = (bools_to_int::bool_slice_to_i16(&bits[144..160]) as f64) * (2.0_f64).powi(-5);
		let omega:f64     = (bools_to_int::bool_slice_to_i32(&bits[160..192]) as f64) * (2.0_f64).powi(-31);
		let omega_dot:f64 = (bools_to_int::bool_slice_to_i32(&bits[192..216]) as f64) * (2.0_f64).powi(-43);
		let iode:u8       =  bools_to_int::bool_slice_to_u8( &bits[216..224]);
		let idot:f64      = (bools_to_int::bool_slice_to_i16(&bits[224..238]) as f64) * (2.0_f64).powi(-43);
		Ok(Body{ cic, omega0, cis, i0, crc, omega, omega_dot, iode, idot })		
	}

}