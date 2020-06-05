
use ::serde::{Serialize, Deserialize};

use crate::DigSigProcErr;
use crate::utils::bools_to_int;

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub struct Body {
	pub data_id:u8, 
	pub sv_id:u8, 
	pub page:Page
}

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub enum Page {
	AlmanacData{e:f64, t_oa:u32, delta_i:f64, omega_dot:f64, sv_health:u8, sqrt_a:f64, omega0:f64, omega:f64, m0:f64, af0:f64, af1:f64},
	Page25{t_oa:u32, WN_a:u8, sv_health:[u8; 24]},
}

impl Body {

	pub fn new(bits:&[bool; 240]) -> Result<Body, DigSigProcErr> {
		let data_id:u8 = bools_to_int::to_u8(&bits[48..50])?;
		let sv_id:u8   = bools_to_int::to_u8(&bits[50..56])?;
		let page:Page = match sv_id {
			1..=24 => {
				let e:f64         = (bools_to_int::to_u16(&bits[ 56..72 ])? as f64) * (2.0_f64).powi(-21);
				let t_oa:u32      =  bools_to_int::to_u32(&bits[ 72..80 ])? * 2_u32.pow(12);
				let delta_i:f64   = (bools_to_int::to_i16(&bits[ 80..96 ])? as f64) * (2.0_f64).powi(-19);
				let omega_dot:f64 = (bools_to_int::to_i16(&bits[ 96..112])? as f64) * (2.0_f64).powi(-38);
				let sv_health:u8  =  bools_to_int::to_u8( &bits[112..120])?;
				let sqrt_a:f64    = (bools_to_int::to_u32(&bits[120..144])? as f64) * (2.0_f64).powi(-11);
				let omega0:f64    = (bools_to_int::to_i32(&bits[144..168])? as f64) * (2.0_f64).powi(-23);
				let omega:f64     = (bools_to_int::to_i32(&bits[168..192])? as f64) * (2.0_f64).powi(-23);
				let m0:f64        = (bools_to_int::to_i32(&bits[192..216])? as f64) * (2.0_f64).powi(-23);
				let af0:f64       = (bools_to_int::to_i16(&[&bits[216..224], &bits[235..238]].concat())? as f64) * (2.0_f64).powi(-20);
				let af1:f64       = (bools_to_int::to_i32(&bits[224..235])? as f64) * (2.0_f64).powi(-18);
				Page::AlmanacData{e, t_oa, delta_i, omega_dot, sv_health, sqrt_a, omega0, omega, m0, af0, af1}
			},
			25 => {
				let t_oa:u32 = bools_to_int::to_u32(&bits[56..64])? * 2_u32.pow(12);
				let WN_a:u8  = bools_to_int::to_u8(&bits[64..72])?;
				let mut sv_health:[u8; 24] = [0; 24];
				for i in 0..24 {
					sv_health[i] = bools_to_int::to_u8(&bits[(72+(i*6))..(78+(i*6))])?;
				}
				Page::Page25{t_oa, WN_a, sv_health}
			},
			_ => return Err(DigSigProcErr::InvalidTelemetryData("Page number other than 1 through 25")),
		};
		Ok(Body{ data_id, sv_id, page })		
	}

}