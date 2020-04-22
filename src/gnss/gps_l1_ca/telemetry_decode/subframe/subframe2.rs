
use ::serde::{Serialize, Deserialize};

use crate::DigSigProcErr;
use crate::utils;

// TODO: think about whether Copy and Clone are really necessary
#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub struct Body {
	pub iode:u8, 
	pub crs:f64, 
	pub dn:f64, 
	pub m0:f64, 
	pub cuc:f64, 
	pub e:f64, 
	pub cus:f64, 
	pub sqrt_a:f64, 
	pub t_oe:f64, 
	pub fit_interval:bool, 
	pub aodo:u8 
}

impl Body {

	// TODO: change this to take a &[u8; 30]
	pub fn new(bits:&[bool; 240]) -> Result<Body, DigSigProcErr> {
		let iode:u8    =  utils::bool_slice_to_u8( &bits[ 48..56 ]);
		let crs:f64    = (utils::bool_slice_to_i16(&bits[ 56..72 ]) as f64) * (2.0_f64).powi(-5);
		let dn:f64     = (utils::bool_slice_to_i16(&bits[ 72..88 ]) as f64) * (2.0_f64).powi(-43);
		let m0:f64     = (utils::bool_slice_to_i32(&bits[ 88..120]) as f64) * (2.0_f64).powi(-31);
		let cuc:f64    = (utils::bool_slice_to_i16(&bits[120..136]) as f64) * (2.0_f64).powi(-29);
		let e:f64      = (utils::bool_slice_to_u32(&bits[136..168]) as f64) * (2.0_f64).powi(-33);
		let cus:f64    = (utils::bool_slice_to_i16(&bits[168..184]) as f64) * (2.0_f64).powi(-29);
		let sqrt_a:f64 = (utils::bool_slice_to_u32(&bits[184..216]) as f64) * (2.0_f64).powi(-19);
		let t_oe:f64   = (utils::bool_slice_to_u16(&bits[216..232]) as f64) * (2.0_f64).powi(4);
		let fit_interval:bool = bits[233];
		let aodo:u8    =  utils::bool_slice_to_u8( &bits[234..239]);
		Ok(Body{ iode, crs, dn, m0, cuc, e, cus, sqrt_a, t_oe, fit_interval, aodo })
	}

}