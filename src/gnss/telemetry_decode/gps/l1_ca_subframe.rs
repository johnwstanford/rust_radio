
extern crate byteorder;
extern crate serde;

use self::serde::{Serialize, Deserialize};
use ::DigSigProcErr;
use ::utils;

#[derive(Debug, Serialize, Deserialize)]
pub struct CommonFields {
	time_of_week_truncated:u32,
	subframe_id:u8,
}

#[derive(Debug, Serialize, Deserialize)]
pub enum Subframe {
	Subframe1{common:CommonFields, week_number:u16, code_on_l2:CodeOnL2, ura_index:u8, sv_health:u8, iodc:u16},
	Subframe2{common:CommonFields, iode:u8, crs:f64, dn:f64, m0:f64, cuc:f64, e:f64, cus:f64, sqrt_a:f64, t_oe:f64, fit_interval:bool, aodo:u8 },
	Subframe3{common:CommonFields, cic:f64, omega0:f64, cis:f64, i0:f64, crc:f64, omega:f64, omega_dot:f64, iode:u8, idot:f64},
	Subframe4{common:CommonFields},
	Subframe5{common:CommonFields},
}

#[derive(Debug, Serialize, Deserialize)]
#[allow(non_camel_case_types)]
pub enum CodeOnL2 {
	Reserved,
	P_Code,
	CA_Code,
}

pub fn decode(bits:[bool; 240]) -> Result<Subframe, DigSigProcErr> {
	let time_of_week_truncated:u32 = utils::bool_slice_to_u32(&bits[24..41]);
	let subframe_id:u8 = utils::bool_slice_to_u8(&bits[43..46]);
	let common = CommonFields{ time_of_week_truncated, subframe_id };

	match subframe_id {
		1 => {
			let week_number:u16 = utils::bool_slice_to_u16(&bits[48..58]);
			let code_on_l2 = match (bits[58], bits[59]) {
				(false, false) => CodeOnL2::Reserved,
				(false, true ) => CodeOnL2::P_Code,
				(true,  false) => CodeOnL2::CA_Code,
				(true,  true ) => return Err(DigSigProcErr::InvalidTelemetryData),
			};
			let ura_index:u8 = utils::bool_slice_to_u8(&bits[60..64]);
			let sv_health:u8 = utils::bool_slice_to_u8(&bits[64..70]);
			let iodc:u16     = utils::bool_slice_to_u16(&[&bits[70..72], &bits[168..176]].concat());

			Ok(Subframe::Subframe1{ common, week_number, code_on_l2, ura_index, sv_health, iodc })
		},
		2 => {
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
			Ok(Subframe::Subframe2{ common, iode, crs, dn, m0, cuc, e, cus, sqrt_a, t_oe, fit_interval, aodo })
		},
		3 => {
			let cic:f64       = (utils::bool_slice_to_i16(&bits[ 48..64 ]) as f64) * (2.0_f64).powi(-29);
			let omega0:f64    = (utils::bool_slice_to_i32(&bits[ 64..96 ]) as f64) * (2.0_f64).powi(-31);
			let cis:f64       = (utils::bool_slice_to_i16(&bits[ 96..112]) as f64) * (2.0_f64).powi(-29);
			let i0:f64        = (utils::bool_slice_to_i32(&bits[112..144]) as f64) * (2.0_f64).powi(-31);
			let crc:f64       = (utils::bool_slice_to_i16(&bits[144..160]) as f64) * (2.0_f64).powi(-5);
			let omega:f64     = (utils::bool_slice_to_i32(&bits[160..192]) as f64) * (2.0_f64).powi(-31);
			let omega_dot:f64 = (utils::bool_slice_to_i32(&bits[192..216]) as f64) * (2.0_f64).powi(-43);
			let iode:u8       =  utils::bool_slice_to_u8( &bits[216..224]);
			let idot:f64      = (utils::bool_slice_to_i16(&bits[224..238]) as f64) * (2.0_f64).powi(-43);
			Ok(Subframe::Subframe3{ common, cic, omega0, cis, i0, crc, omega, omega_dot, iode, idot })
		},
		4 => {
			Ok(Subframe::Subframe4{ common })
		},
		5 => {
			Ok(Subframe::Subframe5{ common })
		},
		_ => Err(DigSigProcErr::InvalidTelemetryData),
	}
}