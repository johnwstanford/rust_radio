
extern crate byteorder;
extern crate serde;

use self::serde::{Serialize, Deserialize};
use ::DigSigProcErr;
use ::utils;

#[derive(Debug, Serialize, Deserialize)]
pub struct CommonFields {
	time_of_week_truncated:u32,
	subframe_id:u8,
	start_sample_idx:usize,
}

#[derive(Debug, Serialize, Deserialize)]
pub enum Subframe {
	Subframe1{common:CommonFields, week_number:u16, code_on_l2:CodeOnL2, ura_index:u8, sv_health:u8, iodc:u16, t_gd:f64, t_oc:u32, a_f2:f64, a_f1:f64, a_f0:f64},
	Subframe2{common:CommonFields, iode:u8, crs:f64, dn:f64, m0:f64, cuc:f64, e:f64, cus:f64, sqrt_a:f64, t_oe:f64, fit_interval:bool, aodo:u8 },
	Subframe3{common:CommonFields, cic:f64, omega0:f64, cis:f64, i0:f64, crc:f64, omega:f64, omega_dot:f64, iode:u8, idot:f64},
	Subframe4{common:CommonFields, data_id:u8, sv_id:u8},
	Subframe5{common:CommonFields, data_id:u8, sv_id:u8, page:Subframe5},
}

#[derive(Debug, Serialize, Deserialize)]
pub enum Subframe4 {
	AlmanacData,
	NavigationMessageCorrectionTable,
	SpecialMessages,
	IonosphereAndUTC,
	AntispoofAndHealth,
	Reserved,
}

#[derive(Debug, Serialize, Deserialize)]
pub enum Subframe5 {
	AlmanacData{e:f64, t_oa:u32, delta_i:f64, omega_dot:f64, sv_health:u8, sqrt_a:f64, omega0:f64, omega:f64, m0:f64, af0:f64, af1:f64},
	Page25{t_oa:u32, WN_a:u8, sv_health:[u8; 24]},
}

#[derive(Debug, Serialize, Deserialize)]
#[allow(non_camel_case_types)]
pub enum CodeOnL2 {
	Reserved,
	P_Code,
	CA_Code,
}

pub fn decode(bits:[bool; 240], start_sample_idx:usize) -> Result<Subframe, DigSigProcErr> {
	let time_of_week_truncated:u32 = utils::bool_slice_to_u32(&bits[24..41]);
	let subframe_id:u8 = utils::bool_slice_to_u8(&bits[43..46]);
	let common = CommonFields{ time_of_week_truncated, subframe_id, start_sample_idx };

	match subframe_id {
		1 => {
			let week_number:u16 = utils::bool_slice_to_u16(&bits[48..58]);
			let code_on_l2 = match (bits[58], bits[59]) {
				(false, false) => CodeOnL2::Reserved,
				(false, true ) => CodeOnL2::P_Code,
				(true,  false) => CodeOnL2::CA_Code,
				(true,  true ) => return Err(DigSigProcErr::InvalidTelemetryData),
			};
			let ura_index:u8 =  utils::bool_slice_to_u8(&bits[60..64]);
			let sv_health:u8 =  utils::bool_slice_to_u8(&bits[64..70]);
			let iodc:u16     =  utils::bool_slice_to_u16(&[&bits[70..72], &bits[168..176]].concat());
			let t_gd:f64     = (utils::bool_slice_to_i8(&bits[160..168]) as f64) * (2.0_f64).powi(-31);
			let t_oc:u32     =  utils::bool_slice_to_u32(&bits[176..192]) * 16_u32;
			let a_f2:f64     = (utils::bool_slice_to_i8(&bits[192..200]) as f64) * (2.0_f64).powi(-55);
			let a_f1:f64     = (utils::bool_slice_to_i16(&bits[200..216]) as f64) * (2.0_f64).powi(-43);
			let a_f0:f64     = (utils::bool_slice_to_i32(&bits[216..238]) as f64) * (2.0_f64).powi(-31);

			Ok(Subframe::Subframe1{ common, week_number, code_on_l2, ura_index, sv_health, iodc, t_gd, t_oc, a_f2, a_f1, a_f0 })
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
			let data_id:u8 = utils::bool_slice_to_u8(&bits[48..50]);
			let sv_id:u8   = utils::bool_slice_to_u8(&bits[50..56]);
			Ok(Subframe::Subframe4{ common, data_id, sv_id })
		},
		5 => {
			let data_id:u8 = utils::bool_slice_to_u8(&bits[48..50]);
			let sv_id:u8   = utils::bool_slice_to_u8(&bits[50..56]);
			let page:Subframe5 = match sv_id {
				1..=24 => {
					let e:f64         = (utils::bool_slice_to_u16(&bits[ 56..72 ]) as f64) * (2.0_f64).powi(-21);
					let t_oa:u32      =  utils::bool_slice_to_u32(&bits[ 72..80 ]) * 2_u32.pow(12);
					let delta_i:f64   = (utils::bool_slice_to_i16(&bits[ 80..96 ]) as f64) * (2.0_f64).powi(-19);
					let omega_dot:f64 = (utils::bool_slice_to_i16(&bits[ 96..112]) as f64) * (2.0_f64).powi(-38);
					let sv_health:u8  =  utils::bool_slice_to_u8( &bits[112..120]);
					let sqrt_a:f64    = (utils::bool_slice_to_u32(&bits[120..144]) as f64) * (2.0_f64).powi(-11);
					let omega0:f64    = (utils::bool_slice_to_i32(&bits[144..168]) as f64) * (2.0_f64).powi(-23);
					let omega:f64     = (utils::bool_slice_to_i32(&bits[168..192]) as f64) * (2.0_f64).powi(-23);
					let m0:f64        = (utils::bool_slice_to_i32(&bits[192..216]) as f64) * (2.0_f64).powi(-23);
					let af0:f64       = (utils::bool_slice_to_i16(&[&bits[216..224], &bits[235..238]].concat()) as f64) * (2.0_f64).powi(-20);
					let af1:f64       = (utils::bool_slice_to_i32(&bits[224..235]) as f64) * (2.0_f64).powi(-18);
					Subframe5::AlmanacData{e, t_oa, delta_i, omega_dot, sv_health, sqrt_a, omega0, omega, m0, af0, af1}
				},
				25 => {
					let t_oa:u32 = utils::bool_slice_to_u32(&bits[56..64]) * 2_u32.pow(12);
					let WN_a:u8  = utils::bool_slice_to_u8(&bits[64..72]);
					let mut sv_health:[u8; 24] = [0; 24];
					for i in 0..24 {
						sv_health[i] = utils::bool_slice_to_u8(&bits[(72+(i*6))..(78+(i*6))]);
					}
					Subframe5::Page25{t_oa, WN_a, sv_health}
				},
				_ => return Err(DigSigProcErr::InvalidTelemetryData),
			};
			Ok(Subframe::Subframe5{ common, data_id, sv_id, page })
		},
		_ => Err(DigSigProcErr::InvalidTelemetryData),
	}
}