
extern crate byteorder;
extern crate serde;

use self::serde::{Serialize, Deserialize};
use ::DigSigProcErr;
use ::utils;

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub struct CommonFields {
	pub time_of_week_truncated:u32,
	pub subframe_id:u8,
	pub start_sample_idx:usize,
}

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub enum Subframe {
	Subframe1{common:CommonFields, week_number:u16, code_on_l2:CodeOnL2, ura_index:u8, sv_health:u8, iodc:u16, t_gd:f64, t_oc:u32, a_f2:f64, a_f1:f64, a_f0:f64},
	Subframe2{common:CommonFields, iode:u8, crs:f64, dn:f64, m0:f64, cuc:f64, e:f64, cus:f64, sqrt_a:f64, t_oe:f64, fit_interval:bool, aodo:u8 },
	Subframe3{common:CommonFields, cic:f64, omega0:f64, cis:f64, i0:f64, crc:f64, omega:f64, omega_dot:f64, iode:u8, idot:f64},
	Subframe4{common:CommonFields, data_id:u8, sv_id:u8, page:Subframe4},
	Subframe5{common:CommonFields, data_id:u8, sv_id:u8, page:Subframe5},
}

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub enum Subframe4 {
	AlmanacData{e:f64, t_oa:u32, delta_i:f64, omega_dot:f64, sv_health:u8, sqrt_a:f64, omega0:f64, omega:f64, m0:f64, af0:f64, af1:f64},
	NavigationMessageCorrectionTable{availability:u8, erd:[u8; 30]},
	SpecialMessages([u8; 22]),
	Page18{ alpha0:f64, alpha1:f64, alpha2:f64, alpha3:f64, beta0:f64, beta1:f64, beta2:f64, beta3:f64, a1:f64, a0:f64, t_ot:u32, wn_t:u8, delta_t_LS:i8, wn_LSF:u8, delta_t_LSF:i8 },
	Page25{ antispoof_and_config:[u8; 32], sv_health:[u8; 8] },
	Reserved,
}

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub enum Subframe5 {
	AlmanacData{e:f64, t_oa:u32, delta_i:f64, omega_dot:f64, sv_health:u8, sqrt_a:f64, omega0:f64, omega:f64, m0:f64, af0:f64, af1:f64},
	Page25{t_oa:u32, WN_a:u8, sv_health:[u8; 24]},
}

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
#[allow(non_camel_case_types)]
pub enum CodeOnL2 {
	Reserved,
	P_Code,
	CA_Code,
}

impl Subframe {

	pub fn time_of_week(&self) -> f64 { match self {
		Subframe::Subframe1{common, week_number:_, code_on_l2:_, ura_index:_, sv_health:_, iodc:_, t_gd:_, t_oc:_, a_f2:_, a_f1:_, a_f0:_} => (common.time_of_week_truncated as f64) * 6.0,
		Subframe::Subframe2{common, iode:_, crs:_, dn:_, m0:_, cuc:_, e:_, cus:_, sqrt_a:_, t_oe:_, fit_interval:_, aodo:_ } => (common.time_of_week_truncated as f64) * 6.0,
		Subframe::Subframe3{common, cic:_, omega0:_, cis:_, i0:_, crc:_, omega:_, omega_dot:_, iode:_, idot:_} => (common.time_of_week_truncated as f64) * 6.0,
		Subframe::Subframe4{common, data_id:_, sv_id:_, page:_} => (common.time_of_week_truncated as f64) * 6.0,
		Subframe::Subframe5{common, data_id:_, sv_id:_, page:_} => (common.time_of_week_truncated as f64) * 6.0,
	}}

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
				(true,  true ) => return Err(DigSigProcErr::InvalidTelemetryData("Invalid code_on_l2 field in subframe 1")),
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
			let page:Subframe4 = match sv_id {
				25..=32 => {
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
					Subframe4::AlmanacData{e, t_oa, delta_i, omega_dot, sv_health, sqrt_a, omega0, omega, m0, af0, af1}
				},
				52 => {
					let availability:u8 = utils::bool_slice_to_u8(&bits[56..58]);
					let mut erd:[u8; 30] = [0; 30];
					for i in 0..30 {
						erd[i] = utils::bool_slice_to_u8(&bits[(58+(i*6))..(64+(i*6))]);
					}
					Subframe4::NavigationMessageCorrectionTable{availability, erd}
				},
				55 => {
					let mut message:[u8; 22] = [0; 22];
					for i in 0..22 {
						message[i] = utils::bool_slice_to_u8(&bits[(56+(i*8))..(64+(i*8))]);
					}
					Subframe4::SpecialMessages(message)
				},
				56 => {
					let alpha0:f64     = (utils::bool_slice_to_i8(&bits[ 56..64 ]) as f64) * (2.0_f64).powi(-30);
					let alpha1:f64     = (utils::bool_slice_to_i8(&bits[ 64..72 ]) as f64) * (2.0_f64).powi(-27);
					let alpha2:f64     = (utils::bool_slice_to_i8(&bits[ 72..80 ]) as f64) * (2.0_f64).powi(-24);
					let alpha3:f64     = (utils::bool_slice_to_i8(&bits[ 80..88 ]) as f64) * (2.0_f64).powi(-24);
					let beta0:f64      = (utils::bool_slice_to_i8(&bits[ 88..96 ]) as f64) * (2.0_f64).powi(11);
					let beta1:f64      = (utils::bool_slice_to_i8(&bits[ 96..104]) as f64) * (2.0_f64).powi(14);
					let beta2:f64      = (utils::bool_slice_to_i8(&bits[104..112]) as f64) * (2.0_f64).powi(16);
					let beta3:f64      = (utils::bool_slice_to_i8(&bits[112..120]) as f64) * (2.0_f64).powi(16);
					let a1:f64         = (utils::bool_slice_to_i32(&bits[120..144]) as f64) * (2.0_f64).powi(-50); 
					let a0:f64         = (utils::bool_slice_to_i32(&bits[144..176]) as f64) * (2.0_f64).powi(-30);
					let t_ot:u32       =  utils::bool_slice_to_u32(&bits[176..184]) * (2_u32).pow(12);
					let wn_t:u8        =  utils::bool_slice_to_u8(&bits[184..192]);
					let delta_t_LS:i8  =  utils::bool_slice_to_i8(&bits[192..200]);
					let wn_LSF:u8      =  utils::bool_slice_to_u8(&bits[200..208]);  
					let delta_t_LSF:i8 =  utils::bool_slice_to_i8(&bits[208..216]);

					Subframe4::Page18{ alpha0, alpha1, alpha2, alpha3, beta0, beta1, beta2, beta3, a1, a0, t_ot, wn_t, delta_t_LS, wn_LSF, delta_t_LSF }
				},
				62 => {
					let mut antispoof_and_config:[u8; 32] = [0; 32];
					for i in 0..32 {
						antispoof_and_config[i] = utils::bool_slice_to_u8(&bits[(56+(i*4))..(60+(i*4))]);
					}
					let mut sv_health:[u8; 8] = [0; 8];
					for i in 0..8 {
						sv_health[i] = utils::bool_slice_to_u8(&bits[(186+(i*6))..(192+(i*6))])
					}
					Subframe4::Page25{ antispoof_and_config, sv_health }
				},
				_ => Subframe4::Reserved,
			};
			Ok(Subframe::Subframe4{ common, data_id, sv_id, page })
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
				_ => return Err(DigSigProcErr::InvalidTelemetryData("Page number other than 1 through 25")),
			};
			Ok(Subframe::Subframe5{ common, data_id, sv_id, page })
		},
		_ => Err(DigSigProcErr::InvalidTelemetryData("Subframe number other than 1 through 5")),
	}
}