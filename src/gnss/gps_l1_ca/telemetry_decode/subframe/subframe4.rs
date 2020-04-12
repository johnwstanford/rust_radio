
extern crate serde;

use self::serde::{Serialize, Deserialize};
use ::DigSigProcErr;
use ::utils;

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub struct Body {
	pub data_id:u8, 
	pub sv_id:u8, 
	pub page:Page
}

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub enum Page {
	AlmanacData{e:f64, t_oa:u32, delta_i:f64, omega_dot:f64, sv_health:u8, sqrt_a:f64, omega0:f64, omega:f64, m0:f64, af0:f64, af1:f64},
	NavigationMessageCorrectionTable{availability:u8, erd:[u8; 30]},
	SpecialMessages([u8; 22]),
	Page18{ alpha0:f64, alpha1:f64, alpha2:f64, alpha3:f64, beta0:f64, beta1:f64, beta2:f64, beta3:f64, a1:f64, a0:f64, t_ot:u32, wn_t:u8, delta_t_LS:i8, wn_LSF:u8, delta_t_LSF:i8 },
	Page25{ antispoof_and_config:[u8; 32], sv_health:[u8; 8] },
	Reserved,
}

impl Body {

	pub fn new(bits:&[bool; 240]) -> Result<Body, DigSigProcErr> {
		let data_id:u8 = utils::bool_slice_to_u8(&bits[48..50]);
		let sv_id:u8   = utils::bool_slice_to_u8(&bits[50..56]);
		let page:Page = match sv_id {
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
				Page::AlmanacData{e, t_oa, delta_i, omega_dot, sv_health, sqrt_a, omega0, omega, m0, af0, af1}
			},
			52 => {
				let availability:u8 = utils::bool_slice_to_u8(&bits[56..58]);
				let mut erd:[u8; 30] = [0; 30];
				for i in 0..30 {
					erd[i] = utils::bool_slice_to_u8(&bits[(58+(i*6))..(64+(i*6))]);
				}
				Page::NavigationMessageCorrectionTable{availability, erd}
			},
			55 => {
				let mut message:[u8; 22] = [0; 22];
				for i in 0..22 {
					message[i] = utils::bool_slice_to_u8(&bits[(56+(i*8))..(64+(i*8))]);
				}
				Page::SpecialMessages(message)
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

				Page::Page18{ alpha0, alpha1, alpha2, alpha3, beta0, beta1, beta2, beta3, a1, a0, t_ot, wn_t, delta_t_LS, wn_LSF, delta_t_LSF }
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
				Page::Page25{ antispoof_and_config, sv_health }
			},
			_ => Page::Reserved,
		};
		
		Ok(Body{ data_id, sv_id, page })

	}

}