
use ::serde::{Serialize, Deserialize};

use crate::utils::bools_to_int;
use crate::DigSigProcErr;

#[derive(Debug, Serialize, Deserialize)]
pub struct Body {
	t_op: u32,
	ura_ned0: u8, ura_ned1: u8, ura_ned2: u8,
	t_oc: u32, 
	a_f0n: f32, a_f1n: f32, a_f2n: f32,
	t_gd: f32,
	isc_l1ca: f32, isc_l2c: f32, isc_l5i5: f32, isc_l5q5: f32,
	alpha0: f32, alpha1: f32, alpha2: f32, alpha3: f32,
	beta0:  f32, beta1:  f32, beta2:  f32, beta3:  f32,
	wn_op: u8
}

impl Body {
	
	pub fn new(bits:&[bool]) -> Result<Self, DigSigProcErr> {
		if bits.len() == 238 {
			// Table 30-III (Clock Correction and Accuracy Parameters) from IS-GPS-200K
			let t_op     = (bools_to_int::to_u16(&bits[  0.. 11])? as u32) * 300u32;			// CEI data sequence propagation time of week
			let ura_ned0 =  bools_to_int::to_u8( &bits[ 11.. 16])?;								// NED accuracy index
			let ura_ned1 =  bools_to_int::to_u8( &bits[ 16.. 19])?;								// NED accuracy change index
			let ura_ned2 =  bools_to_int::to_u8( &bits[ 19.. 22])?;								// NED accuracy change rate index
			let t_oc     = (bools_to_int::to_u16(&bits[ 22.. 33])? as u32) * 300u32;			// Clock data reference time of week
			let a_f0n    = (bools_to_int::to_i32(&bits[ 33.. 59])? as f32) * 2.0_f32.powi(-35);	// SV clock bias correction coefficient [sec]
			let a_f1n    = (bools_to_int::to_i32(&bits[ 59.. 79])? as f32) * 2.0_f32.powi(-48);	// SV clock drift correction coefficient [sec/sec]
			let a_f2n    = (bools_to_int::to_i16(&bits[ 79.. 89])? as f32) * 2.0_f32.powi(-60);	// SV clock drift rate correction coefficient [sec/sec^2]
			
			// Table 30-IV (Group Delay Differential Parameters) from IS-GPS-200K
			let t_gd     = (bools_to_int::to_i16(&bits[ 89..102])? as f32) * 2.0_f32.powi(-35);	// [sec]
			let isc_l1ca = (bools_to_int::to_i16(&bits[102..115])? as f32) * 2.0_f32.powi(-35); // [sec]
			let isc_l2c  = (bools_to_int::to_i16(&bits[115..128])? as f32) * 2.0_f32.powi(-35); // [sec]
			let isc_l5i5 = (bools_to_int::to_i16(&bits[128..141])? as f32) * 2.0_f32.powi(-35); // [sec]
			let isc_l5q5 = (bools_to_int::to_i16(&bits[141..154])? as f32) * 2.0_f32.powi(-35); // [sec]

			// Table 20-X (Ionospheric Parameters) from IS-GPS-200K
			let alpha0   = (bools_to_int::to_i8( &bits[154..162])? as f32) * 2.0_f32.powi(-30);	// [sec]
			let alpha1   = (bools_to_int::to_i8( &bits[162..170])? as f32) * 2.0_f32.powi(-27);	// [sec/semicircle]
			let alpha2   = (bools_to_int::to_i8( &bits[170..178])? as f32) * 2.0_f32.powi(-24); // [sec/semicircle^2]
			let alpha3   = (bools_to_int::to_i8( &bits[178..186])? as f32) * 2.0_f32.powi(-24); // [sec/semicircle^3]
			let beta0    = (bools_to_int::to_i8( &bits[186..194])? as f32) * 2.0_f32.powi( 11); // [sec]
			let beta1    = (bools_to_int::to_i8( &bits[194..202])? as f32) * 2.0_f32.powi( 14);	// [sec/semicircle]
			let beta2    = (bools_to_int::to_i8( &bits[202..210])? as f32) * 2.0_f32.powi( 16); // [sec/semicircle^2]
			let beta3    = (bools_to_int::to_i8( &bits[210..218])? as f32) * 2.0_f32.powi( 16); // [sec/semicircle^3]

			// CEI Data Sequence Propagation Week Number, section 30.3.3.3.1.3 of IS-GPS-200K
			let wn_op    =  bools_to_int::to_u8( &bits[218..226])?;

			// 12 reserved bits

			Ok(Self{ t_op, ura_ned0, ura_ned1, ura_ned2, t_oc, a_f0n, a_f1n, a_f2n, t_gd, isc_l1ca, isc_l2c, isc_l5i5, isc_l5q5,
					 alpha0, alpha1, alpha2, alpha3, beta0, beta1, beta2, beta3, wn_op })

		} else {
			Err(DigSigProcErr::InvalidTelemetryData("Expected a bool slice of length 238 in type30::Body::new"))
		}
	}

}