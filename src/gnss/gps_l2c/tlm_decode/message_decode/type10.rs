
use ::serde::{Serialize, Deserialize};

use crate::utils::bools_to_int;
use crate::DigSigProcErr;

#[derive(Debug, Serialize, Deserialize)]
pub struct Body {
	week_num: u16,
	l1_health: bool, l2_health: bool, l5_health: bool,
	t_op:  u32, ura_ed: i8,  t_oe:     u32, d_a:  f64,
	a_dot: f64, d_n0:   f64, d_n0_dot: f64, m0_n: f64,
	e_n:   f64, om_n:   f64,
	integrity_status_flag: bool, l2c_phasing: bool
}

impl Body {
	
	pub fn new(bits:&[bool]) -> Result<Self, DigSigProcErr> {
		if bits.len() == 238 {
			// Descriptions and scale factors given in IS-GPS-200K, Table 30-I

			let week_num  = bools_to_int::to_u16(&bits[0..13])?;

			// false indicates "Signal OK" and true indicates "Signal bad or unavailable" (IS-GPS-200K, section 30.3.3.1.1.2)
			let l1_health = bits[13];
			let l2_health = bits[14];
			let l5_health = bits[15];

			let t_op     = (bools_to_int::to_u16(&bits[ 16.. 27])? as u32) * 300u32;			// CEI data sequence propagation time of week
			let ura_ed   =  bools_to_int::to_i8( &bits[ 27.. 32])?;								// Elevation-dependent component of User Range Accuracy (IS-GPS-200K, section 30.3.3.1.1.4)
			let t_oe     = (bools_to_int::to_u16(&bits[ 32.. 43])? as u32) * 300u32;			// Ephemeris data reference time of week
			let d_a      = (bools_to_int::to_i32(&bits[ 43.. 69])? as f64) * 2.0_f64.powi(-9);	// Difference in semi-major axis from reference value of 26,559,710 [meters]
			let a_dot    = (bools_to_int::to_i32(&bits[ 69.. 94])? as f64) * 2.0_f64.powi(-21);	// Rate of change of semi-major axis [meters/sec]
			let d_n0     = (bools_to_int::to_i32(&bits[ 94..111])? as f64) * 2.0_f64.powi(-44); // Mean motion difference from computed value at reference time [semicircles/sec]
			let d_n0_dot = (bools_to_int::to_i32(&bits[111..134])? as f64) * 2.0_f64.powi(-57); // Rate of change of mean motion difference [semicircles/sec^2]
			let m0_n     = (bools_to_int::to_i64(&bits[134..167])? as f64) * 2.0_f64.powi(-32); // Mean anomaly as reference time [semicircles]
			let e_n      = (bools_to_int::to_u64(&bits[167..200])? as f64) * 2.0_f64.powi(-34); // Eccentricity
			let om_n     = (bools_to_int::to_i64(&bits[200..233])? as f64) * 2.0_f64.powi(-32); // Argument of perigee [semicircles]

			let integrity_status_flag = bits[233];	// true indicates "enhanced level of integrity assurance" (IS-GPS-200K, section 30.3.3.1.1)
			let l2c_phasing           = bits[234];	// true indicated that L2C and L2P(Y) are in-phase; false indicates L2C leading L2P(Y) by 90 degrees (IS-GPS-200K, section 3.3.1.5.1)
			// 3 bits reserved

			Ok(Self{ week_num, l1_health, l2_health, l5_health, t_op, ura_ed, t_oe, d_a, a_dot, d_n0, d_n0_dot, m0_n, e_n, om_n, integrity_status_flag, l2c_phasing })

		} else {
			Err(DigSigProcErr::InvalidTelemetryData("Expected a bool slice of length 238 in type10::Body::new"))
		}
	}

}