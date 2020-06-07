
use ::serde::{Serialize, Deserialize};

use crate::utils::bools_to_int;
use crate::DigSigProcErr;

#[derive(Debug, Serialize, Deserialize)]
pub struct Body {
	t_oe:     u32, om_0n: f64, i_0n:  f64, d_om_dot: f64,
	i_0n_dot: f64, cis_n: f64, cic_n: f64, crs_n:    f64,
	crc_n:    f64, cus_n: f64, cuc_n: f64
}

impl Body {
	
	pub fn new(bits:&[bool]) -> Result<Self, DigSigProcErr> {
		if bits.len() == 238 {
			// Scale factors given in IS-GPS-200K, Table 30-I
			let t_oe     = (bools_to_int::to_u16(&bits[  0.. 11])? as u32) * 300u32;
			let om_0n    = (bools_to_int::to_i64(&bits[ 11.. 44])? as f64) * 2.0_f64.powi(-32);		// Longitude of ascending node of orbit plane at weekly epoch [semicircles]
			let i_0n     = (bools_to_int::to_i64(&bits[ 44.. 77])? as f64) * 2.0_f64.powi(-32);		// Inclination angle at reference time [semicircles]
			let d_om_dot = (bools_to_int::to_i32(&bits[ 77.. 94])? as f64) * 2.0_f64.powi(-44);		// Rate of right ascension difference from reference value of -2.6e-9 [semicircles/sec]
			let i_0n_dot = (bools_to_int::to_i16(&bits[ 94..109])? as f64) * 2.0_f64.powi(-44);		// Rate of inclination angle [semicircles/sec]
			let cis_n    = (bools_to_int::to_i16(&bits[109..125])? as f64) * 2.0_f64.powi(-30);		// Amplitude of the sine harmonic correction term to the angle of inclination [radians]
			let cic_n    = (bools_to_int::to_u16(&bits[125..141])? as f64) * 2.0_f64.powi(-30);		// Amplitude of the cosine harmonic correction term to the angle of inclination [radians]
			let crs_n    = (bools_to_int::to_u32(&bits[141..165])? as f64) * 2.0_f64.powi(-8);		// Amplitude of the sine correction term to the orbit radius [meters]
			let crc_n    = (bools_to_int::to_u32(&bits[165..189])? as f64) * 2.0_f64.powi(-8);		// Amplitude of the cosine correction term to the orbit radius [meters]
			let cus_n    = (bools_to_int::to_u32(&bits[189..210])? as f64) * 2.0_f64.powi(-30);		// Amplitude of the sine harmonic correction term to the argument of latitude [radians]
			let cuc_n    = (bools_to_int::to_u32(&bits[210..231])? as f64) * 2.0_f64.powi(-30);		// Amplitude of the cosine harmonic correction term to the argument of latitude [radians]
			// 7 reserved bits

			Ok(Self{ t_oe, om_0n, i_0n, d_om_dot, i_0n_dot, cis_n, cic_n, crs_n, crc_n, cus_n, cuc_n })

		} else {
			Err(DigSigProcErr::InvalidTelemetryData("Expected a bool slice of length 238 in type11::Body::new"))
		}
	}

}