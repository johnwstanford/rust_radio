
extern crate nalgebra as na;

use std::f64::consts;

#[derive(Debug, Clone, Copy)]
pub struct Model {
	pub alpha0:f64, pub alpha1:f64, pub alpha2:f64, pub alpha3:f64, 
	pub beta0:f64,  pub beta1:f64,  pub beta2:f64,  pub beta3:f64
}

impl Model {
	
	pub fn delay(&self, az_radians:f64, el_radians:f64, latitude_radians:f64, longitude_radians:f64, t:f64) -> f64 {

		let el_semicircles:f64 = el_radians / consts::PI;

		let mut phi_u:f64 = latitude_radians  / consts::PI;
		let mut lam_u:f64 = longitude_radians / consts::PI;
		if phi_u > 0.5 {
			phi_u = 1.0 - phi_u;
			lam_u -= 1.0;
		}
		if phi_u < -0.5 {
			phi_u = -1.0 - phi_u;
			lam_u -= 1.0;
		}
		if lam_u > 1.0 {
			lam_u -= 2.0;
		}
		if lam_u < -1.0 {
			lam_u += 2.0;
		}

		let psi:f64 = (0.0137/(el_semicircles + 0.11)) - 0.022;		// [semicircles]
		let phi_i:f64 = {										    // [semicircles]
			let ans = phi_u + (psi*az_radians.cos());
			if      ans < -0.416 { -0.416 }
			else if ans >  0.416 {  0.416 }
			else                 {  ans   }
		};
		let lam_i:f64 = lam_u + ((psi*az_radians.sin()) / (phi_i * consts::PI).cos()); 		// [semicircles]

		let phi_m:f64 = phi_i + 0.064*((lam_i * consts::PI) - 1.617).cos();		// [semicircles]
		let t_lcl:f64 = (4.32e4 * lam_i) + t;									// [sec]

		let f_iono:f64 = 1.0 + 16.0*(0.53 - el_semicircles).powi(3);			// []
		let x:f64 = {															// [radians]
			let per_p:f64 = self.beta0 + self.beta1*phi_m + self.beta2*phi_m.powi(2) + self.beta3.powi(3);
			let per:f64 = if per_p < 72000.0 { 72000.0 } else { per_p };
			(2.0 * consts::PI*(t_lcl - 50400.0)) / per
		};
		let amp:f64 = {
			let ans:f64 = self.alpha0 + self.alpha1*phi_m + self.alpha2*phi_m.powi(2) + self.alpha3.powi(3);
			if ans < 0.0 { 0.0 } else { ans }
		};

		if x.abs() < 1.57 {
			f_iono * (5.0e-9 + amp*(1.0 - (x.powi(2)/2.0) + (x.powi(4)/24.0) ))
		} else {
			f_iono * 5.0e-9
		}
	}

}
