
extern crate nalgebra as na;
extern crate serde;

use self::serde::{Serialize, Deserialize};
use self::na::base::{Matrix3, Matrix3x1, DMatrix, Vector3, Vector4, DVector, U3, U1};

use ::utils::kinematics;
use ::gnss::gps_l1_ca::channel;

pub const MU:f64 = 3.986005e14;                  // [m^3/s^2] WGS-84 value of the earth's gravitational constant
pub const OMEGA_DOT_E:f64 = 7.2921151467e-5;     // [rad/s] WGS-84 value of the earth's rotation rate
pub const C:f64 = 2.99792458e8;					 // [m/s] speed of light
pub const F:f64 = -4.442807633e-10;				 // [sec/root-meter]

const MAX_ITER:usize = 10;
const SV_COUNT_THRESHOLD:usize = 5;

use std::f64::consts;

#[derive(Debug, Serialize, Deserialize)]
pub struct GnssFix {
	pub pos_ecef:(f64, f64, f64),
	pub residual_norm:f64,
	pub residuals: Vec<f64>,
	pub sv_count:usize,
	pub current_rx_time: f64,
	pub obs_this_soln:Vec<channel::track_and_tlm::ChannelObservation>,
}

pub fn solve_position_and_time(obs_this_soln:Vec<channel::track_and_tlm::ChannelObservation>, x0:Vector4<f64>, current_rx_time:f64) -> Result<(GnssFix, Vector4<f64>), &'static str> {
	if obs_this_soln.len() >= SV_COUNT_THRESHOLD {
		let n = obs_this_soln.len();

		let mut x = x0.clone();
		let mut v = DVector::from_element(n, 0.0);

		// Try to solve for position
		for _ in 0..MAX_ITER {
			// Not needed right now, but maybe later
			// let pos_wgs84 = kinematics::ecef_to_wgs84(x[0], x[1], x[2]);

			let mut h = DMatrix::from_element(n, 4, 0.0);

			let p_ob_e = Vector3::new(x[0], x[1], x[2]);
			for (i, ob) in obs_this_soln.iter().enumerate() {
				let p_sv_e = Vector3::new(ob.pos_ecef.0, ob.pos_ecef.1, ob.pos_ecef.2);
				let (p_r_e_norm, p_r_mag) = {
					let p_r_e = p_sv_e - p_ob_e;
					let r:f64 = p_r_e.norm();
					(p_r_e/r, r)
				};

				v[i] = ob.pseudorange_m - p_r_mag - x[3];
				for j in 0..3 { h[(i,j)] = -p_r_e_norm[j]; }
				h[(i,3)] = 1.0;
			}


			if let Some(q) = (h.tr_mul(&h)).try_inverse() {
				let dx = q * h.tr_mul(&v);

				x = x + dx.clone();

				if dx.norm() < 1.0e-4 { 

					// The iterative least squares method has converged
					if x.iter().chain(v.iter()).all(|a| a.is_finite()) {
						// Return the fix regardless of the residual norm and let the calling scope determine whether it's good enough
						let residuals:Vec<f64> = v.iter().map(|x| *x).collect();
						let fix = GnssFix{pos_ecef:(x[0], x[1], x[2]), residual_norm:v.norm(), residuals, sv_count:n, current_rx_time, obs_this_soln };
						return Ok((fix, x))
					}

					return Err("Solution and/or residual is infinite");
				}

			} else { 
				// If we get a non-invertible matrix, just return None
				return Err("Non-invertible matrix");
			}

		}

	}

	Err("Not enough observations")
}

#[derive(Debug)]
pub struct IonosphericModel {
	pub alpha0:f64, pub alpha1:f64, pub alpha2:f64, pub alpha3:f64, 
	pub beta0:f64,  pub beta1:f64,  pub beta2:f64,  pub beta3:f64
}

impl IonosphericModel {
	
	pub fn delay(&self, obs_ecef:(f64, f64, f64), sv_ecef:(f64, f64, f64), t:f64) -> f64 {

		let po_e = Matrix3x1::from_row_slice_generic(U3, U1, &[obs_ecef.0, obs_ecef.1, obs_ecef.2]);
		let ps_e = Matrix3x1::from_row_slice_generic(U3, U1, &[sv_ecef.0,  sv_ecef.1,  sv_ecef.2 ]);

		// Vector from the observer to the SV in the ECEF frame
		let r_e = ps_e - po_e;

		let obs_wgs84 = kinematics::ecef_to_wgs84(obs_ecef.0, obs_ecef.1, obs_ecef.2);
		let dcm_le = match obs_wgs84 {
			kinematics::PositionWGS84 { latitude:phi, longitude:lam, height_above_ellipsoid:_ } => {
				Matrix3::new(-phi.sin()*lam.cos(), -phi.sin()*lam.sin(),  phi.cos(),
					         -lam.sin(),            lam.cos(),            0.0,
					         -phi.cos()*lam.cos(), -phi.cos()*lam.cos(), -phi.sin())
			}
		};

		// Vector from the observer to the SV in the local-level frame
		let r_l = dcm_le * r_e;

		let r_horizontal:f64 = (r_l[(0,0)].powi(2) + r_l[(1,0)].powi(2)).sqrt();
		let az_radians:f64 = r_l[(1,0)].atan2(r_l[(0,0)]);
		let el_radians:f64 = r_l[(2,0)].atan2(r_horizontal);

		// Algorithm from IS-GPS-200H, Figure 20-4
		// Note: the GPS ICD has some units in semicircles and some in radians, but everything passed into trig functions needs to be in radians
		// TODO: this is one of the worst cases of missing units and magic numbers I've ever seen; figure out what everything is and document it
		// let az_semicircles:f64 = az_radians / consts::PI;
		let el_semicircles:f64 = el_radians / consts::PI;

		let mut phi_u:f64 = obs_wgs84.latitude  / consts::PI;
		let mut lam_u:f64 = obs_wgs84.longitude / consts::PI;
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

#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
pub struct CalendarAndEphemeris {
	pub week_number:u16, pub t_gd:f64,	  pub aodo: u8,    pub fit_interval:bool,
	pub t_oc: f64,       pub a_f0: f64,   pub a_f1: f64,   pub a_f2: f64,
	pub t_oe: f64,       pub sqrt_a: f64, pub dn: f64,     pub m0: f64,
	pub e: f64,          pub omega: f64,  pub omega0: f64, pub omega_dot: f64,
	pub cus: f64,        pub cuc: f64,    pub crs: f64,    pub crc: f64,
	pub cis: f64,        pub cic: f64,    pub i0: f64,     pub idot: f64,
	pub iodc: u16,
}

impl CalendarAndEphemeris {

	// Correction factor between the SV clock and GPS system time
	pub fn dt_sv(&self, t:f64) -> f64 { self.a_f0 + self.a_f1*(t - self.t_oc) + self.a_f2*(t - self.t_oc).powi(2) }

	pub fn pos_and_clock(&self, t:f64) -> ((f64, f64, f64), f64) {
		// Note: this is the time without the relativistic correction because we need the eccentric anomaly, which
		// we haven't calculated yet, but this is a good approximation.  Also, we should use t in these equations instead
		// of t_sv but for this purpose, t_sv is a good approximation to t.  The GPS ICD also mentions this issue and
		// recommends this approximation.

		// Note: Code commented above each line is Python code used to rapid-prototype this algorithm

		// Find ECEF coordinates using the algorithm described in IS-GPS-200H, Table 20-IV
	    // A = pow(sf2['sqrt_a'], 2)      # [m]
		let a:f64 = self.sqrt_a.powi(2);	
	    
	    // n0 = sqrt(mu / pow(A, 3))      # [rad/s]
	    let n0:f64 = (MU / a.powi(3)).sqrt();
	    
	    // tk = t - sf2['t_oe']           # [sec]
	    let tk:f64 = t - self.t_oe;

	    // n = n0 + (sf2['dn'] * pi)      # [rad/s]
	    let n:f64 = n0 + (self.dn * consts::PI);

	    // Mean anomaly
	    // Mk = (sf2['m0'] * pi) + n*tk   # [rad]
	    let mk:f64 = (self.m0 * consts::PI) + n*tk;

	    let mut ek:f64 = mk;
	    for _ in 0..10 {
	    	// Iteratively find eccentric anomaly using the Newton-Raphson method
	    	// TODO: make the number of iterations configurable and/or based on a tolerance, but 10 is probably good for now
	        // Ek = Ek - (Ek - e*sin(Ek) - Mk)/(1.0 - e*cos(Ek))
	        ek = ek - (ek - self.e*ek.sin() - mk)/(1.0 - self.e*ek.cos());
	    }

	    // nu_k = arctan2(sqrt(1.0 - e*e)*sin(Ek) / (1.0 - e*cos(Ek)), (cos(Ek) - e)/(1.0 - e*cos(Ek)))
	    let nu_k:f64 = {
	    	let y:f64 = ((1.0 - self.e.powi(2)).sqrt() * ek.sin()) / (1.0 - (self.e*ek.cos()));
	    	let x:f64 = (ek.cos() - self.e) / (1.0 - (self.e*ek.cos()));
	    	y.atan2(x)
	    };

	    // Ek = arccos((e + cos(nu_k))/(1.0 + e*cos(nu_k)))    # [radians]
	    // Phi_k = nu_k + (sf3['omega'] * pi)    # [radians]
	    let phi_k:f64 = nu_k + (self.omega * consts::PI);

	    // du_k = sf2['cus']*sin(2*Phi_k) + sf2['cuc']*cos(2*Phi_k)
	    let du_k:f64 = self.cus*(2.0*phi_k).sin() + self.cuc*(2.0*phi_k).cos();

	    // dr_k = sf2['crs']*sin(2*Phi_k) + sf3['crc']*cos(2*Phi_k)
	    let dr_k:f64 = self.crs*(2.0*phi_k).sin() + self.crc*(2.0*phi_k).cos();
		
	    // di_k = sf3['cis']*sin(2*Phi_k) + sf3['cic']*cos(2*Phi_k)
	    let di_k:f64 = self.cis*(2.0*phi_k).sin() + self.cic*(2.0*phi_k).cos();

	    // u_k = Phi_k + du_k                                     # [radians]
 		let u_k:f64 = phi_k + du_k;

	    // r_k = A*(1.0 - e*cos(Ek)) + dr_k                       # [m]
	    let r_k:f64 = a*(1.0 - self.e*ek.cos()) + dr_k;

	    // i_k = (sf3['i0'] * pi) + di_k + (sf3['idot'] * pi)*tk  # [radians]
	    let i_k:f64 = (self.i0 * consts::PI) + di_k + (self.idot * consts::PI)*tk;

	    // x_kp = r_k*cos(u_k)   # [m]
		let x_kp:f64 = r_k * u_k.cos();

	    // y_kp = r_k*sin(u_k)   # [m]
	    let y_kp:f64 = r_k * u_k.sin();

	    // Omega_k = (sf3['omega0'] * pi) + ((sf3['omega_dot'] * pi) - omega_dot_e)*tk - omega_dot_e*sf2['t_oe']  # [radians]
	    let omega_k:f64 = (self.omega0 * consts::PI) + ((self.omega_dot * consts::PI) - OMEGA_DOT_E)*tk - OMEGA_DOT_E*self.t_oe;

	    // x_k = x_kp*cos(Omega_k) - y_kp*cos(i_k)*sin(Omega_k)
	    let x_k:f64 = (x_kp * omega_k.cos()) - (y_kp * i_k.cos() * omega_k.sin());

	    // y_k = x_kp*sin(Omega_k) + y_kp*cos(i_k)*cos(Omega_k)
	    let y_k:f64 = (x_kp * omega_k.sin()) + (y_kp * i_k.cos() * omega_k.cos());

	    // z_k = y_kp*sin(i_k)
	    let z_k:f64 = y_kp * (i_k.sin());

	    // Relativistic correction to transmission time
		let dt_r:f64 = F * self.e * self.sqrt_a * ek.sin();
		
		((x_k, y_k, z_k), self.a_f0 + (self.a_f1 * tk) + (self.a_f2 * tk.powi(2)) + dt_r)

	} 

}
