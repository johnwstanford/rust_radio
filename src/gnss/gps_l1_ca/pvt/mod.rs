
extern crate nalgebra as na;
extern crate serde;

use self::serde::{Serialize, Deserialize};
use self::na::base::{Matrix3, Matrix3x1, DMatrix, Vector3, Vector4, DVector, U3, U1};

use ::utils::kinematics;

pub const C:f64 = 2.99792458e8;					 // [m/s] speed of light

const MAX_ITER:usize = 10;
const SV_COUNT_THRESHOLD:usize = 5;

pub mod ephemeris;
pub mod ionosphere;

#[derive(Debug, Serialize, Deserialize)]
pub struct GnssFix {
	pub pos_ecef:(f64, f64, f64),
	pub residual_norm:f64,
	pub residuals: Vec<f64>,
	pub sv_count:usize,
	pub current_rx_time: f64,
	pub obs_this_soln:Vec<Observation>,
}

#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub struct Observation {
	pub sv_tow_sec: f64,
	pub pseudorange_m: f64,
	pub pos_ecef: (f64, f64, f64),
	pub sv_clock: f64,
	pub t_gd: f64,
	pub carrier_freq_hz: f64,
}

impl Observation {
	
	pub fn az_el_from(&self, obs_ecef:(f64, f64, f64)) -> (f64, f64) {
		let po_e = Matrix3x1::from_row_slice_generic(U3, U1, &[obs_ecef.0, obs_ecef.1, obs_ecef.2]);
		let ps_e = Matrix3x1::from_row_slice_generic(U3, U1, &[self.pos_ecef.0,  self.pos_ecef.1,  self.pos_ecef.2 ]);

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

		(az_radians, el_radians)
	}

}

pub fn solve_position_and_time(obs_this_soln:Vec<Observation>, x0:Vector4<f64>, current_rx_time:f64, opt_iono:Option<ionosphere::Model>) -> Result<(GnssFix, Vector4<f64>), &'static str> {
	// TODO: make other time corrections (ionosphere, etc) 

	if obs_this_soln.len() >= SV_COUNT_THRESHOLD {
		let n = obs_this_soln.len();

		let mut x = x0.clone();
		let mut v = DVector::from_element(n, 0.0);

		// Try to solve for position
		for _ in 0..MAX_ITER {

			let mut h = DMatrix::from_element(n, 4, 0.0);

			let p_ob_e = Vector3::new(x[0], x[1], x[2]);
			for (i, ob) in obs_this_soln.iter().enumerate() {
				let p_sv_e = Vector3::new(ob.pos_ecef.0, ob.pos_ecef.1, ob.pos_ecef.2);
				let p_r_e = p_sv_e - p_ob_e;
				let p_r_mag:f64 = p_r_e.norm();
				let p_r_e_norm  = p_r_e / p_r_mag;

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
						let fix = GnssFix{pos_ecef:(x[0], x[1], x[2]), 
							residual_norm:v.norm(), residuals, 
							sv_count:n, current_rx_time, obs_this_soln };
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


