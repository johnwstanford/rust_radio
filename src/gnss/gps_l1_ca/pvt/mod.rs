
use serde::{Serialize, Deserialize};
use nalgebra::base::{Matrix3, DMatrix, Vector3, Vector4, DVector};

use crate::utils::kinematics;

pub const C:f64 = 2.99792458e8;					 // [m/s] speed of light

const MAX_ITER:usize = 10;
const SV_COUNT_THRESHOLD:usize = 5;

pub mod ephemeris;
pub mod ionosphere;

#[derive(Debug, Serialize, Deserialize)]
pub struct GnssFix {
	pub pos_ecef:(f64, f64, f64),
	pub residual_norm:f64,
	pub current_rx_time: f64,
	pub observations:Vec<(Observation, CompletedObservation)>,
}

// This struct is populated by the tracking and telemetry decoding modules and only depends on SV state
#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub struct Observation {
	pub sv_tow_sec: f64,
	pub pseudorange_m: f64,
	pub pos_ecef: (f64, f64, f64),
	pub sv_clock: f64,
	pub t_gd: f64,
	pub carrier_freq_hz: f64,
}

// A CompletedObservation contains data the depends on the observer state in addition to the SV state
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct CompletedObservation {
	residual: f64,
	p_r_mag: f64,
	p_r_e_norm: Vec<f64>,
	az_radians: f64,
	el_radians: f64,
	iono_delay: f64,
}

impl Observation {
	
	pub fn complete(&self, x:Vector4<f64>, opt_iono:Option<ionosphere::Model>) -> CompletedObservation {
		let p_ob_e = Vector3::new(x[0], x[1], x[2]);
		let p_sv_e = Vector3::new(self.pos_ecef.0, self.pos_ecef.1, self.pos_ecef.2);
		
		// Position of the SV relative to the observer
		let p_r_e = p_sv_e - p_ob_e;
		let p_r_mag:f64 = p_r_e.norm();
		let p_r_e_norm  = p_r_e / p_r_mag;

		// Transformation from ECEF to NED
		let obs_wgs84 = kinematics::ecef_to_wgs84(x[0], x[1], x[2]);
		let dcm_ne = match obs_wgs84 {
			kinematics::PositionWGS84 { latitude:phi, longitude:lam, height_above_ellipsoid:_ } => {
				Matrix3::new(-phi.sin()*lam.cos(), -phi.sin()*lam.sin(),  phi.cos(),
					         -lam.sin(),            lam.cos(),            0.0,
					         -phi.cos()*lam.cos(), -phi.cos()*lam.sin(), -phi.sin())
			}
		};

		// Vector from the observer to the SV in the NED frame
		let p_r_n = dcm_ne * p_r_e;

		let r_horizontal:f64 = (p_r_n[(0,0)].powi(2) + p_r_n[(1,0)].powi(2)).sqrt();
		let az_radians:f64 = p_r_n[(1,0)].atan2(p_r_n[(0,0)]);
		let el_radians:f64 = (-p_r_n[(2,0)]).atan2(r_horizontal);

		// Compute ionospheric delay; recorded for testing, but not applied to the pseudorange yet
		let iono_delay:f64 = match opt_iono {
			Some(iono) => iono.delay(az_radians, el_radians, obs_wgs84.latitude, obs_wgs84.longitude, self.sv_tow_sec),
			None => 0.0
		};

		let residual = self.pseudorange_m - p_r_mag - x[3];
		let p_r_e_norm_vec:Vec<f64> = (0..3).map(|j| p_r_e_norm[j] ).collect();
		CompletedObservation{ residual, p_r_mag, p_r_e_norm: p_r_e_norm_vec, az_radians, el_radians, iono_delay }
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

			for (i, ob) in obs_this_soln.iter().map(|obs| obs.complete(x, opt_iono)).enumerate() {

				v[i] = ob.residual;
				for j in 0..3 { h[(i,j)] = -ob.p_r_e_norm[j]; }
				h[(i,3)] = 1.0;
			
			}


			if let Some(q) = (h.tr_mul(&h)).try_inverse() {
				let dx = q * h.tr_mul(&v);

				x = x + dx.clone();

				if dx.norm() < 1.0e-4 { 

					// The iterative least squares method has converged
					if x.iter().chain(v.iter()).all(|a| a.is_finite()) {
						// Return the fix regardless of the residual norm and let the calling scope determine whether it's good enough
						let observations:Vec<(Observation, CompletedObservation)> = obs_this_soln.iter().map(|obs| (*obs, obs.complete(x, opt_iono))).collect();
						let fix = GnssFix{pos_ecef:(x[0], x[1], x[2]), residual_norm:v.norm(), current_rx_time, observations };
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


