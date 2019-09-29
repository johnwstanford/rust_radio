
extern crate serde;

use self::serde::{Serialize, Deserialize};

use ::gnss::telemetry_decode::gps::l1_ca_subframe;

type SF = l1_ca_subframe::Subframe;

pub const MU:f64 = 3.986005e14;                  // [m^3/s^2] WGS-84 value of the earth's gravitational constant
pub const OMEGA_DOT_E:f64 = 7.2921151467e-5;     // [rad/s] WGS-84 value of the earth's rotation rate
pub const C:f64 = 2.99792458e8;					 // [m/s] speed of light
pub const F:f64 = -4.442807633e-10;				 // [sec/root-meter]

use std::f64::consts;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SatellitePosition {
	pub sv_ecef_position:(f64, f64, f64),
	pub gps_system_time:f64,
	pub receiver_code_phase:usize,
}

pub fn get_ecef(sf1:SF, sf2:SF, sf3:SF) -> Option<SatellitePosition> { match (sf1, sf2, sf3) {
	(SF::Subframe1{common:common1, week_number:_, code_on_l2:_, ura_index:_, sv_health:_, iodc, t_gd:_, t_oc, a_f2, a_f1, a_f0}, 
	 SF::Subframe2{common:_, iode:iode2, crs, dn, m0, cuc, e, cus, sqrt_a, t_oe, fit_interval:_, aodo:_ }, 
	 SF::Subframe3{common:_, cic, omega0, cis, i0, crc, omega, omega_dot, iode:iode3, idot}) => {
		// TODO: make other time corrections (ionosphere, etc) 
		// TODO: account for GPS week rollover possibility
		// TODO: consider returning a Result where the Err describes the reason for not producing a position
		if (iodc % 256) != (iode2 as u16) { return None; }
		if iode2 != iode3                 { return None; }

		// Find GPS system time at the time of transmission of the beginning of subframe 1
		// Use the algorithm described in IS-GPS-200H, 20.3.3.3.3.1
		// t_sv = sf1['common']['time_of_week_truncated'] * 4 * 1.5   # [sec]
		let t_sv:f64 = (common1.time_of_week_truncated as f64) * 4.0 * 1.5;
		let dt_sv:f64 = a_f0 + a_f1*(t_sv - (t_oc as f64)) + a_f2*(t_sv - (t_oc as f64)).powi(2);
		let t:f64 = t_sv - dt_sv;

		// Note: this is the time without the relativistic correction because we need the eccentric anomaly, which
		// we haven't calculated yet, but this is a good approximation.  Also, we should use t in these equations instead
		// of t_sv but for this purpose, t_sv is a good approximation to t.  The GPS ICD also mentions this issue and
		// recommends this approximation.

		// Note: Code commented above each line is Python code used to rapid-prototype this algorithm

		// Find ECEF coordinates using the algorithm described in IS-GPS-200H, Table 20-IV
	    // A = pow(sf2['sqrt_a'], 2)      # [m]
		let a:f64 = sqrt_a.powi(2);	
	    
	    // n0 = sqrt(mu / pow(A, 3))      # [rad/s]
	    let n0:f64 = (MU / a.powi(3)).sqrt();
	    
	    // tk = t - sf2['t_oe']           # [sec]
	    let tk:f64 = t - t_oe;

	    // n = n0 + (sf2['dn'] * pi)      # [rad/s]
	    let n:f64 = n0 + (dn * consts::PI);

	    // Mean anomaly
	    // Mk = (sf2['m0'] * pi) + n*tk   # [rad]
	    let mk:f64 = (m0 * consts::PI) + n*tk;

	    let mut ek:f64 = mk;
	    for _ in 0..10 {
	    	// Iteratively find eccentric anomaly using the Newton-Raphson method
	    	// TODO: make the number of iterations configurable and/or based on a tolerance, but 10 is probably good for now
	        // Ek = Ek - (Ek - e*sin(Ek) - Mk)/(1.0 - e*cos(Ek))
	        ek = ek - (ek - e*ek.sin() - mk)/(1.0 - e*ek.cos());
	    }

	    // nu_k = arctan2(sqrt(1.0 - e*e)*sin(Ek) / (1.0 - e*cos(Ek)), (cos(Ek) - e)/(1.0 - e*cos(Ek)))
	    let nu_k:f64 = {
	    	let y:f64 = ((1.0 - e.powi(2)).sqrt() * ek.sin()) / (1.0 - (e*ek.cos()));
	    	let x:f64 = (ek.cos() - e) / (1.0 - (e*ek.cos()));
	    	y.atan2(x)
	    };

	    // Ek = arccos((e + cos(nu_k))/(1.0 + e*cos(nu_k)))    # [radians]
	    // Phi_k = nu_k + (sf3['omega'] * pi)    # [radians]
	    let phi_k:f64 = nu_k + (omega * consts::PI);

	    // du_k = sf2['cus']*sin(2*Phi_k) + sf2['cuc']*cos(2*Phi_k)
	    let du_k:f64 = cus*(2.0*phi_k).sin() + cuc*(2.0*phi_k).cos();

	    // dr_k = sf2['crs']*sin(2*Phi_k) + sf3['crc']*cos(2*Phi_k)
	    let dr_k:f64 = crs*(2.0*phi_k).sin() + crc*(2.0*phi_k).cos();
		
	    // di_k = sf3['cis']*sin(2*Phi_k) + sf3['cic']*cos(2*Phi_k)
	    let di_k:f64 = cis*(2.0*phi_k).sin() + cic*(2.0*phi_k).cos();

	    // u_k = Phi_k + du_k                                     # [radians]
 		let u_k:f64 = phi_k + du_k;

	    // r_k = A*(1.0 - e*cos(Ek)) + dr_k                       # [m]
	    let r_k:f64 = a*(1.0 - e*ek.cos()) + dr_k;

	    // i_k = (sf3['i0'] * pi) + di_k + (sf3['idot'] * pi)*tk  # [radians]
	    let i_k:f64 = (i0 * consts::PI) + di_k + (idot * consts::PI)*tk;

	    // x_kp = r_k*cos(u_k)   # [m]
		let x_kp:f64 = r_k * u_k.cos();

	    // y_kp = r_k*sin(u_k)   # [m]
	    let y_kp:f64 = r_k * u_k.sin();

	    // Omega_k = (sf3['omega0'] * pi) + ((sf3['omega_dot'] * pi) - omega_dot_e)*tk - omega_dot_e*sf2['t_oe']  # [radians]
	    let omega_k:f64 = (omega0 * consts::PI) + ((omega_dot * consts::PI) - OMEGA_DOT_E)*tk - OMEGA_DOT_E*t_oe;

	    // x_k = x_kp*cos(Omega_k) - y_kp*cos(i_k)*sin(Omega_k)
	    let x_k:f64 = (x_kp * omega_k.cos()) - (y_kp * i_k.cos() * omega_k.sin());

	    // y_k = x_kp*sin(Omega_k) + y_kp*cos(i_k)*cos(Omega_k)
	    let y_k:f64 = (x_kp * omega_k.sin()) + (y_kp * i_k.cos() * omega_k.cos());

	    // z_k = y_kp*sin(i_k)
	    let z_k:f64 = y_kp * i_k.sin();

	    // Relativistic correction to transmission time
		let dt_r:f64 = F * e * sqrt_a * ek.sin();

		Some( SatellitePosition{ sv_ecef_position:(x_k, y_k, z_k), gps_system_time:t+dt_r,  receiver_code_phase: common1.start_sample_idx} )
	},
	(_, _, _) => None
}}