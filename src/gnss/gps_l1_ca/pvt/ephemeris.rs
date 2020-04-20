
extern crate serde;

use std::f64::consts;

use self::serde::{Serialize, Deserialize};

pub const MU:f64 = 3.986005e14;              // [m^3/s^2] WGS-84 value of the earth's gravitational constant
pub const F:f64 = -4.442807633e-10;			 // [sec/root-meter]

// NOTE: IS-GPS-200K calls this OMEGA_DOT_E but omega is more commonly used for angular velocity, so omega_dot would be angular
// acceleration, so I changed the name to match the more common convention
pub const OMEGA_E:f64 = 7.2921151467e-5;     // [rad/s] WGS-84 value of the earth's rotation rate

#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
pub struct Ephemeris {
	pub week_number:u16, pub t_gd:f64,	  pub aodo: u8,    pub fit_interval:bool,
	pub t_oc: f64,       pub a_f0: f64,   pub a_f1: f64,   pub a_f2: f64,
	pub t_oe: f64,       pub sqrt_a: f64, pub dn: f64,     pub m0: f64,
	pub e: f64,          pub omega: f64,  pub omega0: f64, pub omega_dot: f64,
	pub cus: f64,        pub cuc: f64,    pub crs: f64,    pub crc: f64,
	pub cis: f64,        pub cic: f64,    pub i0: f64,     pub idot: f64,
	pub iodc: u16,
}

impl Ephemeris {

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
	    let omega_k:f64 = (self.omega0 * consts::PI) + ((self.omega_dot * consts::PI) - OMEGA_E)*tk - OMEGA_E*self.t_oe;

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
