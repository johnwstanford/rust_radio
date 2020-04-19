
extern crate nalgebra as na;

use self::na::base::{Matrix3, Vector3};

pub const WGS84_SEMI_MAJOR_AXIS_METERS:f64 = 6378137.0;
pub const WGS84_SEMI_MINOR_AXIS_METERS:f64 = 6356752.314245;
pub const OMEGA_E:f64 = 7.2921151467e-5;     // [rad/s] WGS-84 value of the earth's rotation rate
pub const C:f64 = 2.99792458e8;					 // [m/s] speed of light

use std::f64::consts;

#[derive(Debug)]
pub struct PositionWGS84 {
	pub latitude:f64,
	pub longitude:f64,
	pub height_above_ellipsoid:f64,
}

pub fn dcm_we(lat:f64, lon:f64) -> Matrix3<f64> {
	Matrix3::new(-lon.sin(),            lon.cos(),           0.0,
		         -lat.sin()*lon.cos(), -lat.sin()*lon.sin(), lat.cos(),
		          lat.cos()*lon.cos(),  lat.cos()*lon.sin(), lat.sin())
}

pub fn az_el(lat:f64, lon:f64, h:f64, los_e:Vector3<f64>) -> (f64, f64) {
	let los_enu:Vector3<f64> = dcm_we(lat, lon) * los_e;
	let dot_01:f64 = los_enu[0]*los_enu[0] + los_enu[1]*los_enu[1];
	let mut az:f64 = if dot_01 > 1.0e-12 { los_enu[0].atan2(los_enu[1]) } else { 0.0 };
	if az < 0.0 { az += 2.0*consts::PI; }
	let el:f64 = los_enu[2].asin();
	if h > -WGS84_SEMI_MAJOR_AXIS_METERS { (az,el) } else { (0.0, 0.5*consts::PI) }
}

pub fn dist_with_sagnac_effect(rs:Vector3<f64>, rr:Vector3<f64>) -> (Vector3<f64>, f64) {
	let e = rs - rr;
	let r:f64 = e.norm();
	(e/r, r)
}

pub fn ecef_to_wgs84(e1:f64, e2:f64, e3:f64) -> PositionWGS84 {
	let a_sq:f64 = WGS84_SEMI_MAJOR_AXIS_METERS.powi(2);
	let b_sq:f64 = WGS84_SEMI_MINOR_AXIS_METERS.powi(2);

	let e_sq:f64  = (a_sq - b_sq) / a_sq;
	let ep_sq:f64 = (a_sq - b_sq) / b_sq;

	let p:f64 = (e1*e1 + e2*e2).sqrt();
	let r:f64 = (p*p   + e3*e3).sqrt();

	let beta:f64 = (((WGS84_SEMI_MINOR_AXIS_METERS*e3)/(WGS84_SEMI_MAJOR_AXIS_METERS*p)) * (1.0 + ep_sq*(WGS84_SEMI_MINOR_AXIS_METERS/r))).atan();

	let latitude:f64 = {
		let num:f64 = e3 + (ep_sq * WGS84_SEMI_MINOR_AXIS_METERS * beta.sin().powi(3));
		let denom:f64 = p - (e_sq * WGS84_SEMI_MAJOR_AXIS_METERS * beta.cos().powi(3));
		(num/denom).atan()
	};
	let longitude:f64 = e2.atan2(e1);

	let v = WGS84_SEMI_MAJOR_AXIS_METERS / (1.0 - (e_sq*latitude.sin().powi(2))).sqrt();
	let height_above_ellipsoid = p*latitude.cos() + e3*latitude.sin() - (WGS84_SEMI_MAJOR_AXIS_METERS.powi(2) / v);

	PositionWGS84{ latitude, longitude, height_above_ellipsoid }
}