
pub const WGS84_SEMI_MAJOR_AXIS_METERS:f64 = 6378137.0;
pub const WGS84_SEMI_MINOR_AXIS_METERS:f64 = 6356752.314245;

#[derive(Debug)]
pub struct PositionWGS84 {
	pub latitude:f64,
	pub longitude:f64,
	pub height_above_ellipsoid:f64,
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