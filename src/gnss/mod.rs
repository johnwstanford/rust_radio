
/// This module contains functionality related to acquiring GNSS signals common to all systems
pub mod acquisition;

/// This module contains functionality related to tracking signals after acquisition
pub mod tracking;

pub mod telemetry_decode;

pub mod constants;

pub mod gps {

	pub mod l1_ca_signal {

		extern crate rustfft;
		
		use self::rustfft::num_complex::Complex;
		use gnss::constants::gps;
		
	    pub fn prn_complex_sampled(prn:usize, fs:f64) -> Vec<Complex<f64>> {
	    	let samples_per_code:usize = (fs / 1000.0) as usize;
	    	let ts:f64 = 1.0 / fs;

	    	let code = prn_complex(prn);

			(0..samples_per_code).map(|i| {
				let code_value_idx:usize = ((ts * ((i+1) as f64)) / gps::SEC_PER_CHIP) as usize;
				if code_value_idx >= gps::CODE_LENGTH { code[gps::CODE_LENGTH-1] } else { code[code_value_idx] }
			}).collect::<Vec<Complex<f64>>>()
	    }

	    pub fn prn_int_sampled(prn:usize, fs:f64) -> Vec<i8> {
	    	let samples_per_code:usize = (fs / 1000.0) as usize;
	    	let ts:f64 = 1.0 / fs;

	    	let code = gps::prn_int(prn);

			(0..samples_per_code).map(|i| {
				let code_value_idx:usize = ((ts * ((i+1) as f64)) / gps::SEC_PER_CHIP) as usize;
				if code_value_idx >= gps::CODE_LENGTH { code[gps::CODE_LENGTH-1] } else { code[code_value_idx] }
			}).collect::<Vec<i8>>()
	    }

	    pub fn prn_complex(prn:usize) -> Vec<Complex<f64>> {
	    	gps::prn_int(prn).iter().map(|x| Complex{ re: *x as f64, im: 0.0 } ).collect::<Vec<Complex<f64>>>()
	    }

	}

}