

pub mod acquisition;
pub mod tracking;
pub mod telemetry_decode;

pub mod gps {

	pub mod l1_ca_signal {

		extern crate rustfft;
		
		use self::rustfft::num_complex::Complex;
		
		const DELAYS:[usize; 51] = [5 /*PRN1*/, 6, 7, 8, 17, 18, 139, 140, 141, 251, 252, 254, 255, 256, 257, 258, 469, 470, 471, 472,
	        473, 474, 509, 512, 513, 514, 515, 516, 859, 860, 861, 862 /*PRN32*/,
	        145 /*PRN120*/, 175, 52, 21, 237, 235, 886, 657, 634, 762,
	        355, 1012, 176, 603, 130, 359, 595, 68, 386 /*PRN138*/];

	    const CODE_LENGTH:usize = 1023;
		const CHIPS_PER_SEC:usize = 1023000;
		const SEC_PER_CHIP:f64 = 1.0 / (CHIPS_PER_SEC as f64);

	    pub fn prn_complex_sampled(prn:usize, fs:f64) -> Vec<Complex<f64>> {
	    	let samples_per_code:usize = (fs / 1000.0) as usize;
	    	let ts:f64 = 1.0 / fs;

	    	let code = prn_complex(prn);

			(0..samples_per_code).map(|i| {
				let code_value_idx:usize = ((ts * ((i+1) as f64)) / SEC_PER_CHIP) as usize;
				if code_value_idx >= CODE_LENGTH { code[CODE_LENGTH-1].clone() } else { code[code_value_idx].clone() }
			}).collect::<Vec<Complex<f64>>>()
	    }

	    pub fn prn_int_sampled(prn:usize, fs:f64) -> Vec<i8> {
	    	let samples_per_code:usize = (fs / 1000.0) as usize;
	    	let ts:f64 = 1.0 / fs;

	    	let code = prn_int(prn);

			(0..samples_per_code).map(|i| {
				let code_value_idx:usize = ((ts * ((i+1) as f64)) / SEC_PER_CHIP) as usize;
				if code_value_idx >= CODE_LENGTH { code[CODE_LENGTH-1].clone() } else { code[code_value_idx].clone() }
			}).collect::<Vec<i8>>()
	    }

	    pub fn prn_complex(prn:usize) -> Vec<Complex<f64>> {
	    	prn_int(prn).iter().map(|x| Complex{ re: *x as f64, im: 0.0 } ).collect::<Vec<Complex<f64>>>()
	    }

	    pub fn prn_int(prn:usize) -> Vec<i8> {

	    	let mut g1_register:[bool; 10] = [true; 10];
	    	let mut g2_register:[bool; 10] = [true; 10];
	    	let mut g1:[bool; CODE_LENGTH] = [true; CODE_LENGTH];
	    	let mut g2:[bool; CODE_LENGTH] = [true; CODE_LENGTH];

	    	let mut feedback1:bool = true;
	    	let mut feedback2:bool = true;

	    	for i in 0..CODE_LENGTH {
	    		g1_register[9] = feedback1;
	    		g2_register[9] = feedback2;

	    		g1[i] = g1_register[0];
	    		g2[i] = g2_register[0];

	    		feedback1 = g1_register[7] ^ g1_register[0];
	    		feedback2 = (vec![8, 7, 4, 2, 1, 0].iter().map(|idx| if g2_register[*idx] {1} else {0}).sum::<u16>() & 0x1) == 0x1;

	    		for j in 0..9 {
	    			g1_register[j] = g1_register[j+1];
	    			g2_register[j] = g2_register[j+1]; 
	    		}
	    	}

	    	let delay_idx:usize = if prn >= 120 { prn - 88 } else { prn - 1 };
	    	let delay:usize = CODE_LENGTH - DELAYS[delay_idx];

	    	(0..CODE_LENGTH).map(|idx| {
	    		let this_delay:usize = delay + idx + 1;
	    		if g1[idx] ^ g2[(this_delay-1) % CODE_LENGTH] {1} else {-1}
	    	}).collect::<Vec<i8>>()

	    }

	}

}