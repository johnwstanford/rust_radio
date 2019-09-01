extern crate byteorder;
extern crate clap;
extern crate rust_radio;
extern crate num_complex;

use std::io::Read;
use std::fs::File;

use byteorder::{ByteOrder, LittleEndian};
use clap::{Arg, App};
use rust_radio::DigSigProcErr;
use rust_radio::fourier_analysis::fft;
use num_complex::Complex;

type GetResult = Result<Complex<f64>, DigSigProcErr>;

fn get_complex_i16_le(f:&mut File) ->  GetResult {
	let mut buffer = [0; 4];
	match f.read_exact(&mut buffer) {
		Ok(_)  => {
	        let re_i16 = LittleEndian::read_i16(&buffer[0..2]);
	        let im_i16 = LittleEndian::read_i16(&buffer[2..4]);
			Ok(Complex{re: re_i16 as f64, im: im_i16 as f64})			
		},
		Err(_) => Err(DigSigProcErr::NoSourceData)
	}
}

fn get_complex_i8_le(f:&mut File) -> GetResult {
	let mut buffer = [0; 2];
	match f.read_exact(&mut buffer) {
		Ok(_)  => Ok(Complex{re: (buffer[0] as i8) as f64, im: (buffer[1] as i8) as f64}),
		Err(_) => Err(DigSigProcErr::NoSourceData)
	}
}

fn main() {
	let matches = App::new("Incoherent Amplitude Spectrum")
		.version("0.1.0")
		.author("John Stanford (johnwstanford@gmail.com)")
		.about("Generates an incoherently-integrated amplitude spectrum for a binary file full of IQ samples.")
		.arg(Arg::with_name("filename")
			.short("f").long("filename")
			.help("Input filename")
			.required(true).takes_value(true))
		.arg(Arg::with_name("input_type")
			.short("t").long("type")
			.help("Input type, defaults to i8 (8-bit signed)")
			.takes_value(true)
			.possible_value("i8")
			.possible_value("i16"))
		.arg(Arg::with_name("big_endian")
			.long("big_endian")
			.help("Use big endian decoding (as opposed to little endian by default)"))
		.arg(Arg::with_name("sample_rate_sps")
			.short("s").long("sample_rate_sps")
			.takes_value(true).required(true))
		.arg(Arg::with_name("baseband_hz")
			.short("b").long("baseband_hz")
			.takes_value(true).required(true))
		.arg(Arg::with_name("resolution_hz")
			.short("r").long("resolution_hz")
			.takes_value(true).required(true))
		.arg(Arg::with_name("output_min")
			.long("output_min")
			.takes_value(true)
			.number_of_values(1)
			.allow_hyphen_values(true))
		.arg(Arg::with_name("output_max")
			.long("output_max")
			.takes_value(true)
			.number_of_values(1)
			.allow_hyphen_values(true))
		.arg(Arg::with_name("max_num_ffts")
			.long("max_num_ffts")
			.takes_value(true)
			.number_of_values(1))
		.get_matches();

	let mut f_in = File::open(matches.value_of("filename").unwrap()).expect("Unable to open source file");
	let f_get:fn(&mut File) -> GetResult = match (matches.occurrences_of("big_endian"), matches.value_of("input_type")) {
		(0, None)        => get_complex_i8_le,
		(0, Some("i8"))  => get_complex_i8_le,
		(0, Some("i16")) => get_complex_i16_le,
		(0, t) => panic!("{:?} not supported with little_endian", t),
		(_, t) => panic!("{:?} not supported with big_endian", t),
	};

	let fs_hz:f64         = matches.value_of("sample_rate_sps").unwrap().parse().unwrap();
	let resolution_hz:f64 = matches.value_of("resolution_hz").unwrap().parse().unwrap();
	let opt_max_num_ffts:Option<usize> = matches.value_of("max_num_ffts").map(|s| s.parse().unwrap());
	let n:usize           = (fs_hz / resolution_hz).log2().ceil().exp2() as usize;
	if n <= 1 { panic!("FFT size determined to be {}, must be at least 2", n); }

	let baseband_hz:f64 = matches.value_of("baseband_hz").unwrap().parse().unwrap();

	let mut avg:Vec<f64> = vec![];
	let mut buffer:Vec<Complex<f64>> = vec![];
	let mut num_ffts:usize = 0;
	while let Ok(c) = f_get(&mut f_in) {
		buffer.push(c);
		if buffer.len() == n {
			eprintln!("{} total FFTs, {} total samples", num_ffts, num_ffts*n);

			// Process this FFT
			let freq_domain:Vec<f64> = fft(&buffer).iter().map(|c| c.norm() / (n as f64)).collect();

			// Averaging
			if avg.len() == 0 { avg = freq_domain; }
			else {
				let num:f64 = num_ffts as f64;
				avg = freq_domain.iter().zip(avg.iter())
					.map(|(this_fft, avg_fft)| (this_fft + avg_fft*num) / (num + 1.0))
					.collect()
			}
			num_ffts += 1;

			// Clear the buffer for next time
			buffer.clear();

			if let Some(max_num_ffts) = opt_max_num_ffts {
				if num_ffts >= max_num_ffts { break; }
			}
		}
	}

	let step:f64 = fs_hz / (n as f64);
	let output_min:Option<f64> = matches.value_of("output_min").map(|s| s.parse().unwrap());
	let output_max:Option<f64> = matches.value_of("output_max").map(|s| s.parse().unwrap());
	let mut freq_vs_mag:Vec<(f64, f64)> = avg.iter().enumerate()
		.map(|pr| {
			let (idx, amp) = pr;
			let freq:f64 =  if idx < (n/2) { (idx as f64) * step } else { ((idx as f64)-(n as f64)) * step };
			let abs_freq:f64 = baseband_hz + freq;
			(abs_freq, *amp)
		})
		.filter(|(f, _)| { output_min.map(|min| *f >= min).unwrap_or(true) && output_max.map(|max| *f <= max).unwrap_or(true) })
		.filter(|(f, _)| *f != baseband_hz )
		.collect();
	freq_vs_mag.sort_by(|a,b| a.0.partial_cmp(&b.0).unwrap() );

	for (freq, mag) in freq_vs_mag {
		println!("{},{},{}", freq, mag, num_ffts);
	}

}