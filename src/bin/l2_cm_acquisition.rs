
extern crate clap;
extern crate dirs;
extern crate nalgebra as na;
extern crate rust_radio;
extern crate rustfft;
extern crate serde;

use clap::{Arg, App};
use rust_radio::io;
use rust_radio::gnss::common::acquisition;
use rust_radio::gnss::common::acquisition::Acquisition;
use rust_radio::gnss::common::acquisition::fast_pcps;
use rust_radio::gnss::gps_l2c::signal_modulation;
use rustfft::num_complex::Complex;
use serde::{Serialize, Deserialize};

#[derive(Debug, Serialize, Deserialize)]
struct AcquisitionRecord {
	pub prn:usize,
	pub doppler_hz:f64,
	pub code_phase:usize,
	pub test_statistic:f64,
}

const L2_CM_PERIOD_SEC:f64 = 20.0e-3;

fn main() {

	let matches = App::new("GPS L2 CM Acquisition")
		.version("0.1.0")
		.author("John Stanford (johnwstanford@gmail.com)")
		.about("Takes IQ samples centered on 1227.6 MHz and produces acquisition results for the L2 CM signal")
		.arg(Arg::with_name("filename")
			.short("f").long("filename")
			.help("Input filename")
			.required(true).takes_value(true))
		.arg(Arg::with_name("input_type")
			.short("t").long("type")
			.takes_value(true)
			.possible_value("i16"))
		.arg(Arg::with_name("sample_rate_sps")
			.short("s").long("sample_rate_sps")
			.takes_value(true).required(true))
		.get_matches();

	let fname:&str = matches.value_of("filename").unwrap();
	let fs = matches.value_of("sample_rate_sps").unwrap().parse().unwrap();

	eprintln!("Decoding {} at {} [samples/sec]", &fname, &fs);

	let mut acqs:Vec<fast_pcps::Acquisition> = (1..=32).map( |prn| {
		let cm_code:[bool; 10230] = signal_modulation::cm_code(prn);

		// Convert bool to i8 and resample
		let n_samples:usize = (fs * L2_CM_PERIOD_SEC as f64) as usize;		// [samples/sec] * [sec]
		let mut symbol:Vec<i8> = vec![];
		for sample_idx in 0..n_samples {
			let chip_idx_f64:f64 = sample_idx as f64 * (10230.0 / n_samples as f64);
			if chip_idx_f64 - chip_idx_f64.floor() < 0.5 {
				if cm_code[chip_idx_f64.floor() as usize] { symbol.push(1) } else { symbol.push(-1) }
			} else {
				symbol.push(0)
			}
		}

		acquisition::make_acquisition(symbol, fs, prn, 140, 12, 0.0)

	}).collect();

	let mut all_records:Vec<AcquisitionRecord> = vec![];

	for s in io::file_source_i16_complex(&fname).map(|(x, idx)| (Complex{ re: x.0 as f64, im: x.1 as f64 }, idx)) {

		for acq in &mut acqs {
			let prn:usize = acq.prn;
			acq.provide_sample(s).unwrap();
			match acq.block_for_result(prn) {
				Ok(Some(result)) => {
					all_records.push(AcquisitionRecord{ prn, doppler_hz: result.doppler_hz, code_phase: result.code_phase, test_statistic: result.test_statistic});
					eprintln!("PRN {} {:?}", prn, result)
				},
				Err(msg) => eprintln!("PRN {}: Error, {}", prn, msg),
				Ok(None) => {},
			}
		}

		if all_records.len() >= 100 { break; }

	}

	// Output data in JSON format
	println!("{}", serde_json::to_string_pretty(&all_records).unwrap());

}