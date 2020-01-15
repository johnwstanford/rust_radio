
extern crate clap;
extern crate colored;
extern crate dirs;
extern crate itertools;
extern crate nalgebra as na;
extern crate rust_radio;
extern crate rustfft;
extern crate serde;

use clap::{Arg, App};
use colored::*;
use itertools::Itertools;
use rust_radio::io;
use rust_radio::gnss::common::acquisition;
use rust_radio::gnss::common::acquisition::Acquisition;
use rust_radio::gnss::common::acquisition::fast_pcps;
use rust_radio::gnss::gps_l1_ca;
use rustfft::num_complex::Complex;
use serde::{Serialize, Deserialize};

#[derive(Debug, Serialize, Deserialize)]
struct AcquisitionRecord {
	pub prn:usize,
	pub doppler_hz:f64,
	pub code_phase:usize,
	pub test_statistic:f64,
	pub carrier_phase:f64,
}

fn main() {

	let matches = App::new("GPS L1 CA Acquisition")
		.version("0.1.0")
		.author("John Stanford (johnwstanford@gmail.com)")
		.about("Takes IQ samples centered on 1575.42 MHz and produces acquisition results for the L1 CA signal")
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
		.arg(Arg::with_name("max_records")
			.short("m").long("max_records")
			.takes_value(true))
		.get_matches();

	let fname:&str = matches.value_of("filename").unwrap();
	let fs = matches.value_of("sample_rate_sps").unwrap().parse().unwrap();
	let opt_max_records:Option<usize> = matches.value_of("max_records").map(|s| s.parse().unwrap() );

	eprintln!("Decoding {} at {} [samples/sec], max_records={:?}", &fname, &fs, &opt_max_records);

	let mut acqs:Vec<fast_pcps::Acquisition> = (1..=32).map( |prn| {

		let symbol:Vec<i8> = gps_l1_ca::signal_modulation::prn_int_sampled(prn, fs);
		let pos_bit:Vec<i8> = (1..=20).map(|_| symbol.clone()).concat();
		let neg_bit:Vec<i8> = pos_bit.iter().map(|a| -a).collect();
		let pos_neg:Vec<i8> = vec![pos_bit, neg_bit].concat();

		acquisition::make_acquisition(pos_neg, fs, prn, 320, 2, 0.0)

	}).collect();

	let mut all_records:Vec<AcquisitionRecord> = vec![];

	'outer: for s in io::file_source_i16_complex(&fname).map(|(x, idx)| (Complex{ re: x.0 as f64, im: x.1 as f64 }, idx)) {

		for acq in &mut acqs {
			let prn:usize = acq.prn;
			
			acq.provide_sample(s).unwrap();
			match acq.block_for_result(prn) {
				Ok(Some(result)) => {

					let result_str = format!("{:9.2} [Hz], {:6} [chips], {:.8}, {:8.2} [radians]", result.doppler_hz, result.code_phase, result.test_statistic(), result.mf_response.arg());
					let time:f64 = s.1 as f64 / fs;
					if result.test_statistic() < 0.005 {
						eprintln!("{:6.2} [sec], PRN {:02} {}", time, prn, result_str.yellow());
					} else {
						eprintln!("{:6.2} [sec], PRN {:02} {}", time, prn, result_str.green());
					}

					let record = AcquisitionRecord { 
						prn, 
						doppler_hz:        result.doppler_hz, 
						code_phase:        result.code_phase, 
						test_statistic:    result.test_statistic(),
						carrier_phase:     result.mf_response.arg(),
					};

					all_records.push(record)
				},
				Err(msg) => eprintln!("PRN {}: Error, {}", prn, msg),
				Ok(None) => {},
			}

			if let Some(max_records) = opt_max_records {
				if all_records.len() >= max_records { break 'outer; }
			}
		}

	}

	// Output data in JSON format
	println!("{}", serde_json::to_string_pretty(&all_records).unwrap());

}