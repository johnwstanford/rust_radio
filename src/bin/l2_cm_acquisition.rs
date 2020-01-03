
extern crate clap;
extern crate colored;
extern crate dirs;
extern crate nalgebra as na;
extern crate rust_radio;
extern crate rustfft;
extern crate serde;

use std::collections::VecDeque;

use clap::{Arg, App};
use colored::*;
use rust_radio::io;
use rust_radio::gnss::common::acquisition::fast_pcps;
use rust_radio::gnss::gps_l2c::signal_modulation;
use rustfft::num_complex::Complex;
use serde::{Serialize, Deserialize};

/*#[derive(Debug, Serialize, Deserialize)]
struct SubframeWithMetadata {
	subframe: SF,
	carrier_freq_hz:f64,
	cn0_snv_db_hz:f64,
	carrier_lock_test:f64,
	acq_test_stat:f64,
	prn:usize,
	snr_coh:f64,
}*/

fn main() {

	for prn in 1..=32 {
		let code = signal_modulation::cm_code(prn);
		println!("{} {:?}", prn, code[10229]);
	}

	/*let matches = App::new("GPS L2 CM Acquisition")
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

	for s in io::file_source_i16_complex(&fname).map(|(x, idx)| (Complex{ re: x.0 as f64, im: x.1 as f64 }, idx)) {

	}

	// Output data in JSON format
	println!("{}", serde_json::to_string_pretty(&all_results).unwrap());*/

}