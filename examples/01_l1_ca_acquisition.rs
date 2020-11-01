
extern crate clap;
extern crate colored;
extern crate dirs;
extern crate nalgebra as na;
extern crate rust_radio;
extern crate rustfft;
extern crate serde;

use std::fs::File;

use clap::{Arg, App};
use colored::*;
use rustfft::num_complex::Complex;

use uhd_rs::usrp::USRP;
use uhd_rs::job::{Job, simple_rx};

use rust_radio::block::Block;
use rust_radio::{io::BufferedSource, Sample};
use rust_radio::gnss::common::acquisition::{self, AcquisitionResult};
use rust_radio::gnss::gps_l1_ca;

#[tokio::main]
pub async fn main() -> Result<(), &'static str> {

	let matches = App::new("GPS L1 CA Acquisition")
		.version("0.1.0")
		.author("John Stanford (johnwstanford@gmail.com)")
		.about("Takes IQ samples centered on 1575.42 MHz and produces acquisition results for the L1 CA signal")
		.arg(Arg::with_name("filename")
			.long("filename")
			.help("Input filename")
			.required_unless("usrp").takes_value(true))
		.arg(Arg::with_name("usrp")
			.long("usrp")
			.help("USRP device arguments; can be an empty string")
			.required_unless("filename").takes_value(true))
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

	let fs = matches.value_of("sample_rate_sps").unwrap().parse().unwrap();
	let opt_max_records:Option<usize> = matches.value_of("max_records").map(|s| s.parse().unwrap() );

	let src:Box<dyn Iterator<Item = ((i16, i16), usize)>> = match (matches.value_of("filename"), matches.value_of("usrp")) {
		(Some(fname), _) => {
			eprintln!("Decoding {} at {} [samples/sec], max_records={:?}", &fname, &fs, &opt_max_records);
			Box::new(BufferedSource::new(File::open(&fname).unwrap()).unwrap())
		},
		(_, Some(usrp_args)) => {
			eprintln!("Creating USRP device with args {:?}", &usrp_args);

			panic!("Need to finish");
		},
		(None, None) => panic!("No filename or USRP args; clap should have caught this")
	};


	let mut acqs:Vec<Block<(), Sample, AcquisitionResult>> = (1..=32).map( |prn| {

		let symbol:Vec<i8> = gps_l1_ca::signal_modulation::prn_int_sampled(prn, fs);
		let acq = acquisition::make_acquisition(symbol, fs, prn, 9, 17, 0.008, 0);

        Block::from(acq)

	}).collect();

	let mut all_records:Vec<AcquisitionResult> = vec![];

	'outer: for s in src.map(|(x, idx)| Sample{ val: Complex{ re: x.0 as f64, im: x.1 as f64 }, idx }) {

		// Send this sample to all acquisition blocks
		for block in &mut acqs {
			block.tx_input.send(s.clone()).await.unwrap();
		}

		// Receive results from all acquisition blocks
		for block in &mut acqs {
			while let Ok(result) = block.rx_output.try_recv() {
					let result_str = format!("{:9.2} [Hz], {:6} [chips], {:.8}, {:8.2} [radians]", result.doppler_hz, result.code_phase, result.test_statistic(), result.mf_response.arg());
					let time:f64 = result.sample_idx as f64 / fs;
					if result.test_statistic() < 0.01 {
						eprintln!("{:6.2} [sec], PRN {:02} {}", time, result.id, result_str.yellow());
					} else {
						eprintln!("{:6.2} [sec], PRN {:02} {}", time, result.id, result_str.green());
					}				
				all_records.push(result);
			}
		}

		if let Some(max_records) = opt_max_records {
			if all_records.len() >= max_records { break 'outer; }
		}

	}

	// Drop all transmit channels; then wait for the spawned threads to finish
	for block in acqs { block.shutdown().await?; }

	// Output data in JSON format
	println!("{}", serde_json::to_string_pretty(&all_records).unwrap());

	Ok(())

}