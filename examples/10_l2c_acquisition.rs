
extern crate nalgebra as na;

use std::fs::File;

use clap::{Arg, App};
use colored::*;

use rustfft::num_complex::Complex;
// use serde::{Serialize, Deserialize};

use rust_radio::block::{BlockFunctionality, BlockResult};
use rust_radio::block::block_tree_sync_static::split_and_merge::SplitAndMerge;
use rust_radio::{io::BufferedSource, Sample};
use rust_radio::gnss::common::acquisition::{self, fast_pcps, basic_pcps, AcquisitionResult};
use rust_radio::gnss::gps_l2c::signal_modulation;

const L2_CM_PERIOD_SEC:f64 = 20.0e-3;
const L2_CL_PERIOD_SEC:f64 = 1.5;

struct AcqL2 {
	acq_cm: fast_pcps::Acquisition,
	acq_cl: basic_pcps::Acquisition,
	opt_cl_result: Option<AcquisitionResult>,
}

impl BlockFunctionality<(), (), Sample, (AcquisitionResult, Option<AcquisitionResult>)> for AcqL2 {

	fn control(&mut self, _:&()) -> Result<(), &'static str> {
		Ok(())
	}

	fn apply(&mut self, input:&Sample) -> BlockResult<(AcquisitionResult, Option<AcquisitionResult>)> {
		self.acq_cm.provide_sample(input).unwrap();
		self.acq_cl.provide_sample(input).unwrap();

		self.opt_cl_result = self.acq_cl.block_for_result().unwrap();

		match self.acq_cm.block_for_result() {
			Ok(Some(result)) => {
				self.acq_cl.doppler_freqs = (-20..=20).map(|dfreq| result.doppler_hz + (dfreq as f64)).collect();

				let opt_cl_result = self.opt_cl_result.clone();
				self.opt_cl_result = None;

				BlockResult::Ready((result, opt_cl_result))
			},
			Ok(None)         => BlockResult::NotReady,
			Err(e)           => BlockResult::Err(e)
		}
	}

}

pub fn main() -> Result<(), &'static str> {

	let matches = App::new("GPS L2C Acquisition")
		.version("0.1.0")
		.author("John Stanford (johnwstanford@gmail.com)")
		.about("Takes IQ samples centered on 1227.6 MHz and produces acquisition results for the L2C signal")
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
	let src:BufferedSource<File, (i16, i16)> = BufferedSource::new(File::open(fname).unwrap()).unwrap();

	eprintln!("Decoding {} at {} [samples/sec]", &fname, &fs);

	let mut sam = SplitAndMerge::from_iter((1..=32).map( |prn| {
		// Create CM code and resample
		let cm_code:[bool; 10230] = signal_modulation::cm_code(prn);
		let n_samples:usize = (fs * L2_CM_PERIOD_SEC as f64) as usize;		// [samples/sec] * [sec]
		let mut symbol_cm:Vec<i8> = vec![];
		for sample_idx in 0..n_samples {
			let chip_idx_f64:f64 = sample_idx as f64 * (10230.0 / n_samples as f64);
			if chip_idx_f64 - chip_idx_f64.floor() < 0.5 {
				if cm_code[chip_idx_f64.floor() as usize] { symbol_cm.push(1) } else { symbol_cm.push(-1) }
			} else {
				symbol_cm.push(0)
			}
		}

		let acq_cm = acquisition::make_acquisition(symbol_cm.into_iter().map(|x| Complex{ re: x as f64, im: 0.0 }).collect(), 
			fs, prn, 140, 3, 0.0, 0);

		// Create CL code and resample
		let cl_code:[bool; 767250] = signal_modulation::cl_code(prn);
		let n_samples:usize = (fs * L2_CL_PERIOD_SEC as f64) as usize;		// [samples/sec] * [sec]
		let mut symbol_cl:Vec<i8> = vec![];
		for sample_idx in 0..n_samples {
			let chip_idx_f64:f64 = sample_idx as f64 * (767250.0 / n_samples as f64);
			if chip_idx_f64 - chip_idx_f64.floor() < 0.5 {
				if cl_code[chip_idx_f64.floor() as usize] { symbol_cl.push(1) } else { symbol_cl.push(-1) }
			} else {
				symbol_cl.push(0)
			}
		}

		let acq_cl = acquisition::basic_pcps::Acquisition::new(symbol_cl.into_iter().map(|x| Complex{ re: x as f64, im: 0.0 }).collect(), 
			fs, prn, 0.0, vec![-2.0, -1.0, 0.0, 1.0, 2.0]);

		AcqL2{ acq_cm, acq_cl, opt_cl_result: None }

	}));

	// let mut all_records:Vec<AcquisitionRecord> = vec![];

	for s in src.map(|(x, idx)| Sample{ val: Complex{ re: x.0 as f64, im: x.1 as f64 }, idx}) {

		// Send this sample to the split-and-merge block
		if let BlockResult::Ready((cm_result, opt_cl_result)) = sam.apply(&s) {
			let result_str = format!("{:9.2} [Hz], {:6} [samples], {:.8}", cm_result.doppler_hz, cm_result.code_phase, cm_result.test_statistic());
			let time:f64 = cm_result.sample_idx as f64 / fs;
			if cm_result.test_statistic() < 0.001 {
				eprint!("{:6.2} [sec], PRN {:02} CM {}", time, cm_result.id, result_str.red());
			} else if cm_result.test_statistic() < 0.003 {
				eprint!("{:6.2} [sec], PRN {:02} CM {}", time, cm_result.id, result_str.yellow());
			} else {
				eprint!("{:6.2} [sec], PRN {:02} CM {}", time, cm_result.id, result_str.green());
			}								

			if let Some(cl_result) = opt_cl_result {
				let result_str = format!("{:9.2} [Hz], {:6} [samples], {:.8}", cl_result.doppler_hz, cl_result.code_phase, cl_result.test_statistic());
				if cl_result.test_statistic() < 0.00001 {
					eprint!(", CL {}", result_str.red());
				} else if cl_result.test_statistic() < 0.001 {
					eprint!(", CL {}", result_str.yellow());
				} else {
					eprint!(", CL {}", result_str.green());
				}							

			}

			eprint!("\n");
		}


	}

	// Output data in JSON format
	// println!("{}", serde_json::to_string_pretty(&all_records).unwrap());

	Ok(())

}