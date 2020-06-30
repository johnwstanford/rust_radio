
use std::fs::File;
use std::io::BufReader;

use clap::{Arg, App};
use colored::*;
use rust_radio::{io, Sample};
use rust_radio::filters::matched_filter;
use rustfft::num_complex::Complex;
use serde::{Serialize, Deserialize};

#[derive(Debug, Serialize, Deserialize)]
struct MatchedFilterSpec {
	filter: Vec<i8>,
	filter_sample_rate_sps: Option<f64>,
	filter_length_sec: Option<f64>
}

fn main() -> Result<(), &'static str> {

	let matches = App::new("Matched Filter")
		.version("0.1.0")
		.author("John Stanford (johnwstanford@gmail.com)")
		.about("Takes IQ samples and a JSON-formatted matched filter specification and produces filter responses")
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
		.arg(Arg::with_name("json_spec")
			.short("j").long("json_spec")
			.required(true).takes_value(true))
		.get_matches();

	// Read command line arguments related to input file
	let fname:&str = matches.value_of("filename").ok_or("No input filename provided")?;
	let fs:f64 = matches.value_of("sample_rate_sps").ok_or("No sample rate provided")?.parse().map_err(|_| "Unable to parse sample rate as an f64")?;

	// Open specification file
	let spec:MatchedFilterSpec = {
		let fname:&str = matches.value_of("json_spec").ok_or("No JSON specification file provided")?;
		let file = File::open(fname).map_err(|_| "Unable to open JSON specification file")?;
		let reader = BufReader::new(file);
		serde_json::from_reader(reader).map_err(|_| "Unable to parse JSON specification")?
	};

	let (filter_length_sec, filter_sample_rate_sps) = match (spec.filter_length_sec, spec.filter_sample_rate_sps) {
		(Some(filter_length_sec), None) => 
			// [samples] / [sec] = [samples/sec]
			(filter_length_sec, (spec.filter.len() as f64) / filter_length_sec),
		(None, Some(filter_sample_rate_sps)) =>
			// [samples] / [samples/sec] = [sec] 
			((spec.filter.len() as f64) / filter_sample_rate_sps, filter_sample_rate_sps),
		(_, _) => return Err("Must specify either matched filter sample rate or length, but not both")
	};

	// Resample matched filter
	let resampled_len_samples:usize = (fs * filter_length_sec) as usize;
	let resampled_matched_filter:Vec<i8> = (0..resampled_len_samples).map(|i| {
		let idx:f64 = (i as f64) * (filter_sample_rate_sps / fs);
		spec.filter[(idx as usize) % spec.filter.len()]
	}).collect();

	let mut mf = matched_filter::MatchedFilter::new(resampled_matched_filter, fs, 2275.0);

	for s in io::file_source_i16_complex(&fname).map(|(x, idx)| Sample{ val: Complex{ re: x.0 as f64, im: x.1 as f64 }, idx }) {

		mf.provide_sample(&s).unwrap();
			match mf.block_for_result() {
				Ok(Some(result)) => {

					let result_str = format!("{:9.2} [Hz], {:6} [chips], {:.8}, {:8.2} [radians]", result.doppler_hz, result.code_phase, result.test_statistic(), result.mf_response.arg());
					let time:f64 = s.idx as f64 / fs;
					if result.test_statistic() < 0.01 {
						eprintln!("{:6.2} [sec] {}", time, result_str.yellow());
					} else {
						eprintln!("{:6.2} [sec] {}", time, result_str.green());
					}

				},
				Err(msg) => eprintln!("Error, {}", msg),
				Ok(None) => {},
			}

	}

	Ok(())

}