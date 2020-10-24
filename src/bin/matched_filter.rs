
use std::fs::File;
use std::io::BufReader;

// use byteorder::{LittleEndian, WriteBytesExt};
use clap::{Arg, App};
use colored::*;
use rust_radio::{io::BufferedFileSource, Sample};
use rust_radio::filters::matched_filter::{self, MatchedFilterTestStatResult};
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
		.arg(Arg::with_name("output_filename")
			.short("o").long("output_filename")
			.help("Output filename")
			.takes_value(true))
		.arg(Arg::with_name("input_type")
			.short("t").long("type")
			.takes_value(true)
			.possible_value("i16"))
		.arg(Arg::with_name("freq_shift")
			.short("q").long("freq_shift")
			.takes_value(true))
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
	let freq_shift:f64 = matches.value_of("freq_shift").unwrap_or("0.0").parse().map_err(|_| "Unable to parse frequency shift")?;

	// Open output file
	// let mut f_out = File::create(matches.value_of("output_filename").unwrap_or("output.dat"))
	// 	.map_err(|_| "Unable to create output file")?;

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

	let mut mf = matched_filter::MatchedFilter::new(resampled_matched_filter, fs, freq_shift);

	let mut results:Vec<(f64, MatchedFilterTestStatResult)> = vec![];
	let src:BufferedFileSource<(i16, i16)> = BufferedFileSource::new(&fname).unwrap();
	for s in src.map(|(x, idx)| Sample{ val: Complex{ re: x.0 as f64, im: x.1 as f64 }, idx }) {

		match mf.apply(&s) {
			Some(result) => {

				// for c in &result.response {
				// 	f_out.write_f32::<LittleEndian>(c.re as f32).map_err(|_| "Unable to write to file")?;
				// 	f_out.write_f32::<LittleEndian>(c.im as f32).map_err(|_| "Unable to write to file")?;
				// }

				let result_test_stat = result.test_statistic();

				let result_str = format!("{:9.2} [Hz], {:6} [chips], {:.8}", result.doppler_hz, result_test_stat.max_idx, result_test_stat.test_stat);
				let t:f64 = s.idx as f64 / fs;
				if result_test_stat.test_stat < 0.01 {
					eprintln!("{:6.2} [sec] {}", t, result_str.yellow());
				} else {
					eprintln!("{:6.2} [sec] {}", t, result_str.green());
				}

				results.push((t, result_test_stat))

			},
			None => {},
		}

	}

	// Output data in JSON format
	println!("{}", serde_json::to_string_pretty(&results).unwrap());

	Ok(())

}