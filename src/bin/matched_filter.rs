
use std::fs::File;
use std::io::BufReader;

use clap::{Arg, App};
use serde::{Serialize, Deserialize};

#[derive(Debug, Serialize, Deserialize)]
struct MatchedFilterSpec {
	filter: Vec<f64>,
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

	// Open specification file
	let spec:MatchedFilterSpec = {
		let fname:&str = matches.value_of("json_spec").ok_or("No JSON specification file provided")?;
		let file = File::open(fname).map_err(|_| "Unable to open JSON specification file")?;
		let reader = BufReader::new(file);
		serde_json::from_reader(reader).map_err(|_| "Unable to parse JSON specification")?
	};

	println!("{:?}", spec);

	Ok(())

}