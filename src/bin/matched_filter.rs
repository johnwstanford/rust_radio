
extern crate clap;

use clap::{Arg, App};

fn main() {

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

}