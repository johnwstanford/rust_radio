
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

use rust_radio::block::{BlockFunctionality, BlockResult};
use rust_radio::block::block_tree_sync_static::SeriesLeftControl;
use rust_radio::block::block_tree_sync_static::acquire_and_track::AcquireAndTrack;
use rust_radio::block::block_tree_sync_static::split_and_merge::RotatingSplitAndMerge;
use rust_radio::{io::BufferedSource, Sample};
use rust_radio::gnss::common::acquisition::two_stage_pcps;
use rust_radio::gnss::gps_l1_ca::{self, tracking::algorithm_standard};
use rust_radio::gnss::gps_l1_ca::telemetry_decode::{TelemetryDecoder, subframe::Subframe};

fn main() -> Result<(), &'static str> {

	let matches = App::new("GPS L1 CA Subframe Decoding")
		.version("0.1.0")
		.author("John Stanford (johnwstanford@gmail.com)")
		.about("Takes IQ samples centered on 1575.42 MHz and produces subframes for the L1 CA signal")
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
	let src:BufferedSource<File, (i16, i16)> = BufferedSource::new(File::open(&fname).unwrap()).unwrap();

	let fs = matches.value_of("sample_rate_sps").unwrap().parse().unwrap();
	let opt_max_records:Option<usize> = matches.value_of("max_records").map(|s| s.parse().unwrap() );

	eprintln!("Decoding {} at {} [samples/sec], max_records={:?}", &fname, &fs, &opt_max_records);

	let mut sam = RotatingSplitAndMerge::from_iter((1..=32).map( |prn| {

		let symbol:Vec<i8> = gps_l1_ca::signal_modulation::prn_int_sampled(prn, fs);
		let acq = two_stage_pcps::Acquisition::new(symbol, fs, prn, 9, 3, 50.0, 0.008, 8);
		let trk = algorithm_standard::new_1st_order_tracker(prn, 0.0, fs, 0.85, 0.55);

		let aat = AcquireAndTrack::new(acq, trk);

		let tlm = TelemetryDecoder::new();

		SeriesLeftControl::new(aat, tlm)
	}), 200_000, None);

	let mut all_records:Vec<(usize, Subframe, usize)> = vec![];

	'outer: for s in src.map(|(x, idx)| Sample{ val: Complex{ re: x.0 as f64, im: x.1 as f64 }, idx }) {

		match sam.apply(&s) {
			BlockResult::Ready((prn, sf, idx)) => {
				let s =	format!("{:?}", sf);
				eprintln!("{:4} {:6.2} [sec], PRN {:02}: {}", all_records.len(), (idx as f64)/fs, prn, s.blue());

				all_records.push((prn, sf, idx));
			},
			_ => ()
		}

		if let Some(max_records) = opt_max_records {
			if all_records.len() >= max_records { break 'outer; }
		}

	}

	// Output data in JSON format
	println!("{}", serde_json::to_string_pretty(&all_records).unwrap());

	Ok(())

}