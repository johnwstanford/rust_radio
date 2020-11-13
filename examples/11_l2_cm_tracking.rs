
extern crate nalgebra as na;

use std::collections::HashMap;
use std::io::Write;
use std::fs::File;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use clap::{Arg, App};
use crossterm::{
    execute,
    style::{self, Color}, cursor, terminal//, Result
};

use rust_radio::{io::BufferedSource, Sample};
use rust_radio::block::Block;
use rust_radio::block::block_tree_sync_static::acquire_and_track::AcquireAndTrack;
use rust_radio::block::block_tree_sync_static::split_and_merge::RotatingSplitAndMerge;
use rust_radio::gnss::common::acquisition::two_stage_pcps;
use rust_radio::gnss::common::tracking::TrackReport;
use rust_radio::gnss::gps_l2c::{signal_modulation, L2_CM_PERIOD_SEC};
use rust_radio::gnss::gps_l2c::tracking_cm;
use rustfft::num_complex::Complex;

const DISPLAY_AGE_OUT_TIME_SEC:f64 = 3.0;
const DISPLAY_RATE_SAMPLES:usize = 200_000;

#[derive(Debug, Default, Clone)]
struct ChannelState {
	pub test_stat: f64,
	pub carrier_hz: f64,
	pub last_update_sec: f64,
}

#[tokio::main]
pub async fn main() -> Result<(), &'static str> {

    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    }).expect("Error setting Ctrl-C handler");

	let matches = App::new("GPS L2 CM Tracker")
		.version("0.1.0")
		.author("John Stanford (johnwstanford@gmail.com)")
		.about("Takes IQ samples centered on 1227.6 MHz and produces subframes and fixes for the L2C signal")
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
		.arg(Arg::with_name("outfile_json")
			.long("outfile_json")
			.takes_value(true))
		.get_matches();

	let fname:&str = matches.value_of("filename").unwrap();
	let fs = matches.value_of("sample_rate_sps").unwrap().parse().unwrap();

	eprintln!("Decoding {} at {} [samples/sec]", &fname, &fs);

	let sam = RotatingSplitAndMerge::from_iter((1..=32).map( |prn| {

		let acq: two_stage_pcps::Acquisition = {
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

			two_stage_pcps::Acquisition::new(symbol_cm, fs, prn, 140, 2, 2.0, 0.0005, 0)
		};

		// Tracking
		let trk = tracking_cm::new_default_tracker(prn, 0.0, fs);

		AcquireAndTrack::new(acq, trk)

	}), 200_000, None);

	let mut blk:Block<(), (usize, usize), Sample, TrackReport> = Block::from(sam);

	let mut all_records:Vec<TrackReport> = vec![];

	let mut display_state:HashMap<usize, ChannelState> = HashMap::new();

	let src:BufferedSource<File, (i16, i16)> = BufferedSource::new(File::open(fname).unwrap()).unwrap();
	'outer: for s in src.map(|(x, idx)| Sample{ val: Complex{ re: x.0 as f64, im: x.1 as f64 }, idx}) {

		// Process diplay
		if s.idx % DISPLAY_RATE_SAMPLES == 0 {
			let current_time_sec:f64 = (s.idx as f64) / fs;

			// Process age-out
			display_state = display_state.into_iter().filter(|(_, state)| (state.last_update_sec - current_time_sec).abs() < DISPLAY_AGE_OUT_TIME_SEC).collect();

			let mut stderr = std::io::stderr();

			execute!(stderr, terminal::Clear(terminal::ClearType::All)).unwrap();
			execute!(stderr, style::SetForegroundColor(Color::White)).unwrap();

			execute!(stderr, cursor::MoveTo(5,1)).unwrap();
			eprintln!("{:6.2} [sec]: {} SVs tracked", current_time_sec, display_state.len());

			let mut sv_states:Vec<(usize, ChannelState)> = display_state.iter().map(|(prn, state)| (prn.clone(), state.clone())).collect();
			sv_states.sort_by_key(|x| x.0);

			for (prn, state) in sv_states {
				if      state.test_stat > 0.003 { execute!(stderr, style::SetForegroundColor(Color::Green)).unwrap();  }
				else if state.test_stat > 0.001 { execute!(stderr, style::SetForegroundColor(Color::Yellow)).unwrap(); }
				else                            { execute!(stderr, style::SetForegroundColor(Color::Red)).unwrap();    }

				eprintln!("PRN {:02}: test_stat={:.5}, carrier={:7.3} [kHz], updated {:.1} [sec] ago", prn, 
					state.test_stat, state.carrier_hz*1.0e-3, current_time_sec-state.last_update_sec);
			}
			
			execute!(stderr, style::SetForegroundColor(Color::White)).unwrap();

		}

		blk.apply(s).await.unwrap();

		if let Ok(report) = blk.try_recv() {

			if !display_state.contains_key(&report.id) { display_state.insert(report.id, ChannelState::default()); }
			
			if let Some(state) = display_state.get_mut(&report.id) {
				state.test_stat = report.test_stat;
				state.carrier_hz = report.freq_hz;
				state.last_update_sec = (report.sample_idx as f64)/fs;
			}

			all_records.push(report);
		}

		// Break out of this loop if SIGINT is detected (Ctrl-C)
		if !running.load(Ordering::SeqCst) { break 'outer; }

	}

	// Output data in JSON format
	if let Some(outfile) = matches.value_of("outfile_json") {
		eprintln!("Outputting results to {}", outfile);
		std::fs::write(outfile, serde_json::to_string_pretty(&all_records).unwrap().as_bytes()).map_err(|_| "Unable to write prompt JSON")?;
	} else {
		eprintln!("No JSON output file specified");
	}


	Ok(())

}