
extern crate clap;
extern crate colored;
extern crate dirs;
extern crate nalgebra as na;
extern crate regex;
extern crate rustfft;
extern crate rust_radio;
extern crate serde;

use std::collections::{VecDeque, HashMap};
use std::fs::File;
use std::io::{BufReader, Write};

use clap::{Arg, App};
use colored::*;
use regex::Regex;
use rustfft::num_complex::Complex;
use rust_radio::io;
use rust_radio::gnss::{channel, pvt};
use rust_radio::gnss::acquisition::basic_pcps;

const NUM_ACTIVE_CHANNELS:usize = 7;

fn main() {

	let matches = App::new("GPS Ephemerides")
		.version("0.1.0")
		.author("John Stanford (johnwstanford@gmail.com)")
		.about("Stores GPS ephemerides in JSON format and can reload and analyze them")
		.arg(Arg::with_name("json_file")
			.long("json_file")
			.help("JSON file used to store ephemerides data")
			.takes_value(true).required(true))
		.arg(Arg::with_name("src_file")
			.long("src_file")
			.help("Binary source file used to update ephemerides")
			.takes_value(true))
		.arg(Arg::with_name("sample_rate_sps")
			.long("sample_rate_sps")
			.takes_value(true))
		.arg(Arg::with_name("ecef")
			.long("ecef")
			.takes_value(true))
		.get_matches();

	let json_filename:&str = matches.value_of("json_file").unwrap();

	//                               PRN            week	     iodc
	let mut ephemerides_data:HashMap<usize, HashMap<u16, HashMap<u16, pvt::CalendarAndEphemeris>>> = match File::open(json_filename) {
		Ok(json_file) => serde_json::from_reader(BufReader::new(json_file)).unwrap(),
		Err(_) => HashMap::new()
	};

	// Update JSON
	match (matches.value_of("src_file"), matches.value_of("sample_rate_sps")) {
		(Some(fname), Some(fs_str)) => {
			let fs:f64 = fs_str.parse().unwrap();
			eprintln!("Decoding {} at {} [samples/sec]", &fname, &fs);
		
			let mut inactive_channels:VecDeque<channel::Channel<basic_pcps::Acquisition>> = (1..=32).map(|prn| channel::new_channel(prn, fs, 0.0, 0.01)).collect();
			let mut active_channels:VecDeque<channel::Channel<basic_pcps::Acquisition>>   = inactive_channels.drain(..NUM_ACTIVE_CHANNELS).collect();

			for s in io::file_source_i16_complex(&fname).map(|(x, idx)| (Complex{ re: x.0 as f64, im: x.1 as f64 }, idx)) {

				for chn in &mut active_channels {
					match chn.apply(s) {
						channel::ChannelResult::Acquisition{ doppler_hz, test_stat } => {
							eprintln!("{}", format!("PRN {}: Acquired at {} [Hz] doppler, {} test statistic, attempting to track", chn.prn, doppler_hz, test_stat).green());
						},
						channel::ChannelResult::Err(e) => eprintln!("{}", format!("PRN {}: Error due to {:?}", chn.prn, e).red()),
						_ => {}
					}

					// Add new calendar and ephemeris data if available
					match chn.calendar_and_ephemeris() {
						Some(cae) => {
							let this_prn:&mut HashMap<u16, HashMap<u16, pvt::CalendarAndEphemeris>> = ephemerides_data.entry(chn.prn).or_insert(HashMap::new());
							let this_week:&mut HashMap<u16, pvt::CalendarAndEphemeris> = this_prn.entry(cae.week_number).or_insert(HashMap::new());
							match this_week.insert(cae.iodc, cae) {
								Some(_) => {},
								None => { 
									eprintln!("New ephemeris for PRN {}: {:?}", chn.prn, &cae);
								}
							}
						},
						None => {}
					}

				}

				// Every 0.1 sec, move channels without a signal lock to the inactive buffer and replace them with new ones
				if (s.1 % (fs as usize / 10) == 0) && (s.1 > 0) {
					for _ in 0..NUM_ACTIVE_CHANNELS {
						let this_channel = active_channels.pop_front().unwrap();
						if this_channel.state() == channel::track_and_tlm::ChannelState::AwaitingAcquisition {
							// Move this channel to inactive and replace it
							let replacement_channel = inactive_channels.pop_front().unwrap();
							eprintln!("{:.1} [sec]: Putting PRN {} in the inactive buffer, replacing with PRN {}", (s.1 as f64)/fs, this_channel.prn, replacement_channel.prn);
							inactive_channels.push_back(this_channel);
							active_channels.push_back(replacement_channel);
						} else {
							// Keep this channel in the active buffer
							active_channels.push_back(this_channel);
						}
					}
					assert!(active_channels.len() == NUM_ACTIVE_CHANNELS);
					assert!(inactive_channels.len() == (32 - NUM_ACTIVE_CHANNELS));
				}

			}
		},
		(_, _) => {}
	}

	// Find ECEF location if requested
	if let Some(ecef_request) = matches.value_of("ecef") {
		// Example format: prn=22,week=13,tow_ms=324906741.046
		let re = Regex::new(r"prn=(\d+),week=(\d+),tow_ms=(\d+\.\d+)").unwrap();
		match re.captures(ecef_request) {
			Some(capts) => {
				// Parse request
				let prn:usize = capts[1].parse().unwrap();
				let week:u16 = capts[2].parse().unwrap();
				let tow_ms:f64 = capts[3].parse().unwrap();
				println!("Finding ECEF for PRN {} at {} [ms] of week {}", prn, tow_ms, week);

				// Find results, if any
				match ephemerides_data.get(&prn) {
					Some(this_prn) => { 
						match this_prn.get(&week) {
							Some(this_week) => {
								// TODO: account for GPS week roll-over
								let ephs_within_two_hours:Vec<pvt::CalendarAndEphemeris> = this_week.values()
									.filter(|cae| ((cae.t_oe * 1000.0) - tow_ms).abs() <= 7200000.0 ).map(|cae| *cae).collect();
								if ephs_within_two_hours.len() > 0 {
									for cae in ephs_within_two_hours {
										println!("{:?}", cae.pos_and_clock(tow_ms / 1000.0).0);
									}
								} else {
									println!("No ephemeris data with two hours for PRN {} during week {}", prn, week);
								}
							},
							None => println!("No ephemeris data for PRN {} during week {}", prn, week)
						}
					},
					None => println!("No ephemeris data for PRN {}", prn)
				};
			},
			None => {}
		}
	}

	// Output JSON data
	let mut file = File::create(json_filename).unwrap();
    file.write_all(serde_json::to_string_pretty(&ephemerides_data).unwrap().as_bytes()).unwrap();
}