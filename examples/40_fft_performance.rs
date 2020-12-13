
use std::time::Instant;

use clap::{Arg, App};

use num_complex::Complex;
use num_traits::Zero;
use rustfft::FFTplanner;

use rand_distr::{Distribution, Normal};

fn main() -> Result<(), &'static str> {
	
	let matches = App::new("FFT Performance Test")
		.version("0.1.0")
		.author("John Stanford (johnwstanford@gmail.com)")
		.about("Measure the performance of several different FFT algorithms")
		.arg(Arg::with_name("size").long("size").help("FFT size"))
		.arg(Arg::with_name("num_trials").long("num_trials").help("Number of trials"))
		.get_matches();

	let size_fft:usize = matches.value_of("size").unwrap_or("2048").parse().unwrap_or(2048);
	let num_trials:usize = matches.value_of("num_trials").unwrap_or("100").parse().unwrap_or(100);

	let mut rng = rand::thread_rng();

	let noise = Normal::new(0.0, 100.0).unwrap();
	let time_domain:Vec<Complex<f64>> = (0..size_fft).map(|_| Complex{ re: noise.sample(&mut rng), im: noise.sample(&mut rng) }).collect();

	/* Algorithm 1: rustfft */
	let time1:f64 = {
		// Set up
		let mut fft_out: Vec<Complex<f64>> = vec![Complex::zero(); size_fft];
		let mut planner = FFTplanner::new(false);
		let fft = planner.plan_fft(size_fft);
		let mut time_domain = time_domain.clone();

		// Time
		let start = Instant::now();
		fft.process(&mut time_domain, &mut fft_out);
		start.elapsed().as_secs_f64()
	};

	/* Algorithm 1: Rust Radio */
	let time2:f64 = {
		let mut time_domain = time_domain.clone();

		// Time
		let start = Instant::now();
		let _freq_domain = rust_radio::fourier_analysis::fft(&time_domain);
		start.elapsed().as_secs_f64()
	};

	println!("rustfft:    {:?}", time1);
	println!("Rust Radio: {:?}", time2);

	Ok(())

}