
use clap::{Arg, App};

use num_complex::Complex;

use rand_distr::{Distribution, Normal};

fn main() -> Result<(), &'static str> {
	
	let matches = App::new("FFT Performance Test")
		.version("0.1.0")
		.author("John Stanford (johnwstanford@gmail.com)")
		.about("Measure the performance of several different FFT algorithms")
		.arg(Arg::with_name("size").long("size").help("FFT size"))
		.arg(Arg::with_name("num_trials").long("num_trials").help("Number of trials"))
		.get_matches();

	let size_fft:usize = matches.value_of("size").unwrap_or("4096").parse().unwrap_or(4096);
	let num_trials:usize = matches.value_of("num_trials").unwrap_or("100").parse().unwrap_or(100);

	let mut rng = rand::thread_rng();

	let noise = Normal::new(0.0, 100.0).unwrap();
	let time_domain:Vec<Complex<f64>> = (0..size_fft).map(|_| Complex{ re: noise.sample(&mut rng), im: noise.sample(&mut rng) }).collect();

	println!("{:?}", time_domain);

	Ok(())

}