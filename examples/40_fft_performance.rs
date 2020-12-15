
use std::time::Instant;

use clap::{Arg, App};

use num_complex::Complex;
use num_traits::Zero;
use rustfft::FFTplanner;

use rand_distr::{Distribution, Normal};

#[link(name="fftw3")]
extern {

	fn fftw_malloc(size:usize) -> *mut Complex<f64>;
	fn fftw_free(handle:*mut Complex<f64>);

	fn fftw_plan_dft_1d(n:isize, input:*mut Complex<f64>, output:*mut Complex<f64>, sign:isize, flags:usize) -> usize;
	fn fftw_execute(plan:usize);
	fn fftw_destroy_plan(plan:usize);
}

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
		let ans = start.elapsed().as_secs_f64();

		println!("rustfft freq domain    = [{:?}, ...]", fft_out[0]);

		ans
	};

	/* Algorithm 2: Rust Radio */
	let time2:f64 = {
		let time_domain = time_domain.clone();

		// Time
		let start = Instant::now();
		let freq_domain = rust_radio::fourier_analysis::fft(&time_domain);
		let ans:f64 = start.elapsed().as_secs_f64();

		println!("Rust Radio freq domain = [{:?}, ...]", freq_domain[0]);

		ans
	};

	/* Algorithm 3: FFTW */
	let time3:f64 = unsafe {
		let h_in:*mut Complex<f64> = fftw_malloc(std::mem::size_of::<(f64, f64)>() * size_fft);
		let h_out:*mut Complex<f64> = fftw_malloc(std::mem::size_of::<(f64, f64)>() * size_fft);

		let in_slice:&mut [Complex<f64>] = std::slice::from_raw_parts_mut(h_in, size_fft);
		let out_slice:&mut [Complex<f64>] = std::slice::from_raw_parts_mut(h_out, size_fft);

		let p = fftw_plan_dft_1d(size_fft as isize, h_in, h_out, 1, 0);

		for idx in 0..size_fft {
			in_slice[idx] = time_domain[idx].clone();	
		}

		// Time
		let start = Instant::now();
		fftw_execute(p);
		let ans:f64 = start.elapsed().as_secs_f64();

		println!("FFTW freq domain       = [{:?}, ...]", out_slice[0]);

		// Cleanup
		fftw_destroy_plan(p);
		fftw_free(h_in);
		fftw_free(h_out);

		ans
	};

	println!("rustfft:    {:?}", time1);
	println!("Rust Radio: {:?}", time2);
	println!("FFTW:       {:?}", time3);

	Ok(())

}