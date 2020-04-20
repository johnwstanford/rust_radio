
extern crate rustfft;

use rustfft::num_complex::Complex;

pub mod filters;
pub mod fourier_analysis;
pub mod io;
pub mod gnss;
pub mod types;

pub mod utils;

// Used to use a tuple for this, but now I want to use a struct
// that explicitly doesn't implement Copy or Clone; I think this 
// will help me set up for concurrency
pub struct Sample {
	pub val: Complex<f64>,
	pub idx: usize,
}

#[derive(Debug, Hash, PartialEq, Eq, Clone, Copy)]
pub enum DigSigProcErr {
    NoSourceData,
    LossOfLock,
    InvalidTelemetryData(&'static str),
}
