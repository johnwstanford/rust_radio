
use rustfft::num_complex::Complex;

pub mod block;

pub mod filters;
pub mod fourier_analysis;
pub mod io;
pub mod gnss;
pub mod types;

pub mod utils;

#[derive(Debug, Clone)]
pub struct Sample {
	pub val: Complex<f64>,
	pub idx: usize,
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum DigSigProcErr {
    LossOfLock,
    InvalidTelemetryData(&'static str),
    Other(&'static str),
}
