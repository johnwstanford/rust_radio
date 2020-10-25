
use rustfft::num_complex::Complex;

pub mod filters;
pub mod fourier_analysis;
pub mod io;
pub mod gnss;
pub mod types;

pub mod utils;

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
