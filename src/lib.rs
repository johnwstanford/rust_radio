
pub mod filters;
pub mod fourier_analysis;
pub mod io;
pub mod gnss;
pub mod types;

pub mod utils;

#[derive(Debug, Hash, PartialEq, Eq, Clone, Copy)]
pub enum DigSigProcErr {
    NoSourceData,
    LossOfLock,
    InvalidTelemetryData(&'static str),
}
