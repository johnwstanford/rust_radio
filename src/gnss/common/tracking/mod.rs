
use serde::{Serialize, Deserialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrackReport {
	pub id: usize,
	pub prompt_i: f64,
	pub sample_idx: usize,
	pub test_stat: f64,
	pub freq_hz: f64
}
