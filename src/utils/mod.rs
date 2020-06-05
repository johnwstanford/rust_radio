
pub mod bools_to_int;
pub mod kinematics;

pub struct IntegerClock {
    start_time: f64,
    start_time_updated_at_least_once: bool,
    clock_rate_hz: f64,
    cycles: usize
}

impl IntegerClock {
    
    pub fn new(clock_rate_hz: f64) -> Self {
        Self{ start_time: 0.0, start_time_updated_at_least_once: false, clock_rate_hz, cycles: 0}
    }

    pub fn reset(&mut self, t:f64) {
        self.start_time = t;
        self.start_time_updated_at_least_once = true;
        self.cycles = 0;
    }

    pub fn inc(&mut self) { self.cycles += 1; }
    pub fn set_clock_rate(&mut self, hz:f64) { self.clock_rate_hz = hz; }

    pub fn time(&self) -> f64 { self.start_time + ((self.cycles as f64)/self.clock_rate_hz) }
    pub fn has_start(&self) -> bool { self.start_time_updated_at_least_once }
}

