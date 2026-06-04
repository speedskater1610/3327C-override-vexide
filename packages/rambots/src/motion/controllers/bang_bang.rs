//! Bang-bang (on-off) controller.
//!
//! Useful as a fast, simple controller for mechanisms with binary states
//! or as a fallback when precise PID tuning is not yet available.

/// A bang-bang controller that outputs `high_output` when below the setpoint
/// and `low_output` when at or above it.
#[derive(Debug, Clone)]
pub struct BangBang {
    pub high_output: f64,
    pub low_output: f64,
    /// Dead-band around the setpoint: + or -`tolerance`.
    pub tolerance: f64,
}

impl BangBang {
    /// Create a new bang-bang controller.
    ///
    /// - `high_output` - output when error > tolerance
    /// - `low_output`  - output when error < -tolerance
    /// - `tolerance`   - half the dead-band width
    pub fn new(high_output: f64, low_output: f64, tolerance: f64) -> Self {
        Self { high_output, low_output, tolerance }
    }

    /// Compute the control output.
    pub fn update(&self, setpoint: f64, measurement: f64) -> f64 {
        let error = setpoint - measurement;
        if error > self.tolerance {
            self.high_output
        } else if error < -self.tolerance {
            self.low_output
        } else {
            0.0
        }
    }
}
