//! Motor feedforward controller (kS + kV x v + kA x a).
//!
//! Combine with a PID controller for fast, low-overshoot motion profiles.
//!
//! ```rust
//! let ff = MotorFeedforward::new(0.1, 2.0, 0.05);
//! let voltage = ff.calculate(target_velocity, target_acceleration);
//! ```

/// Simple kS / kV / kA feedforward for a DC motor.
#[derive(Debug, Clone, Copy)]
pub struct MotorFeedforward {
    /// Static friction compensation (V).
    pub ks: f64,
    /// Velocity feedforward (V x s/unit).
    pub kv: f64,
    /// Acceleration feedforward (V x s^2/unit).
    pub ka: f64,
}

impl MotorFeedforward {
    pub fn new(ks: f64, kv: f64, ka: f64) -> Self {
        Self { ks, kv, ka }
    }

    /// Compute the feedforward voltage.
    ///
    /// - `velocity`     - desired velocity (same units as kv)
    /// - `acceleration` - desired acceleration (same units as ka)
    pub fn calculate(&self, velocity: f64, acceleration: f64) -> f64 {
        self.ks * velocity.signum() + self.kv * velocity + self.ka * acceleration
    }
}
