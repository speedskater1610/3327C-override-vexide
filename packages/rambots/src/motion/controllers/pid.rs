//! Discrete PID controller with anti windup and derivative low pass filter.
//!
//! # Example
//! ```rust
//! let mut pid = Pid::new(0.5, 0.01, 0.1)
//!     .with_integral_limit(100.0)
//!     .with_derivative_filter(0.8);
//!
//! loop {
//!     let output = pid.update(setpoint, measurement, dt_secs);
//!     motor.set_voltage(output);
//! }
//! ```

/// A PID controller.
#[derive(Debug, Clone)]
pub struct Pid {
    /// Proportional gain.
    pub kp: f64,
    /// Integral gain.
    pub ki: f64,
    /// Derivative gain.
    pub kd: f64,

    integral_limit: f64,
    derivative_filter_coeff: f64, // a in [0, 1]; 1.0 = no filter

    integral: f64,
    prev_error: f64,
    filtered_derivative: f64,
}

impl Pid {
    /// Create a new PID with the given gains.
    /// Integral limit defaults to `f64::MAX` (no clamping).
    /// Derivative filter defaults to 1.0 (no filtering).
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral_limit: f64::MAX,
            derivative_filter_coeff: 1.0,
            integral: 0.0,
            prev_error: 0.0,
            filtered_derivative: 0.0,
        }
    }

    /// Clamp the integral accumulator to `[-limit, +limit]`.
    pub fn with_integral_limit(mut self, limit: f64) -> Self {
        self.integral_limit = limit;
        self
    }

    /// Set the IIR filter coefficient a for the derivative term.
    /// `a = 1.0` then no filtering; lower values smooth more aggressively.
    pub fn with_derivative_filter(mut self, alpha: f64) -> Self {
        self.derivative_filter_coeff = alpha.clamp(0.0, 1.0);
        self
    }

    /// Compute the next control output.
    ///
    /// - `setpoint`    - desired value
    /// - `measurement` - current measured value
    /// - `dt`          - time elapsed since last call, in seconds
    pub fn update(&mut self, setpoint: f64, measurement: f64, dt: f64) -> f64 {
        let error = setpoint - measurement;

        // Integral with anti-windup clamp
        self.integral = (self.integral + error * dt).clamp(-self.integral_limit, self.integral_limit);

        // Derivative with low-pass filter
        let raw_derivative = if dt > 0.0 {
            (error - self.prev_error) / dt
        } else {
            0.0
        };
        let a = self.derivative_filter_coeff;
        self.filtered_derivative = a * raw_derivative + (1.0 - a) * self.filtered_derivative;

        self.prev_error = error;

        self.kp * error + self.ki * self.integral + self.kd * self.filtered_derivative
    }

    /// Reset integral accumulator, previous error, and filtered derivative.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.filtered_derivative = 0.0;
    }

    /// Returns the last error (useful for settled-condition checks).
    pub fn last_error(&self) -> f64 {
        self.prev_error
    }
}
