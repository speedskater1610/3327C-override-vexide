//! Lift feedback controllers.
//!
//! The [`LiftController`] trait lets you swap between different control
//! strategies without changing the lift struct.
//!
//! # Provided controllers
//!
//! | Type | Best for |
//! |---|---|
//! | [`LiftPid`] | Smooth, accurate positioning; requires tuning |
//! | [`LiftBangBang`] | Simple mechanisms; no tuning needed |

// Trait

/// A feedback controller that maps `(target_in, current_in, dt) -> voltage`
///
/// All units are in **inches** (no raw encoder values here).
pub trait LiftController {
    /// Compute the motor voltage (mV, clamped to + or - 12000) for this iteration
    ///
    /// - `target_in`  - desired height in inches
    /// - `current_in` - current measured height in inches
    /// - `dt`         - elapsed time since last call, in seconds
    fn compute(&mut self, target_in: f64, current_in: f64, dt: f64) -> f64;

    /// Returns `true` when the lift is considered settled at the target.
    fn is_settled(&self) -> bool;

    /// Reset controller state (called when a new target is set)
    fn reset(&mut self);
}

// PID

/// PID controller tuned for lift heights in inches
///
/// The output is a voltage in millivolts (+ or - 12 000)
/// Gains are expressed in `mV / in`, `mV / (in*s)`, `mV*s / in`
///
/// # Tuning guide
///
/// 1. Start with `ki = 0`, `kd = 0`. Increase `kp` until the lift reaches
///    the target without oscillating
///
/// 2. Add `kd` to dampen overshoot
///
/// 3. Add `ki` only if the lift consistently falls short; keep it small and
///    always pair with [`LiftPid::with_integral_limit`]
#[derive(Debug, Clone)]
pub struct LiftPid {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,

    integral_limit_in: f64,
    derivative_filter: f64,
    settled_threshold_in: f64,
    settled_velocity_threshold: f64,

    integral: f64,
    prev_error: f64,
    filtered_deriv: f64,
    settled: bool,
}

impl LiftPid {
    /// Create a new PID controller
    ///
    /// `kp`, `ki`, `kd` are in units of `mV / in`, `mV / (in*s)`, `mV*s / in`
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral_limit_in: f64::MAX,
            derivative_filter: 1.0,
            settled_threshold_in: 0.25,
            settled_velocity_threshold: 0.5,
            integral: 0.0,
            prev_error: 0.0,
            filtered_deriv: 0.0,
            settled: false,
        }
    }

    /// clamp the integral term to + or - `limit` in
    pub fn with_integral_limit(mut self, limit_in: f64) -> Self {
        self.integral_limit_in = limit_in;
        self
    }

    /// IIR filter for the d term
    /// `1.0` = no filtering; `0.0` = completely filtered out.
    pub fn with_derivative_filter(mut self, alpha: f64) -> Self {
        self.derivative_filter = alpha.clamp(0.0, 1.0);
        self
    }

    /// Distance from the target (inches) below which the lift is "settled"
    /// Default: 0.25 in
    pub fn with_settled_threshold(mut self, inches: f64) -> Self {
        self.settled_threshold_in = inches;
        self
    }

    /// minimum velocity (in/s) above which the lift is NOT yet settled
    /// Prevents declaring settled while still moving    
    /// Default: 0.5 in/s
    pub fn with_settled_velocity_threshold(mut self, in_per_s: f64) -> Self {
        self.settled_velocity_threshold = in_per_s;
        self
    }
}

impl LiftController for LiftPid {
    fn compute(&mut self, target_in: f64, current_in: f64, dt: f64) -> f64 {
        let error = target_in - current_in;

        self.integral = (self.integral + error * dt)
            .clamp(-self.integral_limit_in, self.integral_limit_in);

        let raw_deriv = if dt > 1e-6 {
            (error - self.prev_error) / dt
        } else {
            0.0
        };

        let a = self.derivative_filter;
        self.filtered_deriv = a * raw_deriv + (1.0 - a) * self.filtered_deriv;

        self.prev_error = error;

        // settled check: small error AND small velocity
        let velocity = raw_deriv.abs();
        self.settled = error.abs() < self.settled_threshold_in
            && velocity < self.settled_velocity_threshold;

        let output = self.kp * error
            + self.ki * self.integral
            + self.kd * self.filtered_deriv;

        output.clamp(-12_000.0, 12_000.0)
    }

    fn is_settled(&self) -> bool {
        self.settled
    }

    fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.filtered_deriv = 0.0;
        self.settled = false;
    }
}

// bang bang

/// Simple on/off controller.
/// this should be good for fast mechanisms where precise
/// positioning is less important than speed
#[derive(Debug, Clone)]
pub struct LiftBangBang {
    /// voltage applied when below the target (mV)
    pub up_voltage: f64,
    /// voltage applied when above the target (mV)
    pub down_voltage: f64,
    /// Dead band half width (in) within this range output is 0
    pub tolerance_in: f64,
}

impl LiftBangBang {
    pub fn new(up_voltage: f64, down_voltage: f64, tolerance_in: f64) -> Self {
        Self { up_voltage, down_voltage, tolerance_in }
    }
}

impl LiftController for LiftBangBang {
    fn compute(&mut self, target_in: f64, current_in: f64, _dt: f64) -> f64 {
        let error = target_in - current_in;
        if error > self.tolerance_in {
            self.up_voltage.clamp(-12_000.0, 12_000.0)
        } else if error < -self.tolerance_in {
            (-self.down_voltage).clamp(-12_000.0, 12_000.0)
        } else {
            0.0
        }
    }

    fn is_settled(&self) -> bool {
        // bang bang declares settled from within compute track externally
        false // overridden by lift struct using error check
    }

    fn reset(&mut self) {}
}