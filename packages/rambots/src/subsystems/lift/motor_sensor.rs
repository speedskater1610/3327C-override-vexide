//! Sensor adapter that reads position from the lift's own motor encoders.
//!
//! Because the lift struct owns the `Vec<Motor>`, we can't also hold a
//! `&Motor` borrow inside the same struct. [`MotorVecSensor`] stores only
//! the motor index and borrows the slice at read time via
//! [`MotorVecSensor::read`] - the lift calls this in `update()`.
//!
//! For most use cases you don't construct this directly: pass `MotorIndex(n)`
//! when building a lift and the lift will use it automatically.

/// Selects which motor index to use as the primary encoder.
/// Pass this as the `sensor` argument to the lift constructors when you
/// want to use the built-in motor encoder (no external sensor needed).
///
/// # Example
/// ```rust
/// // Use motor 0 as the position sensor
/// let lift = CascadeLift::new(motors, geo, MotorIndex(0), pid, presets);
/// ```
#[derive(Debug, Clone, Copy)]
pub struct MotorIndex(pub usize);

impl MotorIndex {
    /// Read position degrees from the motor at the stored index.
    pub fn read_degrees(self, motors: &[vexide::devices::smart::motor::Motor]) -> Option<f64> {
        motors
            .get(self.0)
            .and_then(|m| m.position().ok())
            .map(|p| p.as_degrees())
    }
}

/// An averaged reading across ALL motors in the lift.
/// More accurate than a single motor - noise cancels out.
#[derive(Debug, Clone, Copy)]
pub struct AveragedMotorSensor;

impl AveragedMotorSensor {
    pub fn read_degrees(motors: &[vexide::devices::smart::motor::Motor]) -> Option<f64> {
        let mut sum = 0.0_f64;
        let mut count = 0usize;
        for m in motors {
            if let Ok(p) = m.position() {
                sum += p.as_degrees();
                count += 1;
            }
        }
        if count == 0 { None } else { Some(sum / count as f64) }
    }
}
