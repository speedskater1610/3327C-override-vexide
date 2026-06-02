//! [`SensorSource`] - a unified enum the lift structs store internally so they
//! can hold either an external sensor *or* a motor-encoder selector without
//! any lifetime issues.
//!
//! You never construct this directly - the lift builders accept your sensor
//! value and wrap it automatically.

use alloc::vec::Vec;

use vexide::devices::smart::motor::Motor;

use super::{
    motor_sensor::{AveragedMotorSensor, MotorIndex},
    sensor::LiftSensor,
};

/// Everything a lift can use as a position source.
pub enum SensorSource<S: LiftSensor> {
    /// Use the built-in encoder of one specific motor.
    MotorIdx(MotorIndex),
    /// Average all motor encoders.
    AllMotors,
    /// Any external sensor implementing [`LiftSensor`].
    External(S),
}

impl<S: LiftSensor> SensorSource<S> {
    /// Read position degrees, borrowing the motor slice when needed.
    pub fn position_degrees(&self, motors: &[Motor]) -> Option<f64> {
        match self {
            Self::MotorIdx(idx) => idx.read_degrees(motors),
            Self::AllMotors => AveragedMotorSensor::read_degrees(motors),
            Self::External(s) => s.position_degrees(),
        }
    }

    pub fn reset(&mut self, motors: &mut Vec<Motor>) {
        match self {
            Self::External(s) => s.reset(),
            // Reset motor encoders - call tare_position on each
            Self::MotorIdx(idx) => {
                if let Some(m) = motors.get_mut(idx.0) {
                    let _ = m.tare_position();
                }
            }
            Self::AllMotors => {
                for m in motors.iter_mut() {
                    let _ = m.tare_position();
                }
            }
        }
    }
}

// Allow From conversions so callers can pass any of the three types naturally.

impl<S: LiftSensor> From<MotorIndex> for SensorSource<S> {
    fn from(v: MotorIndex) -> Self { Self::MotorIdx(v) }
}

impl<S: LiftSensor> From<AveragedMotorSensor> for SensorSource<S> {
    fn from(_: AveragedMotorSensor) -> Self { Self::AllMotors }
}
