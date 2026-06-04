//! Sensor abstractions for lift position feedback.
//!
//! The [`LiftSensor`] trait lets the lift framework accept any feedback source:
//! built-in motor encoders, ADI potentiometers, rotation sensors, or a
//! fused average of several sensors.
//!
//! All sensor implementations return position in **raw degrees** internally.
//! The lift geometry layer converts those degrees to inches for the public API.

use alloc::vec::Vec;
use vexide::{
    adi::{potentiometer::AdiPotentiometer, AdiPort},
    devices::smart::{
        motor::Motor,
        rotation::RotationSensor as VexRotationSensor,
    },
};

// Trait

/// Position feedback source for a lift.
///
/// Implementors report position in **degrees** (one full rotation = 360deg).
/// The lift framework converts to inches using the geometric model.
///
/// Return `None` when the sensor is disconnected or the reading is invalid.
pub trait LiftSensor {
    /// Current sensor position in degrees.
    fn position_degrees(&self) -> Option<f64>;

    /// Reset / zero the sensor at the current position.
    fn reset(&mut self) {}
}


// Motor encoder

/// Uses the built-in encoder of a smart motor.
///
/// This is the most convenient sensor no extra hardware needed.
/// For better accuracy, use multiple motors and [`FusedSensor`].
pub struct MotorEncoder {
    /// Shared reference so the motor is also usable for driving.
    /// We only need a borrow for reading, but vexide motors are owned,
    /// so we store an index into the owning motors Vec.
    ///
    /// Implementation note: the lift structs pass the motor slice in when
    /// calling `position_degrees`, so we only need the index here.
    pub(crate) motor_index: usize,
}

impl MotorEncoder {
    /// Use motor at `index` inside the lift's motors `Vec`.
    pub fn new(motor_index: usize) -> Self {
        Self { motor_index }
    }
}

/// A self contained wrapper when you want to poll a single motor independently.
pub struct StandaloneMotorEncoder<'a> {
    motor: &'a Motor,
}

impl<'a> StandaloneMotorEncoder<'a> {
    pub fn new(motor: &'a Motor) -> Self {
        Self { motor }
    }
}

impl<'a> LiftSensor for StandaloneMotorEncoder<'a> {
    fn position_degrees(&self) -> Option<f64> {
        self.motor.position().ok().map(|p| p.as_degrees())
    }
}


// ADI Potentiometer

/// Wraps a ADI potentiometer and maps its 0-4095 ADC range to degrees.
///
/// The sensor sweeps ~=250deg of physical rotation. At full CCW the pot reads
/// 0; at full CW it reads 4095.
pub struct PotentiometerSensor {
    pot: AdiPotentiometer,
    /// Degrees of physical travel corresponding to the full ADC range (0-4095).
    /// Default: 250deg.
    full_scale_degrees: f64,
    /// ADC value at the lift's lowest physical position (zero inches).
    zero_adc: u16,
    /// Whether increasing ADC -> increasing height (`true`) or the reverse.
    inverted: bool,
}

impl PotentiometerSensor {
    const ADC_MAX: f64 = 4095.0;
    const DEFAULT_FULL_SCALE: f64 = 250.0;

    pub fn new(port: AdiPort) -> Self {
        Self {
            pot: AdiPotentiometer::new(port),
            full_scale_degrees: Self::DEFAULT_FULL_SCALE,
            zero_adc: 0,
            inverted: false,
        }
    }

    /// Override the default 250deg full-scale sweep.
    pub fn with_full_scale_degrees(mut self, deg: f64) -> Self {
        self.full_scale_degrees = deg;
        self
    }

    /// Set the ADC value that corresponds to the lift's lowest position.
    /// Call once during calibration.
    pub fn with_zero_adc(mut self, adc: u16) -> Self {
        self.zero_adc = adc;
        self
    }

    /// Flip the sign so increasing ADC maps to decreasing position.
    pub fn inverted(mut self) -> Self {
        self.inverted = true;
        self
    }
}

impl LiftSensor for PotentiometerSensor {
    fn position_degrees(&self) -> Option<f64> {
        let raw = self.pot.value().ok()? as f64;
        let zero = self.zero_adc as f64;
        let offset = if self.inverted {
            Self::ADC_MAX - raw - (Self::ADC_MAX - zero)
        } else {
            raw - zero
        };
        Some(offset / Self::ADC_MAX * self.full_scale_degrees)
    }
}

// Rotation Sensor

/// Wraps the   rotation sensor (smart port).
pub struct RotationSensor {
    inner: VexRotationSensor,
    inverted: bool,
}

impl RotationSensor {
    pub fn new(sensor: VexRotationSensor) -> Self {
        Self { inner: sensor, inverted: false }
    }

    pub fn inverted(mut self) -> Self {
        self.inverted = true;
        self
    }
}

impl LiftSensor for RotationSensor {
    fn position_degrees(&self) -> Option<f64> {
        let deg = self.inner.position().ok()?.as_degrees();
        Some(if self.inverted { -deg } else { deg })
    }

    fn reset(&mut self) {
        let _ = self.inner.reset_position();
    }
}

// Sensor fusion

/// Combines multiple [`LiftSensor`]s by averaging their readings.
///
/// Sensors that return `None` (disconnected / error) are silently skipped.
/// Returns `None` only if **all** sensors fail.
pub struct FusedSensor<S: LiftSensor> {
    sensors: Vec<S>,
}

impl<S: LiftSensor> FusedSensor<S> {
    pub fn new(sensors: Vec<S>) -> Self {
        Self { sensors }
    }
}

impl<S: LiftSensor> LiftSensor for FusedSensor<S> {
    fn position_degrees(&self) -> Option<f64> {
        let mut sum = 0.0_f64;
        let mut count = 0usize;

        for s in &self.sensors {
            if let Some(deg) = s.position_degrees() {
                sum += deg;
                count += 1;
            }
        }

        if count == 0 { None } else { Some(sum / count as f64) }
    }

    fn reset(&mut self) {
        for s in &mut self.sensors {
            s.reset();
        }
    }
}
