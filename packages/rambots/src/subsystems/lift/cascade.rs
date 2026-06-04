//! Cascade (linear chain) lift subsystem.
//!
//! # Examples
//!
//! ## Motor encoder (no needed external sensor impl)
//! ```rust
//! use controls::subsystems::lift::*;
//!
//! let geo = CascadeGeometry { sprocket_radius_in: 1.096, gear_ratio: 5.0, stages: 2 };
//! let pid = LiftPid::new(900.0, 2.0, 80.0).with_settled_threshold(0.3);
//! let presets = LiftPresets::new()
//!     .add("lowered",    0.0)
//!     .add("score_low",  12.0)
//!     .add("score_high", 28.0);
//!
//! // MotorIndex(0) uses the encoder of the first motor in the vec
//! let mut lift = CascadeLift::new(motors, geo, MotorIndex(0), pid, presets);
//! lift.go_to_preset_name("score_high");
//! ```
//!
//! ## Averaged motor encoders
//! ```rust
//! let mut lift = CascadeLift::new(motors, geo, AveragedMotorSensor, pid, presets);
//! ```
//!
//! ## External potentiometer
//! ```rust
//! let pot = PotentiometerSensor::new(adi_port_a).with_zero_adc(820);
//! let mut lift = CascadeLift::new(motors, geo, pot, pid, presets);
//! ```
//!
//! ## Fused rotation sensors
//! ```rust
//! let sensor = FusedSensor::new(vec![
//!     RotationSensor::new(rotation_port_1),
//!     RotationSensor::new(rotation_port_2),
//! ]);
//! let mut lift = CascadeLift::new(motors, geo, sensor, pid, presets);
//! ```

use alloc::vec::Vec;

use log::{debug, warn};
use vexide::devices::smart::motor::Motor;

use super::{
    controller::LiftController,
    geometry::CascadeGeometry,
    motor_sensor::{AveragedMotorSensor, MotorIndex},
    presets::LiftPresets,
    sensor::LiftSensor,
    source::SensorSource,
};

/// A cascade lift. Generic over sensor type `S` and controller `C`.
///
/// Pass any of the following as the `sensor` argument:
/// * [`MotorIndex(n)`][MotorIndex]      built-in motor encoder
/// * [`AveragedMotorSensor`]            average of all motor encoders
/// * [`PotentiometerSensor`][super::sensor::PotentiometerSensor]
/// * [`RotationSensor`][super::sensor::RotationSensor]
/// * [`FusedSensor<S>`][super::sensor::FusedSensor]
pub struct CascadeLift<S: LiftSensor, C: LiftController> {
    motors: Vec<Motor>,
    geometry: CascadeGeometry,
    source: SensorSource<S>,
    controller: C,
    presets: LiftPresets,

    target_in: f64,
    min_height_in: f64,
    max_height_in: f64,
    hold_position: bool,
}

impl<S: LiftSensor, C: LiftController> CascadeLift<S, C> {
    /// Build a cascade lift from an external sensor implementing [`LiftSensor`].
    ///
    /// For motor encoder sources, use [`CascadeLift::with_motor_encoder`] or
    /// [`CascadeLift::with_averaged_motors`] instead.
    pub fn new(
        motors: Vec<Motor>,
        geometry: CascadeGeometry,
        sensor: S,
        controller: C,
        presets: LiftPresets,
    ) -> Self {
        Self::from_source(motors, geometry, SensorSource::External(sensor), controller, presets)
    }

    fn from_source(
        motors: Vec<Motor>,
        geometry: CascadeGeometry,
        source: SensorSource<S>,
        controller: C,
        presets: LiftPresets,
    ) -> Self {
        Self {
            motors,
            geometry,
            source,
            controller,
            presets,
            target_in: 0.0,
            min_height_in: 0.0,
            max_height_in: f64::MAX,
            hold_position: true,
        }
    }
}

// convience constructors for motor encoder srcs
// if the sensor is unreachable
//      then source is AllMotors/MotorIdx.
impl<C: LiftController> CascadeLift<core::convert::Infallible, C> {
    // These cant be expressed cleanly without a unit sensor type
    // See MotorEncoderLift type alias below
}

/// tyype alias: cascade lift driven entirely by motor encoders.
///
/// ```rust
/// let mut lift = MotorEncoderCascade::single(motors, geo, 0, pid, presets);
/// ```
pub type MotorEncoderCascade<C> = CascadeLift<NullSensor, C>;

/// A no op sensor used as a placeholder when the source is a motor encoder.
/// You never construct this directly.
pub struct NullSensor;
impl LiftSensor for NullSensor {
    fn position_degrees(&self) -> Option<f64> { None }
}

impl<C: LiftController> CascadeLift<NullSensor, C> {
    /// use the built in encoder of motor at `index` as the position source
    pub fn with_motor_encoder(
        motors: Vec<Motor>,
        geometry: CascadeGeometry,
        motor_index: usize,
        controller: C,
        presets: LiftPresets,
    ) -> Self {
        Self::from_source(
            motors,
            geometry,
            SensorSource::MotorIdx(MotorIndex(motor_index)),
            controller,
            presets,
        )
    }

    /// average all motor encoders for position feedback
    pub fn with_averaged_motors(
        motors: Vec<Motor>,
        geometry: CascadeGeometry,
        controller: C,
        presets: LiftPresets,
    ) -> Self {
        Self::from_source(
            motors,
            geometry,
            SensorSource::AllMotors,
            controller,
            presets,
        )
    }
}

// config
impl<S: LiftSensor, C: LiftController> CascadeLift<S, C> {
    /// soft lower limit in inches (with a default of 0.0)
    pub fn with_min_height(mut self, in_: f64) -> Self {
        self.min_height_in = in_;
        self
    }

    /// soft upper limit in inches.
    pub fn with_max_height(mut self, in_: f64) -> Self {
        self.max_height_in = in_;
        return self;
    }

    /// `false` motors coast once settled
    /// `true` hold (default).
    pub fn with_hold_position(mut self, hold: bool) -> Self {
        self.hold_position = hold;
        return self;
    }
}

// targeting heights
impl<S: LiftSensor, C: LiftController> CascadeLift<S, C> {
    /// move to an absolute height in **inches**. This is Clamped to soft limits.
    pub fn go_to_inches(&mut self, height_in: f64) {
        let clamped = height_in.clamp(self.min_height_in, self.max_height_in);
        self.target_in = clamped;
        self.controller.reset();
        debug!("CascadeLift -> {:.3} in", clamped);
    }

    /// move to a named preset
    pub fn go_to_preset_name(&mut self, name: &str) {
        match self.presets.get(name) {
            Some(h) => self.go_to_inches(h),
            None => warn!("CascadeLift: unknown preset '{}'", name),
        }
    }

    /// move to a preset by insertion index
    pub fn go_to_preset_index(&mut self, index: usize) {
        match self.presets.iter().nth(index) {
            Some((name, h)) => {
                debug!("CascadeLift preset[{}] '{}' -> {:.3} in", index, name, h);
                self.go_to_inches(h);
            }
            None => warn!("CascadeLift: preset index {} out of range", index),
        }
    }

    /// cycle to the next taller preset wraps back to the lowest
    pub fn go_to_next_preset(&mut self) {
        let current = self.current_height_inches().unwrap_or(self.target_in);
        let next = self.presets.iter()
            .filter(|(_, h)| *h > current + 0.5)
            .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap());

        match next {
            Some((name, h)) => { 
                debug!("CascadeLift (up) '{}'  {:.2} in", name, h); self.go_to_inches(h);
            }
            None => {
                if let Some((name, h)) = self.presets.iter().next() {
                    debug!("CascadeLift wrap (up) '{}' {:.2} in", name, h);
                    self.go_to_inches(h);
                }
            }
        }
    }

    /// cycle to the next shorter preset wraps back to the highest
    pub fn go_to_prev_preset(&mut self) {
        let current = self.current_height_inches().unwrap_or(self.target_in);
        let prev = self.presets.iter()
            .filter(|(_, h)| *h < current - 0.5)
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap());

        match prev {
            Some((name, h)) => {
                debug!("CascadeLift (down) '{}'  {:.2} in", name, h); self.go_to_inches(h);
            }
            None => {
                if let Some((name, h)) = self.presets.iter().last() {
                    debug!("CascadeLift wrap (down) '{}' {:.2} in", name, h);
                    self.go_to_inches(h);
                }
            }
        }
    }
}

// Control loop
impl<S: LiftSensor, C: LiftController> CascadeLift<S, C> {
    /// run one control iteration
    ///
    /// call every loop tick with the elapsed time in seconds (`dt`)
    /// typical: `lift.update(0.010)` inside a 10 ms loop.
    pub fn update(&mut self, dt: f64) {
        let current = match self.source.position_degrees(&self.motors) {
            Some(deg) => self.geometry.degrees_to_inches(deg),
            None => return,
        };

        let voltage = if self.is_settled() && !self.hold_position {
            0.0
        } else {
            self.controller.compute(self.target_in, current, dt)
        };

        for m in &mut self.motors {
            let _ = m.set_voltage(voltage);
        }
    }
}

// status
impl<S: LiftSensor, C: LiftController> CascadeLift<S, C> {
    /// current height in inches or `None` if the sensor isnt aviable
    pub fn current_height_inches(&self) -> Option<f64> {
        let deg = self.source.position_degrees(&self.motors)?;
        Some(self.geometry.degrees_to_inches(deg))
    }

    /// target height in inches
    pub fn target_height_inches(&self) -> f64 {
        self.target_in
    }

    /// `true` when settled at the target
    pub fn is_settled(&self) -> bool {
        self.controller.is_settled()
    }

    /// height error in inches (positive = below target)
    pub fn error_inches(&self) -> Option<f64> {
        Some(self.target_in - self.current_height_inches()?)
    }

    /// name of the nearest defined preset to the current position
    pub fn nearest_preset_name(&self) -> Option<&str> {
        let current = self.current_height_inches()?;
        self.presets.nearest(current)
    }
}

// manually appling heights
impl<S: LiftSensor, C: LiftController> CascadeLift<S, C> {
    /// apply raw voltage fraction `[-1.0, 1.0]` directly so it bypasses the controller
    pub fn set_manual(&mut self, fraction: f64) {
        let v = fraction.clamp(-1.0, 1.0) * 12_000.0;
        for m in &mut self.motors {
            let _ = m.set_voltage(v);
        }
    }

    /// cut all motor power immediately
    pub fn stop(&mut self) {
        for m in &mut self.motors {
            let _ = m.set_voltage(0.0);
        }
    }

    /// zero the sensor at the current physical position
    pub fn zero_sensor(&mut self) {
        self.source.reset(&mut self.motors);
        self.target_in = 0.0;
        self.controller.reset();
        debug!("CascadeLift sensor has been zeroed");
    }
}