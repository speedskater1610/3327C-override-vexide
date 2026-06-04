//! DR4B lift subsystem.
//!
//! A DR4B stacks two 4-bar linkages so the end stays vertical
//! throughout the full range. Height is non linear with respect to motor
//! angle - [`Dr4bGeometry`] handles the trigonometric conversion.
//!
//! Accepts the same sensor sources as [`CascadeLift`]:
//! * [`MotorIndex(n)`][super::motor_sensor::MotorIndex]
//! * [`AveragedMotorSensor`][super::motor_sensor::AveragedMotorSensor]
//! * [`PotentiometerSensor`][super::sensor::PotentiometerSensor]
//! * [`RotationSensor`][super::sensor::RotationSensor]
//! * [`FusedSensor<S>`][super::sensor::FusedSensor]
//!
//! # Example
//!
//! ```rust
//! use controls::subsystems::lift::*;
//!
//! let geo = Dr4bGeometry { 
//!     arm_length_in: 11.5, gear_ratio: 7.0, base_height_in: 5.0
//! };
//! let pid = LiftPid::new(1100.0, 3.0, 90.0)
//!     .with_derivative_filter(0.85)
//!     .with_settled_threshold(0.25);
//!
//! let presets = LiftPresets::new()
//!     .add("lowered",    5.0)
//!     .add("score_low",  14.0)
//!     .add("score_mid",  20.0)
//!     .add("score_high", 26.0);
//!
//! // Averaged motor encoders - no external sensor needed
//! let mut lift = Dr4bLift::with_averaged_motors(motors, geo, pid, presets);
//! lift.go_to_preset_name("score_high");
//!
//! while !lift.is_settled() {
//!     lift.update(0.010);
//!     sleep(Duration::from_millis(10)).await;
//! }
//! ```

use alloc::vec::Vec;

use log::{debug, warn};
use vexide::devices::smart::motor::Motor;

use super::{
    controller::LiftController,
    geometry::Dr4bGeometry,
    motor_sensor::MotorIndex,
    presets::LiftPresets,
    sensor::LiftSensor,
    source::SensorSource,
};

/// a DR4B lift. Generic over sensor `S` and controller `C`
pub struct Dr4bLift<S: LiftSensor, C: LiftController> {
    motors: Vec<Motor>,
    geometry: Dr4bGeometry,
    source: SensorSource<S>,
    controller: C,
    presets: LiftPresets,

    target_in: f64,
    min_height_in: f64,
    max_height_in: f64,
    hold_position: bool,
}

// constructors
impl<S: LiftSensor, C: LiftController> Dr4bLift<S, C> {
    /// Build with any external sensor implementing [`LiftSensor`]
    pub fn new(
        motors: Vec<Motor>,
        geometry: Dr4bGeometry,
        sensor: S,
        controller: C,
        presets: LiftPresets,
    ) -> Self {
        Self::from_source(motors, geometry, SensorSource::External(sensor), controller, presets)
    }

    fn from_source(
        motors: Vec<Motor>,
        geometry: Dr4bGeometry,
        source: SensorSource<S>,
        controller: C,
        presets: LiftPresets,
    ) -> Self {
        let max = geometry.max_height_in();
        Self {
            motors,
            geometry,
            source,
            controller,
            presets,
            target_in: 0.0,
            min_height_in: 0.0,
            max_height_in: max,
            hold_position: true,
        }
    }
}

/// placeholder for motor encoder only lift types
pub struct NullSensor;
impl LiftSensor for NullSensor {
    fn position_degrees(&self) -> Option<f64> { None }
}

/// type alias: DR4B driven entirely by motor encoders
pub type MotorEncoderDr4b<C> = Dr4bLift<NullSensor, C>;

impl<C: LiftController> Dr4bLift<NullSensor, C> {
    /// use the built in encoder of motor `motor_index` as position feedback for thee lift
    pub fn with_motor_encoder(
        motors: Vec<Motor>,
        geometry: Dr4bGeometry,
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

    /// avg all motor encoders for position feedback
    pub fn with_averaged_motors(
        motors: Vec<Motor>,
        geometry: Dr4bGeometry,
        controller: C,
        presets: LiftPresets,
    ) -> Self {
        Self::from_source(motors, geometry, SensorSource::AllMotors, controller, presets)
    }
}

// config
impl<S: LiftSensor, C: LiftController> Dr4bLift<S, C> {
    pub fn with_min_height(mut self, in_: f64) -> Self { self.min_height_in = in_; self }
    pub fn with_max_height(mut self, in_: f64) -> Self { self.max_height_in = in_; self }
    pub fn with_hold_position(mut self, hold: bool) -> Self { self.hold_position = hold; self }
}

// targeting points
impl<S: LiftSensor, C: LiftController> Dr4bLift<S, C> {
    /// Move to `height_in` inches above the floor.
    ///
    /// Automatically validates geometric reachability and clamps to soft limits.
    pub fn go_to_inches(&mut self, height_in: f64) {
        let geo_max = self.geometry.max_height_in();
        let clamped = height_in.clamp(self.min_height_in, self.max_height_in.min(geo_max));

        if self.geometry.inches_to_degrees(clamped).is_none() {
            warn!("Dr4bLift: {:.2} in unreachable so instead clamping to {:.2}", height_in, geo_max);
            self.target_in = geo_max;
        } else {
            self.target_in = clamped;
        }

        self.controller.reset();
        debug!("Dr4bLift -> {:.3} in", self.target_in);
    }

    /// Move to a named preset.
    pub fn go_to_preset_name(&mut self, name: &str) {
        match self.presets.get(name) {
            Some(h) => self.go_to_inches(h),
            None => warn!("Dr4bLift: unknown preset '{}'", name),
        }
    }

    /// Move to a preset by insertion index.
    pub fn go_to_preset_index(&mut self, index: usize) {
        match self.presets.iter().nth(index) {
            Some((name, h)) => {
                debug!("Dr4bLift preset[{}] '{}' -> {:.3} in", index, name, h);
                self.go_to_inches(h);
            }
            None => warn!("Dr4bLift: preset index {} out of range", index),
        }
    }

    /// cycle up to the next taller preset; wraps to the lowest
    pub fn go_to_next_preset(&mut self) {
        let current = self.current_height_inches().unwrap_or(self.target_in);
        let next = self.presets.iter()
            .filter(|(_, h)| *h > current + 0.5)
            .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap());

        match next {
            Some((name, h)) => { debug!("Dr4bLift (up) '{}' {:.2} in", name, h); self.go_to_inches(h); }
            None => {
                if let Some((name, h)) = self.presets.iter().next() {
                    debug!("Dr4bLift wrap (up) '{}' {:.2} in", name, h);
                    self.go_to_inches(h);
                }
            }
        }
    }

    /// cycle down to the next shorter preset; wraps to the highest
    pub fn go_to_prev_preset(&mut self) {
        let current = self.current_height_inches().unwrap_or(self.target_in);
        let prev = self.presets.iter()
            .filter(|(_, h)| *h < current - 0.5)
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap());

        match prev {
            Some((name, h)) => { debug!("Dr4bLift (down) '{}' {:.2} in", name, h); self.go_to_inches(h); }
            None => {
                if let Some((name, h)) = self.presets.iter().last() {
                    debug!("Dr4bLift wrap (down) '{}' {:.2} in", name, h);
                    self.go_to_inches(h);
                }
            }
        }
    }
}

// controls loop 
impl<S: LiftSensor, C: LiftController> Dr4bLift<S, C> {
    /// Run one control iteration. Call every 10 ms (pass `dt = 0.010`).
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
impl<S: LiftSensor, C: LiftController> Dr4bLift<S, C> {
    pub fn current_height_inches(&self) -> Option<f64> {
        let deg = self.source.position_degrees(&self.motors)?;
        Some(self.geometry.degrees_to_inches(deg))
    }

    pub fn target_height_inches(&self) -> f64 { self.target_in }
    pub fn is_settled(&self) -> bool { self.controller.is_settled() }

    /// positive = below target
    pub fn error_inches(&self) -> Option<f64> {
        Some(self.target_in - self.current_height_inches()?)
    }

    /// geometric maximum height this lift can reach
    pub fn max_reachable_inches(&self) -> f64 { self.geometry.max_height_in() }

    /// name of the nearest preset to the current position
    pub fn nearest_preset_name(&self) -> Option<&str> {
        let current = self.current_height_inches()?;
        self.presets.nearest(current)
    }
}

// maanually update the lift
impl<S: LiftSensor, C: LiftController> Dr4bLift<S, C> {
    /// Raw voltage fraction `[-1.0, 1.0]` bypasses controller.
    pub fn set_manual(&mut self, fraction: f64) {
        let v = fraction.clamp(-1.0, 1.0) * 12_000.0;
        for m in &mut self.motors { let _ = m.set_voltage(v); }
    }

    /// Cut all motor power.
    pub fn stop(&mut self) {
        for m in &mut self.motors { let _ = m.set_voltage(0.0); }
    }

    /// Zero sensor at the current hard-stop. After this, `current_height`
    /// will read as `base_height_in`.
    pub fn zero_sensor(&mut self) {
        self.source.reset(&mut self.motors);
        self.target_in = self.geometry.base_height_in;
        self.controller.reset();
        debug!("Dr4bLift zeroed; base = {:.2} in", self.geometry.base_height_in);
    }
}
