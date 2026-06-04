//! **Scissors lift** subsystem.
//!
//! A scissors lift uses pairs of crossing arms linked at their midpoints.
//! Stacking multiple scissor stages multiplies the height per unit of
//! horizontal spread. Common in VRC for skyrise-style mechanisms.
//!
//! # Geometry
//!
//! ```text
//!  height = stages x arm_length_in x sin(arm_angle)
//! ```
//!
//! # Sensor placement options
//!
//! * `sensor_on_arm = false` (default) - sensor is the **drive motor**.
//!   The geometry converts motor degrees via `gear_ratio` to arm degrees.
//! * `sensor_on_arm = true` - a rotation sensor or potentiometer is mounted
//!   directly on one of the scissors arm pivots. In this case
//!   `gear_ratio` is ignored during conversion (set to `1.0`).
//!
//! # Example
//!
//! ```rust
//! use controls::subsystems::lift::*;
//!
//! let geo = ScissorsGeometry {
//!     arm_length_in: 10.0,
//!     stages:          3,
//!     gear_ratio:      7.0,
//!     sensor_on_arm: false,
//! };
//!
//! let pid = LiftPid::new(1200.0, 4.0, 100.0)
//!     .with_integral_limit(2.0)
//!     .with_settled_threshold(0.4);
//!
//! let presets = LiftPresets::new()
//!     .add("collapsed", 0.0)
//!     .add("low",       8.0)
//!     .add("mid",      16.0)
//!     .add("high",     24.0);
//!
//! let mut lift = ScissorsLift::with_averaged_motors(motors, geo, pid, presets);
//! lift.go_to_preset_name("high");
//!
//! while !lift.is_settled() {
//!     lift.update(0.010);
//! }
//! ```

use alloc::vec::Vec;
use log::{debug, warn};
use vexide::devices::smart::motor::Motor;

use super::{
    cascade::NullSensor,
    controller::LiftController,
    geometry::ScissorsGeometry,
    lift_impl::impl_lift,
    motor_sensor::MotorIndex,
    presets::LiftPresets,
    sensor::LiftSensor,
    source::SensorSource,
};

impl_lift!(
    name     = ScissorsLift,
    geo      = ScissorsGeometry,
    doc      = "Multi-stage scissors lift.",
    fallible = true,
);

impl<S: LiftSensor, C: LiftController> ScissorsLift<S, C> {
    /// Move the platform to `height_in` inches.
    ///
    /// Clamps to soft limits and validates against `stages × arm_length_in`.
    pub fn go_to_inches(&mut self, height_in: f64) {
        let geo_max = self.geometry.max_height_in();
        let clamped = height_in.clamp(self.min_height_in, self.max_height_in.min(geo_max));
        if self.geometry.inches_to_degrees(clamped).is_none() {
            warn!("ScissorsLift: {:.2} in unreachable - clamping to {:.2}", height_in, geo_max);
            self.target_in = geo_max;
        } else {
            self.target_in = clamped;
        }
        self.controller.reset();
        debug!("ScissorsLift → {:.3} in", self.target_in);
    }
}
