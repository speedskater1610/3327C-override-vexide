//! **6-Bar** parallel linkage lift.
//!
//! A 6-bar is essentially two 4-bar linkages sharing a common rocker, giving
//! more vertical travel than a simple 4-bar while keeping the end-effector
//! roughly parallel to the ground. Very common in VRC Nothing But Net and
//! Turning Point for ball-scoring mechanisms.
//!
//! # Geometry model
//!
//! The exact kinematics of a 6-bar depend on the specific link lengths.
//! This implementation uses the **simplified equal-link model**: both
//! linkage stages have the same arm length, and the height contribution of
//! each stage is additive:
//!
//! ```text
//!  height = base_height_in
//!         + lower_arm_in x sin(lower_angle)
//!         + upper_arm_in x sin(upper_angle)
//!
//!  where  upper_angle ~= lower_angle x coupling_ratio
//! ```
//!
//! `coupling_ratio` captures how the upper stage amplifies the lower stage's
//! motion. For a symmetric equal-link 6-bar it is `1.0` (both stages rotate
//! equally). For an over-driven design it can be `> 1.0`.
//!
//! Because of the simplified model the geometry still returns `None` if the
//! requested height is geometrically unreachable.
//!
//! # Example
//!
//! ```rust
//! use controls::subsystems::lift::*;
//!
//! let geo = SixBarGeometry {
//!     lower_arm_in:    9.0,
//!     upper_arm_in:    9.0,
//!     coupling_ratio:  1.0,
//!     gear_ratio:      7.0,
//!     base_height_in:  4.0,
//! };
//!
//! let pid = LiftPid::new(1050.0, 2.5, 85.0)
//!     .with_derivative_filter(0.88)
//!     .with_settled_threshold(0.3);
//!
//! let presets = LiftPresets::new()
//!     .add("lowered",    4.0)
//!     .add("score_low",  14.0)
//!     .add("score_high", 22.0);
//!
//! let mut lift = SixBarLift::with_averaged_motors(motors, geo, pid, presets);
//! lift.go_to_preset_name("score_high");
//! ```

use alloc::vec::Vec;
use log::{debug, warn};
use vexide::devices::smart::motor::Motor;

use super::{
    cascade::NullSensor,
    controller::LiftController,
    geometry::SixBarGeometry,
    lift_impl::impl_lift,
    motor_sensor::MotorIndex,
    presets::LiftPresets,
    sensor::LiftSensor,
    source::SensorSource,
};

impl_lift!(
    name     = SixBarLift,
    geo      = SixBarGeometry,
    doc      = "6-bar parallel linkage lift.",
    fallible = true,
);

impl<S: LiftSensor, C: LiftController> SixBarLift<S, C> {
    /// Move the end-effector to `height_in` inches.
    pub fn go_to_inches(&mut self, height_in: f64) {
        let geo_max = self.geometry.max_height_in();
        let clamped = height_in.clamp(self.min_height_in, self.max_height_in.min(geo_max));
        if self.geometry.inches_to_degrees(clamped).is_none() {
            warn!("SixBarLift: {:.2} in unreachable - clamping to {:.2}", height_in, geo_max);
            self.target_in = geo_max;
        } else {
            self.target_in = clamped;
        }
        self.controller.reset();
        debug!("SixBarLift -> {:.3} in", self.target_in);
    }
}
