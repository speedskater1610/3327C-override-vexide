//! Single-pivot **2-Bar** arm lift.
//!
//! The simplest rotating arm - one rigid link pivots around a fixed point.
//! Extremely common in VRC for mogos, claws, and intake deployments.
//!
//! # Geometry
//!
//! ```text
//!   height = base_height_in + arm_length_in x sin(arm_angle)
//! ```
//!
//! # Example
//!
//! ```rust
//! use controls::subsystems::lift::*;
//!
//! let geo = TwoBarGeometry {
//!     arm_length_in:  14.0,
//!     gear_ratio:      5.0,   // 5:1 reduction
//!     base_height_in:  4.0,   // pivot is 4 in off the floor
//!     zero_angle_deg: -45.0,  // arm rests 45deg below horizontal at encoder zero
//! };
//!
//! let pid = LiftPid::new(800.0, 1.5, 70.0).with_settled_threshold(0.2);
//!
//! let presets = LiftPresets::new()
//!     .add("stowed",  -6.0)   // arm down
//!     .add("floor",    4.0)   // picking from floor
//!     .add("low_goal", 12.0)
//!     .add("high_goal",18.0);
//!
//! let mut arm = TwoBarLift::with_averaged_motors(motors, geo, pid, presets);
//! arm.go_to_preset_name("high_goal");
//!
//! while !arm.is_settled() {
//!     arm.update(0.010);
//! }
//! ```

use alloc::vec::Vec;
use log::{debug, warn};
use vexide::devices::smart::motor::Motor;

use super::{
    cascade::NullSensor,
    controller::LiftController,
    geometry::TwoBarGeometry,
    lift_impl::impl_lift,
    motor_sensor::MotorIndex,
    presets::LiftPresets,
    sensor::LiftSensor,
    source::SensorSource,
};

impl_lift!(
    name     = TwoBarLift,
    geo      = TwoBarGeometry,
    doc      = "Single pivot 2 Bar arm lift",
    fallible = true,
);

// go_to_inches has geometry-specific validation so we add it separately
impl<S: LiftSensor, C: LiftController> TwoBarLift<S, C> {
    /// Move the arm so the end-effector reaches `height_in` inches above the floor.
    ///
    /// Clamps to soft limits and validates geometric reachability.
    pub fn go_to_inches(&mut self, height_in: f64) {
        let geo_max = self.geometry.max_height_in();
        let geo_min = self.geometry.min_height_in();
        let clamped = height_in.clamp(
            self.min_height_in.max(geo_min),
            self.max_height_in.min(geo_max),
        );
        if self.geometry.inches_to_degrees(clamped).is_none() {
            warn!("TwoBarLift: {:.2} in unreachable so clamping instead", height_in);
            self.target_in = geo_max;
        } else {
            self.target_in = clamped;
        }
        self.controller.reset();
        debug!("TwoBarLift -> {:.3} in", self.target_in);
    }
}
