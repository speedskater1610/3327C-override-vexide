//! **Continuous chain** (vertical chain-drive) lift.
//!
//! A single loop of chain runs over a top and bottom sprocket. The carriage
//! is clamped to one run of the chain, giving it the full vertical travel of
//! the mechanism. Mathematically identical to a single-stage cascade, but
//! kept as a distinct type for clarity.
//!
//! # Geometry
//!
//! ```text
//!  height = (motor_revs / gear_ratio) * 2pi * sprocket_radius_in
//! ```
//!
//! # Example
//!
//! ```rust
//! use controls::subsystems::lift::*;
//!
//! let geo = ContinuousChainGeometry {
//!     sprocket_radius_in: 1.096,  // 20-tooth VEX sprocket
//!     gear_ratio:          1.0,   // direct drive
//! };
//!
//! let pid = LiftPid::new(700.0, 1.0, 60.0).with_settled_threshold(0.5);
//!
//! let presets = LiftPresets::new()
//!     .add("bottom", 0.0)
//!     .add("mid",    12.0)
//!     .add("top",    24.0);
//!
//! // Use motor 0 as encoder
//! let mut lift = ContinuousChainLift::with_motor_encoder(motors, geo, 0, pid, presets);
//! lift.go_to_preset_name("top");
//! ```

use alloc::vec::Vec;
use log::debug;
use vexide::devices::smart::motor::Motor;

use super::{
    cascade::NullSensor,
    controller::LiftController,
    geometry::ContinuousChainGeometry,
    lift_impl::impl_lift,
    motor_sensor::MotorIndex,
    presets::LiftPresets,
    sensor::LiftSensor,
    source::SensorSource,
};

impl_lift!(
    name     = ContinuousChainLift,
    geo      = ContinuousChainGeometry,
    doc      = "Continuous chain (vertical chain-drive) lift.",
    fallible = false,
);

impl<S: LiftSensor, C: LiftController> ContinuousChainLift<S, C> {
    /// Move to `height_in` inches. Clamped to soft limits.
    pub fn go_to_inches(&mut self, height_in: f64) {
        self.target_in = height_in.clamp(self.min_height_in, self.max_height_in);
        self.controller.reset();
        debug!("ContinuousChainLift -> {:.3} in", self.target_in);
    }
}
