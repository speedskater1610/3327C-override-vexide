//! **Rack & Pinion** linear actuator.
//!
//! A circular pinion gear meshes with a linear rack. One full pinion rotation
//! moves the rack (and carriage) by `2pi * pinion_radius_in`.
//!
//! # Geometry
//!
//! ```text

//!  height = (motor_degrees / 360) * (2pi * pinion_radius_in) / gear_ratio
//! ```
//!
//! # VEX hardware reference
//!
//! | Part | Pinion radius |
//! |---|---|
//! | VEX rack & pinion kit (standard) | ~0.500 in |
//! | Custom 12t pinion on HS axle | ~0.480 in |
//!
//! # Example
//!
//! ```rust
//! use controls::subsystems::lift::*;
//!
//! let geo = RackPinionGeometry {
//!     pinion_radius_in: 0.500,
//!     gear_ratio:       1.0,   // direct drive - 600 RPM blue motor
//! };
//!
//! let pid = LiftPid::new(600.0, 0.5, 40.0).with_settled_threshold(0.15);
//!
//! let presets = LiftPresets::new()
//!     .add("retracted", 0.0)
//!     .add("mid",       4.0)
//!     .add("extended", 8.0);
//!
//! let mut actuator = RackPinionLift::with_motor_encoder(motors, geo, 0, pid, presets);
//! actuator.go_to_preset_name("extended");
//! ```

use alloc::vec::Vec;
use log::debug;
use vexide::devices::smart::motor::Motor;

use super::{
    cascade::NullSensor,
    controller::LiftController,
    geometry::RackPinionGeometry,
    lift_impl::impl_lift,
    motor_sensor::MotorIndex,
    presets::LiftPresets,
    sensor::LiftSensor,
    source::SensorSource,
};

impl_lift!(
    name     = RackPinionLift,
    geo      = RackPinionGeometry,
    doc      = "Rack-and-pinion linear actuator.",
    fallible = false,
);

impl<S: LiftSensor, C: LiftController> RackPinionLift<S, C> {
    /// Move to `height_in` inches. Clamped to soft limits.
    pub fn go_to_inches(&mut self, height_in: f64) {
        self.target_in = height_in.clamp(self.min_height_in, self.max_height_in);
        self.controller.reset();
        debug!("RackPinionLift → {:.3} in", self.target_in);
    }
}
