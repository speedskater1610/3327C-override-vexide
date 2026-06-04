//! **Belt driven linear slide** subsystem.
//!
//! a toothed timing belt runs over two pulleys. 
//! 
//! The carriage is clamped to one run of the belt giving fast and lightweight linear travel
//!
//! # Geometry
//!
//! ```text
//!  top pulley ---------------------- bottom pulley (motor)
//!             ->->->->->->->->->->->   (carriage side)
//!             [CARRIAGE]
//!             <-<-<-<-<-<-<-<-<-<-<-   (return side)
//!
//!  height = (motor_revs / gear_ratio) * 2(pi) * pulley_radius_in
//! ```
//!
//! # Example
//!
//! ```rust
//! use controls::subsystems::lift::*;
//!
//! let geo = BeltSlideGeometry {
//!     pulley_radius_in: 0.875,   // pulleys radius
//!     gear_ratio:       3.0,     // 3:1 belt reduction
//! };
//!
//! let pid = LiftPid::new(800.0, 1.5, 65.0)
//!     .with_settled_threshold(0.2)
//!     .with_derivative_filter(0.9);
//!
//! let presets = LiftPresets::new()
//!     .add("home",     0.0)
//!     .add("mid",     10.0)
//!     .add("top",     20.0);
//!
//! let mut slide = BeltSlideLift::with_averaged_motors(motors, geo, pid, presets);
//! slide.go_to_preset_name("top");
//!
//! while !slide.is_settled() {
//!     slide.update(0.010);
//! }
//! ```

use alloc::vec::Vec;
use log::debug;
use vexide::devices::smart::motor::Motor;

use super::{
    cascade::NullSensor,
    controller::LiftController,
    geometry::BeltSlideGeometry,
    lift_impl::impl_lift,
    motor_sensor::MotorIndex,
    presets::LiftPresets,
    sensor::LiftSensor,
    source::SensorSource,
};

impl_lift!(
    name     = BeltSlideLift,
    geo      = BeltSlideGeometry,
    doc      = "Belt driven linear slide.",
    fallible = false,
);

impl<S: LiftSensor, C: LiftController> BeltSlideLift<S, C> {
    /// Move to `height_in` in
    /// Clamped to soft limits.
    pub fn go_to_inches(&mut self, height_in: f64) {
        self.target_in = height_in.clamp(self.min_height_in, self.max_height_in);
        self.controller.reset();
        debug!("BeltSlideLift -> {:.3} in", self.target_in);
    }
}
