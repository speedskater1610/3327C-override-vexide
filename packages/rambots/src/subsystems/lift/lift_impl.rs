//! Internal macro that generates a complete lift struct for any geometry type.
//!
//! Each lift type (2-bar, scissors, etc.) is structurally identical they
//! differ only in their geometry struct and the name of their
//! `degrees_to_inches` / `inches_to_degrees` methods. This macro stamps out
//! all the boilerplate so we only have to write and maintain the logic once.
//!
//! The `unreachable` parameter controls whether `inches_to_degrees` returns
//! `Option<f64>` (arms/scissors, can physically run out of range) or `f64`
//! (linear mechanisms that are only limited by soft stops).

/// Generates a complete lift struct plus all targeting/status/control methods.
///
/// # Parameters
///
/// * `$name`         struct name, e.g. `TwoBarLift`
/// * `$geo`          geometry struct, e.g. `TwoBarGeometry`
/// * `$doc`          string literal used as the module-level doc comment
/// * `unreachable`   literal `true` if `inches_to_degrees` returns `Option<f64>`,
///                    `false` if it returns `f64`
macro_rules! impl_lift {
    // Rotary lifts (inches_to_degrees -> Option<f64>)
    (
        name      = $name:ident,
        geo       = $geo:ty,
        doc       = $doc:literal,
        fallible  = true $(,)?
    ) => {
        impl_lift!(@inner $name, $geo, $doc, true);
    };

    // Linear lifts (inches_to_degrees -> f64)
    (
        name      = $name:ident,
        geo       = $geo:ty,
        doc       = $doc:literal,
        fallible  = false $(,)?
    ) => {
        impl_lift!(@inner $name, $geo, $doc, false);
    };

    // shared body
    (@inner $name:ident, $geo:ty, $doc:literal, $fallible:literal) => {
        #[doc = $doc]
        pub struct $name<S: $crate::subsystems::lift::sensor::LiftSensor, C: $crate::subsystems::lift::controller::LiftController> {
            pub(crate) motors:     alloc::vec::Vec<vexide::devices::smart::motor::Motor>,
            pub(crate) geometry:   $geo,
            pub(crate) source:     $crate::subsystems::lift::source::SensorSource<S>,
            pub(crate) controller: C,
            pub(crate) presets:    $crate::subsystems::lift::presets::LiftPresets,
            pub(crate) target_in:      f64,
            pub(crate) min_height_in:  f64,
            pub(crate) max_height_in:  f64,
            pub(crate) hold_position:  bool,
        }

        impl<S, C> $name<S, C>
        where
            S: $crate::subsystems::lift::sensor::LiftSensor,
            C: $crate::subsystems::lift::controller::LiftController,
        {
            /// construct from an external sensor implementing [`LiftSensor`].
            pub fn new(
                motors:     alloc::vec::Vec<vexide::devices::smart::motor::Motor>,
                geometry:   $geo,
                sensor:     S,
                controller: C,
                presets:    $crate::subsystems::lift::presets::LiftPresets,
            ) -> Self {
                Self::from_source(motors, geometry, $crate::subsystems::lift::source::SensorSource::External(sensor), controller, presets)
            }

            pub(crate) fn from_source(
                motors:     alloc::vec::Vec<vexide::devices::smart::motor::Motor>,
                geometry:   $geo,
                source:     $crate::subsystems::lift::source::SensorSource<S>,
                controller: C,
                presets:    $crate::subsystems::lift::presets::LiftPresets,
            ) -> Self {
                Self {
                    motors, geometry, source, controller, presets,
                    target_in: 0.0,
                    min_height_in: 0.0,
                    max_height_in: f64::MAX,
                    hold_position: true,
                }
            }


            // config

            /// Set the soft lower limit in inches.
            pub fn with_min_height(mut self, in_: f64) -> Self { self.min_height_in = in_; self }
            /// Set the soft upper limit in inches.
            pub fn with_max_height(mut self, in_: f64) -> Self { self.max_height_in = in_; self }
            /// When `false`, motors coast after settling.
            pub fn with_hold_position(mut self, hold: bool) -> Self { self.hold_position = hold; self }

            // Control loop

            /// Run one control iteration. Call every loop tick with `dt` in seconds.
            pub fn update(&mut self, dt: f64) {
                let current = match self.source.position_degrees(&self.motors) {
                    Some(deg) => self.geometry.degrees_to_inches(deg),
                    None => return,
                };
                let voltage = if self.controller.is_settled() && !self.hold_position {
                    0.0
                } else {
                    self.controller.compute(self.target_in, current, dt)
                };
                for m in &mut self.motors { let _ = m.set_voltage(voltage); }
            }

            // Status

            /// Current height in inches, or `None` on sensor error.
            pub fn current_height_inches(&self) -> Option<f64> {
                Some(self.geometry.degrees_to_inches(self.source.position_degrees(&self.motors)?))
            }

            /// Target height in inches.
            pub fn target_height_inches(&self) -> f64 { self.target_in }

            /// `true` when settled at the target.
            pub fn is_settled(&self) -> bool { self.controller.is_settled() }

            /// Height error in inches (positive = below target).
            pub fn error_inches(&self) -> Option<f64> {
                Some(self.target_in - self.current_height_inches()?)
            }

            /// Name of the nearest preset to the current position.
            pub fn nearest_preset_name(&self) -> Option<&str> {
                self.presets.nearest(self.current_height_inches()?)
            }

            // Presets

            /// Move to a named preset.
            pub fn go_to_preset_name(&mut self, name: &str) {
                match self.presets.get(name) {
                    Some(h) => self.go_to_inches(h),
                    None => log::warn!(concat!(stringify!($name), ": unknown preset '{}'"), name),
                }
            }

            /// Move to a preset by index.
            pub fn go_to_preset_index(&mut self, index: usize) {
                match self.presets.iter().nth(index) {
                    Some((name, h)) => {
                        log::debug!(concat!(stringify!($name), " preset[{}] '{}' -> {:.3} in"), index, name, h);
                        self.go_to_inches(h);
                    }
                    None => log::warn!(concat!(stringify!($name), ": preset index {} out of range"), index),
                }
            }

            /// Cycle up to the next taller preset; wraps to lowest.
            pub fn go_to_next_preset(&mut self) {
                let current = self.current_height_inches().unwrap_or(self.target_in);
                let next = self.presets.iter()
                    .filter(|(_, h)| *h > current + 0.5)
                    .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap());
                match next {
                    Some((_, h)) => self.go_to_inches(h),
                    None => if let Some((_, h)) = self.presets.iter().next() { self.go_to_inches(h); },
                }
            }

            /// Cycle down to the next shorter preset; wraps to highest.
            pub fn go_to_prev_preset(&mut self) {
                let current = self.current_height_inches().unwrap_or(self.target_in);
                let prev = self.presets.iter()
                    .filter(|(_, h)| *h < current - 0.5)
                    .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap());
                match prev {
                    Some((_, h)) => self.go_to_inches(h),
                    None => if let Some((_, h)) = self.presets.iter().last() { self.go_to_inches(h); },
                }
            }


            // manually update

            /// Raw voltage fraction `[-1.0, 1.0]`; bypasses controller.
            pub fn set_manual(&mut self, fraction: f64) {
                let v = fraction.clamp(-1.0, 1.0) * 12_000.0;
                for m in &mut self.motors { let _ = m.set_voltage(v); }
            }

            /// Cut all motor power.
            pub fn stop(&mut self) {
                for m in &mut self.motors { let _ = m.set_voltage(0.0); }
            }

            /// Zero the sensor (call when at the physical hard-stop).
            pub fn zero_sensor(&mut self) {
                self.source.reset(&mut self.motors);
                self.target_in = 0.0;
                self.controller.reset();
                log::debug!(concat!(stringify!($name), " sensor zeroed"));
            }
        }

        // Motor encoder constructors

        impl<C: $crate::subsystems::lift::controller::LiftController>
            $name<$crate::subsystems::lift::cascade::NullSensor, C>
        {
            /// Use the built-in encoder of motor `motor_index`.
            pub fn with_motor_encoder(
                motors: alloc::vec::Vec<vexide::devices::smart::motor::Motor>,
                geometry: $geo,
                motor_index: usize,
                controller: C,
                presets: $crate::subsystems::lift::presets::LiftPresets,
            ) -> Self {
                Self::from_source(
                    motors, geometry,
                    $crate::subsystems::lift::source::SensorSource::MotorIdx(
                        $crate::subsystems::lift::motor_sensor::MotorIndex(motor_index)
                    ),
                    controller, presets,
                )
            }

            /// avg all motor encoders for position feedback.
            pub fn with_averaged_motors(
                motors: alloc::vec::Vec<vexide::devices::smart::motor::Motor>,
                geometry: $geo,
                controller: C,
                presets: $crate::subsystems::lift::presets::LiftPresets,
            ) -> Self {
                Self::from_source(
                    motors, geometry,
                    $crate::subsystems::lift::source::SensorSource::AllMotors,
                    controller, presets,
                )
            }
        }
    };
}

pub(crate) use impl_lift;
