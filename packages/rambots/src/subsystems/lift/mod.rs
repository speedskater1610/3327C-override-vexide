//! # Lift subsystem framework
//!
//! Geometry-aware, inch-based control for **every common VRC lift type**,
//! with pluggable sensors and controllers.
//!
//! ---
//!
//! ## Lift types
//!
//! | Struct | Geometry | Best for |
//! |---|---|---|
//! | [`CascadeLift`] | Multi-stage chain, sprocket-driven | High-reach stackers, ring elevators |
//! | [`Dr4bLift`] | Double-reverse 4-bar linkage | Precise ring/mogo scoring |
//! | [`TwoBarLift`] | Single-pivot arm | Claws, mogo intake, hang arms |
//! | [`SixBarLift`] | Parallel 6-bar linkage | High-travel ball/ring scoring |
//! | [`ContinuousChainLift`] | Single-loop vertical chain | Fast indexers, single-stage elevators |
//! | [`ScissorsLift`] | Multi-stage crossing arms | Skyrise-style extreme height |
//! | [`RackPinionLift`] | Pinion gear on linear rack | Fast precise linear actuation |
//! | [`BeltSlideLift`] | Timing belt over pulleys | Fast lightweight linear slides |
//!
//! ---
//!
//! ## Sensor sources
//!
//! Every lift accepts any of these as the `sensor` argument:
//!
//! | Type | What it reads |
//! |---|---|
//! | `MotorIndex(n)` | Built-in encoder of motor `n` in the Vec |
//! | `AveragedMotorSensor` | Average of **all** motor encoders most accurate |
//! | `PotentiometerSensor::new(port)` | ADI potentiometer (0-4095 ADC -> degrees) |
//! | `RotationSensor::new(sensor)` | V5 rotation sensor (smart port) |
//! | `FusedSensor::new(vec![...])` | Average of any combination of the above |
//!
//! Or call one of the motor-encoder convenience constructors that skip
//! the sensor argument entirely:
//!
//! ```rust
//! // Single motor encoder
//! let lift = CascadeLift::with_motor_encoder(motors, geo, /*index*/ 0, pid, presets);
//!
//! // Average all motor encoders (recommended)
//! let lift = CascadeLift::with_averaged_motors(motors, geo, pid, presets);
//! ```
//!
//! ---
//!
//! ## Controllers
//!
//! | Type | Description |
//! |---|---|
//! | [`LiftPid`] | PID with anti-windup, derivative filter, and dual settled condition |
//! | [`LiftBangBang`] | On/off with dead-band - no tuning needed |
//!
//! ### Tuning `LiftPid`
//!
//! ```rust
//! let pid = LiftPid::new(/*kp*/ 900.0, /*ki*/ 2.0, /*kd*/ 80.0)
//!     .with_integral_limit(2.0)          // clamp integral to + or -2 in
//!     .with_derivative_filter(0.85)       // IIR filter: 1.0 = off, 0.0 = all filtered
//!     .with_settled_threshold(0.25)       // settled when error < 0.25 in ...
//!     .with_settled_velocity_threshold(0.5); // ... AND velocity < 0.5 in/s
//! ```
//!
//! Gains are in units of **mV / in** (`kp`), **mV / (in*s)** (`ki`),
//! **mV*s / in** (`kd`). Start with `ki = 0`, `kd = 0` and increase `kp`
//! until the lift reaches the target without oscillating, then add `kd` to
//! dampen overshoot.
//!
//! ---
//!
//! ## Named presets
//!
//! ```rust
//! let presets = LiftPresets::new()
//!     .add("lowered",    0.0)
//!     .add("intake",     2.5)
//!     .add("score_low",  12.0)
//!     .add("score_mid",  20.0)
//!     .add("score_high", 28.5);
//!
//! lift.go_to_preset_name("score_high");        // by name
//! lift.go_to_preset_index(2);                  // by index (0-based)
//! lift.go_to_next_preset();                    // cycle up (wraps)
//! lift.go_to_prev_preset();                    // cycle down (wraps)
//! ```
//!
//! ---
//!
//! ## Shared API (all lift types)
//!
//! ```rust
//! // Targeting
//! lift.go_to_inches(18.0);
//! lift.go_to_preset_name("score_high");
//! lift.go_to_next_preset();   // great for a controller button
//! lift.go_to_prev_preset();
//!
//! // Control loop - call every 10 ms
//! lift.update(0.010);
//!
//! // Status
//! lift.is_settled()                  // -> bool
//! lift.current_height_inches()       // -> Option<f64>
//! lift.target_height_inches()        // -> f64
//! lift.error_inches()                // -> Option<f64>
//! lift.nearest_preset_name()         // -> Option<&str>
//!
//! // Manual override
//! lift.set_manual(0.5);   // 50% power up, bypasses controller
//! lift.stop();            // cut power
//! lift.zero_sensor();     // zero at current hard-stop position
//! ```
//!
//! ---
//!
//! ## Quick-start examples
//!
//! ### Cascade - averaged motor encoders
//! ```rust
//! use controls::subsystems::lift::*;
//! let geo = CascadeGeometry { sprocket_radius_in: 1.096, gear_ratio: 5.0, stages: 2 };
//! let pid = LiftPid::new(900.0, 2.0, 80.0).with_settled_threshold(0.3);
//! let presets = LiftPresets::new().add("down", 0.0).add("up", 28.0);
//! let mut lift = CascadeLift::with_averaged_motors(motors, geo, pid, presets);
//! ```
//!
//! ### DR4B - rotation sensor
//! ```rust
//! use controls::subsystems::lift::*;
//! let geo = Dr4bGeometry { arm_length_in: 11.5, gear_ratio: 7.0, base_height_in: 5.0 };
//! let sensor = RotationSensor::new(rotation_port);
//! let pid = LiftPid::new(1100.0, 3.0, 90.0).with_derivative_filter(0.85);
//! let presets = LiftPresets::new().add("down", 5.0).add("score", 24.0);
//! let mut lift = Dr4bLift::new(motors, geo, sensor, pid, presets);
//! ```
//!
//! ### 2-Bar - potentiometer
//! ```rust
//! use controls::subsystems::lift::*;
//! let geo = TwoBarGeometry { arm_length_in: 14.0, gear_ratio: 5.0,
//!                             base_height_in: 4.0, zero_angle_deg: -45.0 };
//! let pot = PotentiometerSensor::new(adi_port).with_zero_adc(820);
//! let pid = LiftPid::new(800.0, 1.5, 70.0).with_settled_threshold(0.2);
//! let presets = LiftPresets::new().add("stowed", -6.0).add("score", 18.0);
//! let mut arm = TwoBarLift::new(motors, geo, pot, pid, presets);
//! ```

pub mod belt_slide;
pub mod cascade;
pub mod continuous_chain;
pub mod controller;
pub mod dr4b;
pub mod geometry;
pub(crate) mod lift_impl;
pub mod motor_sensor;
pub mod presets;
pub mod rack_pinion;
pub mod scissors;
pub mod sensor;
pub mod six_bar;
pub(crate) mod source;
pub mod two_bar;

// Lift structs
pub use belt_slide::BeltSlideLift;
pub use cascade::{CascadeLift, MotorEncoderCascade, NullSensor};
pub use continuous_chain::ContinuousChainLift;
pub use dr4b::{Dr4bLift, MotorEncoderDr4b};
pub use rack_pinion::RackPinionLift;
pub use scissors::ScissorsLift;
pub use six_bar::SixBarLift;
pub use two_bar::TwoBarLift;

// Geometry structs
pub use geometry::{
    BeltSlideGeometry, CascadeGeometry, ContinuousChainGeometry, Dr4bGeometry,
    RackPinionGeometry, ScissorsGeometry, SixBarGeometry,
    TwoBarGeometry,
};

// Controllers
pub use controller::{LiftBangBang, LiftController, LiftPid};

// Sensors
pub use motor_sensor::{AveragedMotorSensor, MotorIndex};
pub use sensor::{
    FusedSensor, LiftSensor, PotentiometerSensor, RotationSensor, StandaloneMotorEncoder,
};

// Presets
pub use presets::LiftPresets;