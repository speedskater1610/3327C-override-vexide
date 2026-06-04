//! Autonomous action primitives.
//!
//! An [`Action`] is a value that describes a single step in an autonomous route.
//! The actual execution logic lives in the robot crate (which owns the hardware),
//! but the action values are defined here so the controls library can compose them.
//!
//! ## Available actions
//!
//! | Variant | Description |
//! |---------|-------------|
//! | `DriveForward` | Drive a distance (mm) at a given power |
//! | `TurnToHeading` | Rotate to an absolute heading (degrees) |
//! | `TurnByAngle`  | Rotate relative to current heading |
//! | `IntakeFor`     | Run intake for a duration (ms) |
//! | `OuttakeFor`    | Run outtake for a duration (ms) |
//! | `LiftTo`        | Move lift to a target height (ticks) |
//! | `ClawOpen`      | Open the claw |
//! | `ClawClose`     | Close the claw |
//! | `Wait`          | Pause for a duration (ms) |
//! | `Parallel`      | Run two actions concurrently (joined when both finish) |
//! | `Race`          | Run two actions concurrently (joined when first finishes) |
//! | `Sequence`      | Run a list of actions sequentially |

use alloc::boxed::Box;
use alloc::vec::Vec;

/// Autonomous action primitive.
///
/// Construct using the associated builder methods (e.g. [`Action::drive_forward`])
/// rather than the enum variants directly the builder API is prob stable
#[derive(Debug)]
pub enum Action {
    /// Drive straight for `distance_mm` millimetres at `power` (0.0-1.0).
    DriveForward {
        distance_mm: f64,
        power: f64,
        timeout_ms: u64,
    },
    /// Drive straight backwards for `distance_mm` millimetres at `power`.
    DriveBackward {
        distance_mm: f64,
        power: f64,
        timeout_ms: u64,
    },
    /// Turn to an absolute field heading in degrees (0 = facing away from alliance wall).
    TurnToHeading {
        heading_deg: f64,
        timeout_ms: u64,
    },
    /// Turn by a relative angle in degrees (positive = clockwise).
    TurnByAngle {
        angle_deg: f64,
        timeout_ms: u64,
    },

    // Intake / Outtake
    IntakeFor { duration_ms: u64 },
    OuttakeFor { duration_ms: u64 },
    IntakeStart,
    IntakeStop,
    OuttakeStart,

    // Lift
    /// Move lift to `ticks` encoder position and wait until settled (or timeout).
    LiftTo { ticks: f64, timeout_ms: u64 },

    // Claw
    ClawOpen,
    ClawClose,
    ClawToggle,

    // Pneumatics
    SolenoidExtend { id: u8 },
    SolenoidRetract { id: u8 },
    SolenoidToggle { id: u8 },

    // Timing
    Wait { duration_ms: u64 },

    //  Combinators
    /// Run all actions in `Vec` sequentially.
    Sequence(Vec<Action>),
    /// Run two actions concurrently; finish when **both** complete.
    Parallel(Box<Action>, Box<Action>),
    /// Run two actions concurrently; finish when the **first** completes.
    Race(Box<Action>, Box<Action>),
}

// Builder methods

impl Action {
    pub fn drive_forward(distance_mm: f64, power: f64, timeout_ms: u64) -> Self {
        Self::DriveForward { distance_mm, power, timeout_ms }
    }

    pub fn drive_backward(distance_mm: f64, power: f64, timeout_ms: u64) -> Self {
        Self::DriveBackward { distance_mm, power, timeout_ms }
    }

    pub fn turn_to_heading(heading_deg: f64, timeout_ms: u64) -> Self {
        Self::TurnToHeading { heading_deg, timeout_ms }
    }

    pub fn turn_by_angle(angle_deg: f64, timeout_ms: u64) -> Self {
        Self::TurnByAngle { angle_deg, timeout_ms }
    }

    pub fn intake_for(duration_ms: u64) -> Self {
        Self::IntakeFor { duration_ms }
    }

    pub fn outtake_for(duration_ms: u64) -> Self {
        Self::OuttakeFor { duration_ms }
    }

    pub fn lift_to(ticks: f64, timeout_ms: u64) -> Self {
        Self::LiftTo { ticks, timeout_ms }
    }

    pub fn wait(duration_ms: u64) -> Self {
        Self::Wait { duration_ms }
    }

    /// Convenience: chain actions into a [`Self::Sequence`].
    pub fn sequence(actions: Vec<Action>) -> Self {
        Self::Sequence(actions)
    }

    /// Run `self` and `other` concurrently; complete when **both** finish.
    pub fn parallel_with(self, other: Action) -> Self {
        Self::Parallel(Box::new(self), Box::new(other))
    }

    /// Run `self` and `other` concurrently; complete when **either** finishes.
    pub fn race_with(self, other: Action) -> Self {
        Self::Race(Box::new(self), Box::new(other))
    }
}
