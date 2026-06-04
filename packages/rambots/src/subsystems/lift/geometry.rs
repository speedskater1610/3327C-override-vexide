//! Geometric models that convert sensor degrees <-> lift height in inches
//!
//! Both models expose two methods:
//! * [`degrees_to_inches`] - sensor degrees -> carriage height (inches)
//! * [`inches_to_degrees`] - target height (inches) -> required sensor degrees
//!
//! # Cascade geometry
//!
//! ```text
//!  height_in = (degrees / 360) * (2pi * sprocket_radius_in) * stages * gear_ratio
//! ```
//!
//! * `stages` - number of cascade stages (each stage multiplies travel; typically 2 - 4)
//! * `gear_ratio` - motor rotations per sprocket rotation (eg `1.0/5.0` for 5:1 reduction)
//! * `sprocket_radius_in` - pitch radius of the drive sprocket in inches
//!
//! # DR4B geometry
//!
//! ```text
//!  height_in = 2 * arm_length_in * sin(angle_rad) * gear_ratio
//! ```
//!
//! * `arm_length_in` - length of one arm link in inches
//! * `gear_ratio` - motor rotations per arm rotation (`7.0` for 7:1 reduction)
//! * `base_height_in` - height of the pivot above the ground at zero degrees

use core::f64::consts::PI;

// Cascade

/// Geometric parameters for a cascade (linear chain) lift
#[derive(Debug, Clone, Copy)]
pub struct CascadeGeometry {
    /// Pitch radius of the drive sprocket in **inches**
    ///
    /// Common sprockets:
    /// | Sprocket teeth | Pitch radius (in) |
    /// |---|---|
    /// | 12t | 0.659 |
    /// | 16t | 0.875 |
    /// | 20t | 1.096 |
    /// | 24t | 1.312 |
    pub sprocket_radius_in: f64,

    /// motor to sprocket gear ratio, expressed as `motor_turns / sprocket_turns`
    pub gear_ratio: f64,

    /// number of cascade stages (each stage multiplies the linear travel)
    /// A single stage lift = 1; a 2-stage = 2, etc
    pub stages: u8,
}

impl CascadeGeometry {
    /// convert a sensor reading (motor degrees) to lift height in inches
    #[inline]
    pub fn degrees_to_inches(&self, motor_degrees: f64) -> f64 {
        let motor_revs = motor_degrees / 360.0;
        let sprocket_revs = motor_revs / self.gear_ratio;
        let chain_travel = sprocket_revs * 2.0 * PI * self.sprocket_radius_in;
        chain_travel * self.stages as f64
    }

    /// Convert a desired height in inches to the required motor position in degrees
    #[inline]
    pub fn inches_to_degrees(&self, height_in: f64) -> f64 {
        let chain_travel = height_in / self.stages as f64;
        let sprocket_revs = chain_travel / (2.0 * PI * self.sprocket_radius_in);
        let motor_revs = sprocket_revs * self.gear_ratio;
        motor_revs * 360.0
    }

    /// Maximum achievable height given `motor_max_degrees` as the physical stop
    pub fn max_height_in(&self, motor_max_degrees: f64) -> f64 {
        self.degrees_to_inches(motor_max_degrees)
    }
}

// DR4B

/// Geometric parameters for a Double-Reverse 4-Bar (DR4B) lift
///
/// A DR4B has two 4-bar linkages stacked The lower 4-bar rotates by `thetaa`
/// while the upper 4-bar rotates by `-theta` (keeping the end-effector vertical)
/// The net height change is `2 * arm_length_in * sin(theta)`
#[derive(Debug, Clone, Copy)]
pub struct Dr4bGeometry {
    /// Length of each arm link in **inches** (lower and upper arms are assumed equal)
    ///
    /// Measure from pivot to pivot along the arm
    pub arm_length_in: f64,

    /// Motor-to-arm gear ratio: `motor_turns / arm_turns`
    ///
    /// Examples:
    /// * 7:1 reduction -> `7.0`
    /// * 5:1 reduction -> `5.0`
    pub gear_ratio: f64,

    /// Height of the lower pivot above the ground in **inches** when the arm
    /// is at its zero (fully lowered) encoder position
    ///
    /// Add this to the kinematic height to get the absolute height of the
    /// end-effector above the floor
    pub base_height_in: f64,
}

impl Dr4bGeometry {
    /// Convert a sensor reading (motor degrees) to end-effector height in inches
    /// above the floor
    ///
    /// `motor_degrees` is the position relative to the zeroed (lowered) position
    #[inline]
    pub fn degrees_to_inches(&self, motor_degrees: f64) -> f64 {
        let arm_degrees = motor_degrees / self.gear_ratio;
        let arm_radians = arm_degrees.to_radians();
        let kinematic_height = 2.0 * self.arm_length_in * arm_radians.sin();
        self.base_height_in + kinematic_height
    }

    /// Convert a desired absolute end-effector height in inches to the required
    /// motor position in degrees
    ///
    /// Returns `None` if `height_in` is geometrically unreachable 
    /// (ie `sin` would exceed + or - 1).
    #[inline]
    pub fn inches_to_degrees(&self, height_in: f64) -> Option<f64> {
        let kinematic_height = height_in - self.base_height_in;
        let sin_val = kinematic_height / (2.0 * self.arm_length_in);

        if sin_val.abs() > 1.0 {
            return None; // geometrically unreachable
        }

        let arm_radians = sin_val.asin();
        let arm_degrees = arm_radians.to_degrees();
        let motor_degrees = arm_degrees * self.gear_ratio;
        Some(motor_degrees)
    }

    /// Maximum achievable height (arm at 90deg, fully extended)
    pub fn max_height_in(&self) -> f64 {
        self.base_height_in + 2.0 * self.arm_length_in
    }
}

// unit tests
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cascade_round_trip() {
        let geo = CascadeGeometry {
            sprocket_radius_in: 1.096, // 20t sprocket
            gear_ratio: 5.0,
            stages: 2,
        };
        let original = 18.0_f64;
        let degrees = geo.inches_to_degrees(original);
        let back = geo.degrees_to_inches(degrees);
        assert!((back - original).abs() < 1e-9, "round-trip failed: {back}");
    }

    #[test]
    fn dr4b_round_trip() {
        let geo = Dr4bGeometry {
            arm_length_in: 12.0,
            gear_ratio: 7.0,
            base_height_in: 4.0,
        };
        let original = 18.0_f64;
        let degrees = geo.inches_to_degrees(original).unwrap();
        let back = geo.degrees_to_inches(degrees);
        assert!((back - original).abs() < 1e-9, "round-trip failed: {back}");
    }

    #[test]
    fn dr4b_unreachable() {
        let geo = Dr4bGeometry { arm_length_in: 6.0, gear_ratio: 5.0, base_height_in: 3.0 };
        // max reachable = 3 + 2 * 6 = 15 in; 20 in is beyond that
        assert!(geo.inches_to_degrees(20.0).is_none());
    }
}

// 2-Bar (single-pivot arm)

/// Geometric parameters for a **2-Bar** (single pivot) arm lift
///
/// The simplest arm design: one rigid link rotates around a fixed pivot
/// Height is purely a function of the angle the arm makes with horizontal
///
/// ```text

///  height_in = base_height_in + arm_length_in * sin(arm_angle_rad)
/// ```
///
/// # Measuring your 2-bar
///
/// * `arm_length_in` - distance from pivot bolt to end-effector mounting point
/// * `gear_ratio`    - motor turns per arm turn (eg 5.0 for a 5:1 reduction)
/// * `base_height_in`- height of the pivot bolt above the floor
/// * `zero_angle_deg`- what angle (from horizontal) the arm is at when the
///                     sensor/encoder reads zero. Usually 0 if you zero at
///                     horizontal, or a negative value if you zero at rest-down
#[derive(Debug, Clone, Copy)]
pub struct TwoBarGeometry {
    /// Length from pivot to end-effector in inches
    pub arm_length_in: f64,
    /// Motor-to-arm gear ratio (`motor_turns / arm_turns`)
    pub gear_ratio: f64,
    /// Height of the pivot above the floor in inches
    pub base_height_in: f64,
    /// Angle (degrees) the arm sits at when the encoder reads zero
    /// Set to 0 if you zero the encoder when the arm is horizontal
    /// Set to -90 if you zero when fully hanging down
    pub zero_angle_deg: f64,
}

impl TwoBarGeometry {
    /// Motor encoder degrees -> end-effector height in inches
    #[inline]
    pub fn degrees_to_inches(&self, motor_degrees: f64) -> f64 {
        let arm_degrees = motor_degrees / self.gear_ratio + self.zero_angle_deg;
        let arm_radians = arm_degrees.to_radians();
        self.base_height_in + self.arm_length_in * arm_radians.sin()
    }

    /// Target height in inches -> required motor encoder degrees
    ///
    /// Returns `None` when the height is geometrically unreachable
    #[inline]
    pub fn inches_to_degrees(&self, height_in: f64) -> Option<f64> {
        let relative = height_in - self.base_height_in;
        let sin_val = relative / self.arm_length_in;
        if sin_val.abs() > 1.0 {
            return None;
        }
        let arm_degrees = sin_val.asin().to_degrees() - self.zero_angle_deg;
        Some(arm_degrees * self.gear_ratio)
    }

    /// Maximum height (arm at 90def above horizontal)
    pub fn max_height_in(&self) -> f64 {
        self.base_height_in + self.arm_length_in
    }

    /// Minimum height (arm at -90deg, pointing straight down)
    pub fn min_height_in(&self) -> f64 {
        self.base_height_in - self.arm_length_in
    }
}

// Continuous chain (elevator / vertical chain drive)

/// Geometric parameters for a **continuous chain** (vertical chain drive) lift
///
/// Unlike a cascade lift, a continuous chain lift has a single loop of chain
/// running over one or more fixed sprockets. The carriage is attached to one
/// side of the chain loop so it travels the full length of the vertical run
///
/// ```text
///  height_in = (motor_degrees / 360) * (2pi * sprocket_radius_in) / gear_ratio
/// ```
///
/// This is mathematically identical to a single-stage cascade. The struct
/// exists as a distinct named type for code clarity
#[derive(Debug, Clone, Copy)]
pub struct ContinuousChainGeometry {
    /// Pitch radius of both sprockets in inches (they must be equal)
    ///
    /// Common sprocket pitch radii:
    /// | Teeth | Radius (in) |
    /// |---|---|
    /// | 12t   | 0.659       |
    /// | 16t   | 0.875       |
    /// | 20t   | 1.096       |
    /// | 24t   | 1.312       |
    pub sprocket_radius_in: f64,

    /// Motor turns per sprocket turn
    pub gear_ratio: f64,
}

impl ContinuousChainGeometry {
    #[inline]
    pub fn degrees_to_inches(&self, motor_degrees: f64) -> f64 {
        let motor_revs = motor_degrees / 360.0;
        let sprocket_revs = motor_revs / self.gear_ratio;
        sprocket_revs * 2.0 * PI * self.sprocket_radius_in
    }

    #[inline]
    pub fn inches_to_degrees(&self, height_in: f64) -> f64 {
        let sprocket_revs = height_in / (2.0 * PI * self.sprocket_radius_in);
        sprocket_revs * self.gear_ratio * 360.0
    }
}

// Scissors lift

/// Geometric parameters for a **scissors lift**
///
/// A scissors lift uses pairs of crossing arms linked at their midpoints
/// As the bottom scissors spread horizontally, the platform rises. Each
/// additional scissors stage doubles the height per unit of horizontal spread
///
/// ```text
///  height_in = stages * arm_length_in * sin(half_angle_rad)
/// ```
///
/// For scissors lifts the motor typically drives a lead screw or rack &
/// pinion that changes the horizontal spread, so `gear_ratio` here is the
/// ratio of motor turns to lead-screw rotations (or direct if belt-driven).
///
/// The position feedback is usually from the **drive motor** directly if using
/// a lead screw (where motor angle -> linear extension is known), or from a
/// rotation sensor on one of the scissors arms. Set `sensor_on_arm = true`
/// to have the geometry read the arm angle directly instead of the motor.
#[derive(Debug, Clone, Copy)]
pub struct ScissorsGeometry {
    /// Length of each scissors arm link from end-pivot to center-pivot (inches)
    pub arm_length_in: f64,
    /// Number of scissors stages stacked vertically
    pub stages: u8,
    /// Motor turns per arm-angle degree (only used when `sensor_on_arm = false`)
    /// If you're reading the arm pivot directly, set this to 1.0
    pub gear_ratio: f64,
    /// Set `true` if the sensor is mounted directly on a scissors arm pivot
    /// When `true`, `degrees_to_inches` interprets its input as arm degrees
    /// directly (skips the gear ratio division)
    pub sensor_on_arm: bool,
}

impl ScissorsGeometry {
    /// Motor (or arm) degrees -> platform height in inches.
    #[inline]
    pub fn degrees_to_inches(&self, sensor_degrees: f64) -> f64 {
        let arm_degrees = if self.sensor_on_arm {
            sensor_degrees
        } else {
            sensor_degrees / self.gear_ratio
        };
        let arm_radians = arm_degrees.to_radians();
        self.stages as f64 * self.arm_length_in * arm_radians.sin()
    }

    /// Target height -> motor degrees. Returns `None` if unreachable.
    #[inline]
    pub fn inches_to_degrees(&self, height_in: f64) -> Option<f64> {
        let sin_val = height_in / (self.stages as f64 * self.arm_length_in);
        if sin_val.abs() > 1.0 {
            return None;
        }
        let arm_degrees = sin_val.asin().to_degrees();
        let sensor_degrees = if self.sensor_on_arm {
            arm_degrees
        } else {
            arm_degrees * self.gear_ratio
        };
        Some(sensor_degrees)
    }

    /// Maximum height (arms at 90deg).
    pub fn max_height_in(&self) -> f64 {
        self.stages as f64 * self.arm_length_in
    }
}

// Rack & Pinion

/// Geometric parameters for a **rack & pinion** linear actuator.
///
/// a pinion gear mounted on the motor shaft meshes with a linear rack.
/// One full pinion revolution moves the rack by `2pi * pinion_radius_in`.
///
/// ```text
///  height_in = (motor_degrees / 360) * (2pi * pinion_radius_in) / gear_ratio
/// ```
#[derive(Debug, Clone, Copy)]
pub struct RackPinionGeometry {
    /// Pitch radius of the pinion gear in inches.
    ///
    /// Standard vex rack & pinion: `0.500 in`
    pub pinion_radius_in: f64,
    /// Motor turns per pinion turn.
    pub gear_ratio: f64,
}

impl RackPinionGeometry {
    #[inline]
    pub fn degrees_to_inches(&self, motor_degrees: f64) -> f64 {
        let motor_revs = motor_degrees / 360.0;
        let pinion_revs = motor_revs / self.gear_ratio;
        pinion_revs * 2.0 * PI * self.pinion_radius_in
    }

    #[inline]
    pub fn inches_to_degrees(&self, height_in: f64) -> f64 {
        let pinion_revs = height_in / (2.0 * PI * self.pinion_radius_in);
        pinion_revs * self.gear_ratio * 360.0
    }
}

// linear slide (lead screw / ball screw)

/// Geometric parameters for a **lead screw** or **ball screw** linear actuator.
///
/// A threaded rod turns and a carriage on the rod translates linearly.
/// Travel per revolution = `lead_in` (the "pitch" of the thread).
///
/// ```text
///  height_in = (motor_degrees / 360) * lead_in / gear_ratio
/// ```
#[derive(Debug, Clone, Copy)]
pub struct LeadScrewGeometry {
    /// Linear travel per screw revolution in **inches**.
    pub lead_in: f64,
    /// motor turns per screw turn
    pub gear_ratio: f64,
}

impl LeadScrewGeometry {
    #[inline]
    pub fn degrees_to_inches(&self, motor_degrees: f64) -> f64 {
        let motor_revs = motor_degrees / 360.0;
        let screw_revs = motor_revs / self.gear_ratio;
        screw_revs * self.lead_in
    }

    #[inline]
    pub fn inches_to_degrees(&self, height_in: f64) -> f64 {
        let screw_revs = height_in / self.lead_in;
        screw_revs * self.gear_ratio * 360.0
    }
}

// Belt-driven linear slide

/// Geometric parameters for a **belt-driven linear slide**.
///
/// A toothed belt runs over two pulleys and the carriage is clamped to one
/// run of the belt. One pulley revolution moves the carriage by
/// `2pi * pulley_radius_in`.
///
/// ```text
///  height_in = (motor_degrees / 360) * (2pi * pulley_radius_in) / gear_ratio
/// ```
#[derive(Debug, Clone, Copy)]
pub struct BeltSlideGeometry {
    /// Pitch radius of the drive pulley in inches.
    pub pulley_radius_in: f64,
    /// Motor turns per pulley turn.
    pub gear_ratio: f64,
}

impl BeltSlideGeometry {
    #[inline]
    pub fn degrees_to_inches(&self, motor_degrees: f64) -> f64 {
        let motor_revs = motor_degrees / 360.0;
        let pulley_revs = motor_revs / self.gear_ratio;
        pulley_revs * 2.0 * PI * self.pulley_radius_in
    }

    #[inline]
    pub fn inches_to_degrees(&self, height_in: f64) -> f64 {
        let pulley_revs = height_in / (2.0 * PI * self.pulley_radius_in);
        pulley_revs * self.gear_ratio * 360.0
    }
}

// Extended unit tests
#[cfg(test)]
mod more_tests {
    use super::*;

    #[test]
    fn two_bar_round_trip() {
        let geo = TwoBarGeometry {
            arm_length_in: 14.0,
            gear_ratio: 5.0,
            base_height_in: 3.0,
            zero_angle_deg: 0.0,
        };
        let h = 10.0;
        let deg = geo.inches_to_degrees(h).unwrap();
        let back = geo.degrees_to_inches(deg);
        assert!((back - h).abs() < 1e-9, "2 bar round trip: {back}");
    }

    #[test]
    fn two_bar_unreachable() {
        let geo = TwoBarGeometry { arm_length_in: 6.0, gear_ratio: 5.0, base_height_in: 2.0, zero_angle_deg: 0.0 };
        assert!(geo.inches_to_degrees(9.0).is_none()); // 2+6=8 max
    }

    #[test]
    fn continuous_chain_round_trip() {
        let geo = ContinuousChainGeometry { sprocket_radius_in: 1.096, gear_ratio: 3.0 };
        let h = 24.0;
        let deg = geo.inches_to_degrees(h);
        assert!((geo.degrees_to_inches(deg) - h).abs() < 1e-9);
    }

    #[test]
    fn scissors_round_trip() {
        let geo = ScissorsGeometry { arm_length_in: 10.0, stages: 3, gear_ratio: 7.0, sensor_on_arm: false };
        let h = 20.0;
        let deg = geo.inches_to_degrees(h).unwrap();
        assert!((geo.degrees_to_inches(deg) - h).abs() < 1e-9);
    }

    #[test]
    fn rack_pinion_round_trip() {
        let geo = RackPinionGeometry { pinion_radius_in: 0.5, gear_ratio: 1.0 };
        let h = 8.0;
        let deg = geo.inches_to_degrees(h);
        assert!((geo.degrees_to_inches(deg) - h).abs() < 1e-9);
    }

    #[test]
    fn lead_screw_round_trip() {
        let geo = LeadScrewGeometry { lead_in: 0.125, gear_ratio: 1.0 };
        let h = 3.0;
        let deg = geo.inches_to_degrees(h);
        assert!((geo.degrees_to_inches(deg) - h).abs() < 1e-9);
    }

    #[test]
    fn belt_slide_round_trip() {
        let geo = BeltSlideGeometry { pulley_radius_in: 0.875, gear_ratio: 3.0 };
        let h = 18.0;
        let deg = geo.inches_to_degrees(h);
        assert!((geo.degrees_to_inches(deg) - h).abs() < 1e-9);
    }
}

// 6-Bar

/// Geometric parameters for a **6-Bar** parallel linkage lift.
///
/// Uses the simplified equal-coupling model:
///
/// ```text
///  height = base_height_in
///         + lower_arm_in * sin(lower_angle_rad)
///         + upper_arm_in * sin(lower_angle_rad * coupling_ratio)
/// ```
#[derive(Debug, Clone, Copy)]
pub struct SixBarGeometry {
    /// Length of the lower linkage arm in inches (pivot-to-pivot).
    pub lower_arm_in: f64,
    /// Length of the upper linkage arm in inches (pivot-to-pivot).
    pub upper_arm_in: f64,
    /// Ratio of upper-stage rotation to lower-stage rotation.
    /// `1.0` for a symmetric 6-bar; `> 1.0` for over-driven designs.
    pub coupling_ratio: f64,
    /// Motor turns per lower-arm revolution.
    pub gear_ratio: f64,
    /// Height of the lower pivot above the floor in inches.
    pub base_height_in: f64,
}

impl SixBarGeometry {
    /// Motor degrees -> end-effector height in inches.
    #[inline]
    pub fn degrees_to_inches(&self, motor_degrees: f64) -> f64 {
        let lower_deg = motor_degrees / self.gear_ratio;
        let lower_rad = lower_deg.to_radians();
        let upper_rad = (lower_deg * self.coupling_ratio).to_radians();
        self.base_height_in
            + self.lower_arm_in * lower_rad.sin()
            + self.upper_arm_in * upper_rad.sin()
    }

    /// Target height -> motor degrees.
    ///
    /// Uses Newton-Raphson iteration because the closed-form inverse requires
    /// solving a transcendental equation. Returns `None` if the height is
    /// unreachable or iteration fails to converge.
    pub fn inches_to_degrees(&self, height_in: f64) -> Option<f64> {
        let target = height_in - self.base_height_in;
        let max_reach = self.lower_arm_in + self.upper_arm_in;
        if target < -max_reach || target > max_reach {
            return None;
        }

        // Initial guess: treat as single arm of combined length
        let sin_guess = (target / max_reach).clamp(-1.0, 1.0);
        let mut lower_rad = sin_guess.asin();

        // Newton-Raphson: minimise f(theta) = lower_arm * sin(theta) + upper_arm * sin(theta * k) - target
        for _ in 0..50 {
            let k = self.coupling_ratio;
            let f = self.lower_arm_in * lower_rad.sin()
                  + self.upper_arm_in * (lower_rad * k).sin()
                  - target;
            let df = self.lower_arm_in * lower_rad.cos()
                   + self.upper_arm_in * k * (lower_rad * k).cos();
            if df.abs() < 1e-10 { break; }
            let step = f / df;
            lower_rad -= step;
            if step.abs() < 1e-9 { break; }
        }

        // Verify convergence
        let result_height = self.lower_arm_in * lower_rad.sin()
            + self.upper_arm_in * (lower_rad * self.coupling_ratio).sin();
        if (result_height - target).abs() > 0.01 {
            return None; // did not converge
        }

        Some(lower_rad.to_degrees() * self.gear_ratio)
    }

    /// Theoretical maximum height (both arms fully extended upward).
    pub fn max_height_in(&self) -> f64 {
        self.base_height_in + self.lower_arm_in + self.upper_arm_in
    }
}

#[cfg(test)]
mod six_bar_tests {
    use super::*;

    #[test]
    fn six_bar_round_trip() {
        let geo = SixBarGeometry {
            lower_arm_in: 9.0,
            upper_arm_in: 9.0,
            coupling_ratio: 1.0,
            gear_ratio: 7.0,
            base_height_in: 4.0,
        };
        let h = 16.0_f64;
        let deg = geo.inches_to_degrees(h).expect("should be reachable");
        let back = geo.degrees_to_inches(deg);
        assert!((back - h).abs() < 0.01, "6-bar round-trip failed: {back}");
    }

    #[test]
    fn six_bar_unreachable() {
        let geo = SixBarGeometry {
            lower_arm_in: 5.0,
            upper_arm_in: 5.0,
            coupling_ratio: 1.0,
            gear_ratio: 5.0,
            base_height_in: 2.0,
        };
        // max = 2 + 5 + 5 = 12; 15 is beyond that
        assert!(geo.inches_to_degrees(15.0).is_none());
    }
}
