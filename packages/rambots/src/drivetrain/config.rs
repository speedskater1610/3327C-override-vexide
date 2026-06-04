//! Physical constants for the Rambots drivetrain.
//!
//! **Edit this file** whenever you change hardware.  Every other module
//! reads from here so you only ever need to change measurements in one place.
//!
//! # Unit convention
//! All linear measurements are in **inches**.
//! All angles are in **degrees** unless a function explicitly says radians.

// Wheel & gear configuration

/// Radius of the drive wheels in inches.
///
/// VEX wheel sizes (radius = diameter / 2):
/// | Wheel | Radius (in) |
/// |---|---|
/// | 2.75 in (2 3/4) | 1.375 |
/// | 3.25 in (3 1/4) | 1.625 |
/// | 4.00 in         | 2.000 |
pub const WHEEL_RADIUS_IN: f64 = 1.625; // TODO: change me (3.25 in wheel default)

/// Gear ratio from motor output shaft to wheel axle.
///
/// The 600 RPM V5 motors connect to a **36-tooth** drive gear.
/// The wheel axle carries a **60-tooth** driven gear.
/// Ratio = drive_teeth / driven_teeth = 36 / 60 = 0.6 (motor turns 1.667x per wheel turn)
///
/// The 5.5W motor is compound-geared to the same final speed,
/// so all three motors share this effective ratio.
///
/// If you change gearing: `gear_ratio = motor_gear_teeth / wheel_gear_teeth`
pub const DRIVE_GEAR_RATIO: f64 = 36.0 / 60.0; // 36T -> 60T

/// Free speed of the 600 RPM V5 motors in RPM.
pub const MOTOR_600_FREE_RPM: f64 = 600.0;

/// Free speed of the 5.5W (200 RPM) motor in RPM.
pub const MOTOR_55W_FREE_RPM: f64 = 200.0;

/// Theoretical maximum drive speed in inches per second.
/// = (motor_rpm x gear_ratio x 2pi x wheel_radius) / 60
pub const MAX_DRIVE_SPEED_IN_S: f64 =
    MOTOR_600_FREE_RPM * DRIVE_GEAR_RATIO * 2.0 * core::f64::consts::PI * WHEEL_RADIUS_IN / 60.0;

// Chassis dimensions

/// Distance between the left and right drive wheels, centre-to-centre, in inches.
/// Measure from the middle of the left tread to the middle of the right tread.
pub const TRACK_WIDTH_IN: f64 = 12.0; // TODO: change me

/// Wheelbase (front-to-back wheel spacing) in inches.
/// For a tank drive this is used for odometry, not steering.
pub const WHEELBASE_IN: f64 = 10.0; // <- change me

// Tracking wheel geometry

/// Radius of the tracking-wheel omni wheels in inches.
/// Default: 2-inch omni -> radius = 1.0 in.
pub const TRACKING_WHEEL_RADIUS_IN: f64 = 1.0; // 2 in omni

/// How far the **left** tracking wheel's contact point is offset from the
/// robot's geometric centre in the forward (+Y) direction, in inches.
/// Positive = in front of centre, negative = behind.
pub const LEFT_TRACKER_OFFSET_Y_IN: f64 = 0.0; // TODO: change me

/// How far the **left** tracking wheel is offset from the robot centre
/// in the lateral (+X) direction.  Negative = towards left side.
pub const LEFT_TRACKER_OFFSET_X_IN: f64 = -3.5; //  change me

/// Same offsets for the **right** tracking wheel.
pub const RIGHT_TRACKER_OFFSET_Y_IN: f64 = 0.0; // TODO: change me
pub const RIGHT_TRACKER_OFFSET_X_IN: f64 = 3.5; // TODO: change me

/// Offset of the **sideways** (strafe) tracking wheel from robot centre,
/// along the robot's forward axis (+Y = in front of centre).
pub const STRAFE_TRACKER_OFFSET_Y_IN: f64 = 0.0; // TODO: change me

// IMU configuration

/// The two IMUs are mounted one on top of the other, rotated 180deg relative
/// to each other (mirrored).  When fused, IMU B's heading is negated before
/// averaging, then divided by 2.  This cancels common-mode vibration noise.
///
/// Set to `true` if IMU B needs its heading negated to be in the same
/// frame as IMU A.  Almost always `true` for the mirrored stacked mount.
pub const IMU_B_INVERTED: bool = true;

// Distance sensor offsets from robot centre (inches)
//
// Each sensor is described by how far its face is from the geometric centre
// of the robot.  The "direction" (N/E/S/W) refers to which way it faces
// in the robot's local frame (+Y = forward = North).
//
// Positive offsets mean the sensor is displaced in its own facing direction
// from centre; negative means it's on the opposite side.

/// How far the NORTH (front-facing) distance sensor face is from centre (in).
pub const DIST_NORTH_OFFSET_IN: f64 = 7.0; // TODO: change me

/// How far the SOUTH (rear-facing) distance sensor face is from centre (in).
pub const DIST_SOUTH_OFFSET_IN: f64 = 7.0; // TODO: change me

/// How far the EAST (right-facing) distance sensor face is from centre (in).
pub const DIST_EAST_OFFSET_IN: f64 = 6.0; // TODO: change me

/// How far the WEST (left-facing) distance sensor face is from centre (in).
pub const DIST_WEST_OFFSET_IN: f64 = 6.0; // TODO: change me

// Control tuning
/// Arcade drive steering sensitivity in `[-1.0, 1.0]`.
/// Lower values make turning less sensitive at full stick.
pub const STEERING_SENSITIVITY: f64 = 0.75;

/// Minimum stick deflection before motion is applied (dead-band).
/// Prevents drift from stick centre slop.
pub const STICK_DEADBAND: f64 = 0.05;
