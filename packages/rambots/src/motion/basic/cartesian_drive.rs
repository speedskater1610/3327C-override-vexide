//! Basic open-loop drive helpers: tank drive and arcade drive.
//!
//! These are thin helpers that map stick inputs to left/right motor voltages.
//! They do no path following use the higher-level motion modules for that.

use vexide::devices::smart::motor::Motor;

/// Maximum voltage sent to motors (mV). VEX V5 motors accept -12000 to 12000.
const MAX_VOLTAGE: f64 = 12000.0;

/// Tank drive: left stick controls left motors, right stick controls right motors.
///
/// `left_axis` and `right_axis` should be in `[-1.0, 1.0]`.
pub fn tank_drive(
    left_motors: &mut [&mut Motor],
    right_motors: &mut [&mut Motor],
    left_axis: f64,
    right_axis: f64,
) {
    let lv = left_axis.clamp(-1.0, 1.0) * MAX_VOLTAGE;
    let rv = right_axis.clamp(-1.0, 1.0) * MAX_VOLTAGE;

    for m in left_motors.iter_mut() {
        let _ = m.set_voltage(lv);
    }
    for m in right_motors.iter_mut() {
        let _ = m.set_voltage(rv);
    }
}

/// Arcade drive: one stick for forward/back, another for left/right turn.
///
/// Both axes should be in `[-1.0, 1.0]`.
pub fn arcade_drive(
    left_motors: &mut [&mut Motor],
    right_motors: &mut [&mut Motor],
    throttle: f64,
    steering: f64,
) {
    let t = throttle.clamp(-1.0, 1.0);
    let s = steering.clamp(-1.0, 1.0);

    let lv = (t + s).clamp(-1.0, 1.0) * MAX_VOLTAGE;
    let rv = (t - s).clamp(-1.0, 1.0) * MAX_VOLTAGE;

    for m in left_motors.iter_mut() {
        let _ = m.set_voltage(lv);
    }
    for m in right_motors.iter_mut() {
        let _ = m.set_voltage(rv);
    }
}

/// Curvature drive (cheesy drive): constant curvature turning, better for
/// high-speed maneuvers than arcade drive.
///
/// - `throttle` in `[-1.0, 1.0]`
/// - `curvature` in `[-1.0, 1.0]` (positive = turn right)
pub fn curvature_drive(
    left_motors: &mut [&mut Motor],
    right_motors: &mut [&mut Motor],
    throttle: f64,
    curvature: f64,
) {
    let t = throttle.clamp(-1.0, 1.0);
    let c = curvature.clamp(-1.0, 1.0);

    let lv = (t + t.abs() * c).clamp(-1.0, 1.0) * MAX_VOLTAGE;
    let rv = (t - t.abs() * c).clamp(-1.0, 1.0) * MAX_VOLTAGE;

    for m in left_motors.iter_mut() {
        let _ = m.set_voltage(lv);
    }
    for m in right_motors.iter_mut() {
        let _ = m.set_voltage(rv);
    }
}
