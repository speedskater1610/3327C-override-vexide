//! # Rambots Override 2026-27 - Robot Entry Point
//!
//! ## Hardware layout
//!
//! ### Drivetrain (tank, arcade-controlled)
//! Each side has **three** motors sharing a common geartrain:
//!
//! | Motor | Cartridge | Gear on motor | Gear on wheel axle | Effective wheel RPM |
//! |---|---|---|---|---|
//! | Left/Right Front | 600 RPM (green) | 36T | 60T | 360 RPM |
//! | Left/Right Mid   | 600 RPM (green) | 36T | 60T | 360 RPM |
//! | Left/Right 5.5W  | 200 RPM (red)   | compound-geared to match | - | 360 RPM |
//!
//! The 5.5W motor is compound-geared so it runs at the same final wheel speed
//! as the 600 RPM motors. All three share mechanical load equally.
//!
//! ### Tracking wheels
//! Two forward-facing parallel encoders (left / right) + one sideways encoder.
//! All use 2-inch omni wheels on AMT102-V quadrature encoders (8192 ticks/rev).
//!
//! ### IMUs
//! Two V5 inertial sensors mounted one on top of the other, rotated 180deg
//! relative to each other. Fused with [`ImuFusion`] for noise rejection.
//!
//! ### Distance sensors
//! Four VEX V5 distance sensors pointing North (forward), East (right),
//! South (rear), and West (left) for wall-based localisation correction.
//!
//! ## Port map
//!
//! | Port | Device |
//! |---|---|
//! | 1  | IMU A (normal orientation) |
//! | 2  | IMU B (inverted / mirrored) |
//! | 3  | Distance sensor - North |
//! | 4  | Distance sensor - East |
//! | 5  | Distance sensor - South |
//! | 6  | Distance sensor - West |
//! | 7  | Left tracking wheel encoder (top ADI port = A, bottom = B) |
//! | 8  | Right tracking wheel encoder (top ADI port = C, bottom = D) |
//! | 9  | Strafe tracking wheel encoder (top ADI port = E, bottom = F) |
//! | 11 | Left front motor (600 RPM, forward) |
//! | 12 | Left mid motor (600 RPM, forward) |
//! | 13 | Left 5.5W motor (200 RPM, forward) |
//! | 18 | Right 5.5W motor (200 RPM, reverse) |
//! | 19 | Right mid motor (600 RPM, reverse) |
//! | 20 | Right front motor (600 RPM, reverse) |

#![no_std]
#![no_main]
extern crate alloc;

use core::time::Duration;

use rambots::{
    drivetrain::{
        config::{
            DRIVE_GEAR_RATIO,
            STICK_DEADBAND,
            STEERING_SENSITIVITY,
            TRACK_WIDTH_IN,
            TRACKING_WHEEL_RADIUS_IN,
        },
        distance_localizer::DistanceLocalizer,
        imu_fusion::ImuFusion,
        odometry::{Odometry, Pose},
    },
    hardware::encoder::Amt102V,
    logger::RobotLogger,
};
use log::{info, LevelFilter};
use vexide::{
    adi::{encoder::AdiEncoder, AdiPort},
    devices::smart::{
        distance::DistanceSensor,
        imu::InertialSensor,
        motor::{Gearset, Motor},
        rotation::RotationSensor,
    },
    prelude::*,
};

mod routes;

// Logger

static LOGGER: RobotLogger = RobotLogger;

/// All robot hardware in one place.
///
/// Sub-systems (intake, lift, claw) should be added here as the robot is built.
pub struct Robot {
    // UI
    pub controller: Controller,
    pub display: Display,

    // Drive motors - LEFT side
    /// 600 RPM, forward, 36T->60T gearing
    pub left_front: Motor,
    /// 600 RPM, forward, 36T->60T gearing
    pub left_mid: Motor,
    /// 5.5W (200 RPM), compound-geared to same final speed as the 600s
    pub left_55w: Motor,

    // Drive motors - RIGHT side
    /// 5.5W (200 RPM), compound-geared to same final speed as the 600s
    pub right_55w: Motor,
    /// 600 RPM, reversed, 36T->60T gearing
    pub right_mid: Motor,
    /// 600 RPM, reversed, 36T->60T gearing
    pub right_front: Motor,

    // Localisation
    /// Dual-IMU heading fuser (IMU A normal, IMU B inverted)
    pub imu: ImuFusion,
    /// Forward-facing left tracking wheel (2 in omni, AMT102-V)
    pub enc_left: AdiEncoder<8192>,
    /// Forward-facing right tracking wheel (2 in omni, AMT102-V)
    pub enc_right: AdiEncoder<8192>,
    /// Sideways (strafe) tracking wheel (2 in omni, AMT102-V)
    pub enc_strafe: AdiEncoder<8192>,
    /// Four-direction distance sensor array
    pub dist: DistanceLocalizer,
    /// Odometry state - updated every 5 ms
    pub odom: Odometry,
}

impl Robot {
    // Drive helpers

    /// Set left-side drive voltage (mV, + or -12 000).
    pub fn set_left_voltage(&mut self, mv: f64) {
        let _ = self.left_front.set_voltage(mv);
        let _ = self.left_mid.set_voltage(mv);
        let _ = self.left_55w.set_voltage(mv);
    }

    /// Set right-side drive voltage (mV, + or -12 000).
    pub fn set_right_voltage(&mut self, mv: f64) {
        let _ = self.right_front.set_voltage(mv);
        let _ = self.right_mid.set_voltage(mv);
        let _ = self.right_55w.set_voltage(mv);
    }

    /// Stop all drive motors (coast).
    pub fn stop_drive(&mut self) {
        self.set_left_voltage(0.0);
        self.set_right_voltage(0.0);
    }

    // Odometry helpers

    /// Tick the odometry tracker with the latest encoder and IMU readings.
    ///
    /// Call every 5-10 ms.
    pub fn update_odom(&mut self) {
        let l = self.enc_left.position()
            .map(|p| p.as_degrees())
            .unwrap_or(0.0);
        let r = self.enc_right.position()
            .map(|p| p.as_degrees())
            .unwrap_or(0.0);
        let s = self.enc_strafe.position()
            .map(|p| p.as_degrees())
            .unwrap_or(0.0);

        let imu_heading = self.imu.heading_degrees();

        self.odom.update(l, r, s, imu_heading);
    }

    /// Current robot pose.
    pub fn pose(&self) -> Pose {
        self.odom.pose
    }

    /// Attempt to correct odometry position using distance-sensor wall readings.
    ///
    /// Only corrects axes where both opposing sensors have valid readings.
    pub fn apply_distance_correction(&mut self) {
        if let Some(x) = self.dist.x_estimate() {
            self.odom.pose.x = x;
        }
        if let Some(y) = self.dist.y_estimate() {
            self.odom.pose.y = y;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    // Logger
    LOGGER
        .init(LevelFilter::Debug)
        .expect("Failed to initialise logger");

    // Build robot 
    let mut robot = Robot {
        controller: peripherals.primary_controller,
        display: peripherals.display,

        // Drive - LEFT (all forward)
        left_front: Motor::new(peripherals.port_11, Gearset::Green,  Direction::Forward),
        left_mid:   Motor::new(peripherals.port_12, Gearset::Green,  Direction::Forward),
        left_55w:   Motor::new(peripherals.port_13, Gearset::Red,    Direction::Forward),

        // Drive - RIGHT (all reversed)
        right_55w:  Motor::new(peripherals.port_18, Gearset::Red,    Direction::Reverse),
        right_mid:  Motor::new(peripherals.port_19, Gearset::Green,  Direction::Reverse),
        right_front: Motor::new(peripherals.port_20, Gearset::Green, Direction::Reverse),

        // IMU fusion: port 1 = normal, port 2 = inverted
        imu: ImuFusion::new(
            InertialSensor::new(peripherals.port_1),
            InertialSensor::new(peripherals.port_2),
        ),

        enc_fwd:   AdiEncoder::new(
            peripherals.port_7.adi_a, // TODO: adjust port letters to match wiring
            peripherals.port_7.adi_b,
        ),
        enc_strafe: AdiEncoder::new(
            peripherals.port_7.adi_e,
            peripherals.port_7.adi_f,
        ),

        // Distance sensors
        dist: DistanceLocalizer::new(
            DistanceSensor::new(peripherals.port_3),  // North (forward)
            DistanceSensor::new(peripherals.port_4),  // East  (right)
            DistanceSensor::new(peripherals.port_5),  // South (rear)
            DistanceSensor::new(peripherals.port_6),  // West  (left)
        ),

        // Odometry - track_width is the distance between the L/R tracking wheel
        // contact patches (centre-to-centre, inches).
        odom: Odometry::new(TRACK_WIDTH_IN),
    };

    // Calibrate IMUs
    info!("Calibrating IMUs...");
    robot.imu.calibrate().await;
    info!("IMU calibration complete.");

    // Set starting pose
    // Adjust x, y, heading to match your starting tile.
    // 0deg = facing away from your alliance wall (positive Y direction).
    robot.odom.set_pose(Pose::new(24.0, 24.0, 0.0));

    info!("Robot ready. Starting competition loop.");

    // Competition state machine
    loop {
        match vexide::competition::status() {
            CompetitionStatus::Autonomous => {
                autonomous(&mut robot).await;
            }
            CompetitionStatus::Driver => {
                driver_control(&mut robot).await;
            }
            _ => {
                // Disabled or no competition switch - keep odom ticking
                robot.update_odom();
                vexide::time::sleep(Duration::from_millis(5)).await;
            }
        }
    }
}

// Autonomous
async fn autonomous(robot: &mut Robot) {
    info!("Autonomous start");

    // Dispatch to the route selector in routes/mod.rs
    routes::run_autonomous(robot).await;

    info!("Autonomous end");
}

// Driver control

/// Arcade drive + odometry tick.
///
/// # Arcade drive mapping
///
/// ```text
///  Left stick Y  -> forward / backward
///  Right stick X -> turn left / right
///
///  left_voltage  = throttle + steering * STEERING_SENSITIVITY
///  right_voltage = throttle - steering * STEERING_SENSITIVITY
/// ```
///
/// A dead-band of [`STICK_DEADBAND`] is applied to both axes before
/// processing to eliminate drift from stick centre slop.
async fn driver_control(robot: &mut Robot) {
    info!("Driver control start");

    loop {
        // Odometry tick
        robot.update_odom();

        // Optionally apply distance-sensor correction once per second or on
        // a button press - uncomment when tuned:
        // robot.apply_distance_correction(); // TODO: see if driver likes this

        // Controller input
        let state = robot.controller.state().unwrap_or_default();

        // Raw stick values in [-127, 127]; normalise to [-1.0, 1.0]
        let raw_throttle = state.left_stick.y()  as f64 / 127.0;
        let raw_steering = state.right_stick.x() as f64 / 127.0;

        // Dead-band
        let throttle = apply_deadband(raw_throttle, STICK_DEADBAND);
        let steering  = apply_deadband(raw_steering,  STICK_DEADBAND);

        // Arcade mix with steering sensitivity
        let left  = (throttle + steering * STEERING_SENSITIVITY).clamp(-1.0, 1.0);
        let right = (throttle - steering * STEERING_SENSITIVITY).clamp(-1.0, 1.0);

        robot.set_left_voltage(left  * 12_000.0);
        robot.set_right_voltage(right * 12_000.0);

        // Debug telemetry (every ~500 ms to avoid log spam)
        // Uncomment to see live pose on the serial terminal:
        //
        // static TICK: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
        // let t = TICK.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        // if t % 50 == 0 {
        //     let p = robot.pose();
        //     log::debug!("Pose  x={:.2} in  y={:.2} in  h={:.1}deg",
        //                 p.x, p.y, p.heading_degrees());
        // }

        // Exit condition
        if !matches!(vexide::competition::status(), CompetitionStatus::Driver) {
            robot.stop_drive();
            break;
        }

        vexide::time::sleep(Duration::from_millis(10)).await;
    }
}

// utils

/// Apply a symmetric dead-band to `value`.
/// Returns 0.0 if `|value| < threshold`, otherwise rescales the remaining
/// range to [0, 1] so there is no jump at the edge of the dead-band.
#[inline]
fn apply_deadband(value: f64, threshold: f64) -> f64 {
    if value.abs() < threshold {
        0.0
    } else {
        // Rescale so the output is continuous from 0 at the edge of the
        // dead-band to + or -1.0 at full deflection.
        let sign = value.signum();
        sign * (value.abs() - threshold) / (1.0 - threshold)
    }
}
