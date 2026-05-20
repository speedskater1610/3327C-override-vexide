use std::time::{Duration, Instant};

use 3327C::{hardware::calibration::calibrate_imu, logger::RobotLogger, theme::THEME};
use evian::{
    control::loops::{AngularPid, Pid},
    math::Angle,
    prelude::*,
    tracking::{
        shared_motors,
        wheeled::{TrackingWheel, WheeledTracking},
    },
    drivetrain::model::{Arcade, Differential},
    motion::{Basic, Seeking}, 
};
use log::{LevelFilter, info};
use vexide::prelude::*;

pub mod routes;

struct Bot {
    controller: Controller,
    drivetrain: Drivetrain<Differential, WheeledTracking>,
}

impl Bot {
    // Robot measurements
    pub const TRACK_WIDTH: f64 = 11.5;
    pub const WHEEL_DIAMETER: f64 = 3.25;

    // Robot measurements
    pub const TRACKING_WHEEL_DIAMETER: f64 = 2.0;
    pub const SIDEWAYS_TRACKING_WHEEL_OFFSET: f64 = -2.5;

    // Controllers
    pub const LINEAR_PID: Pid = Pid::new(0.1, 0.001, 0.01, Some(3.0));
    pub const LATERAL_PID: Pid = Pid::new(0.09, 0.001, 0.004, Some(2.0));
    pub const ANGUALR_PID: AngularPid = AngularPid::new(
                                            3.0, 0.1, 0.175, 
                                            Some(Angle::from_degrees(5.0))
                                        );

    // Tolerances
    pub const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(1.0)
        .velocity(0.25)
        .duration(Duration::from_millis(15));
    pub const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(f64::to_radians(8.0))
        .velocity(0.05)
        .duration(Duration::from_millis(15));
}

impl Compete for Bot {
    async fn autonomous(&mut self) {
        let start = Instant::now();

        self.aura().await;

        info!("Route completed successfully in {:?}.", start.elapsed());
        info!(
            "Position: {} Heading: {}{}",
            self.drivetrain.tracking.position(),
            self.drivetrain.tracking.heading().as_degrees(),
            "\u{00B0}",
        );
    }

    async fn driver(&mut self) {
        loop {
            let state = self.controller.state().unwrap_or_default();

            _ = self
                .drivetrain
                .model
                .drive_arcade(state.left_stick.y(), state.left_stick.x());

            // controller
            // state.button_r1.is_pressed()

            sleep(Motor::WRITE_INTERVAL).await;
        }
    }
}

#[vexide::main(banner(theme = THEME))]
async fn main(peripherals: Peripherals) {
    RobotLogger.init(LevelFilter::Trace).unwrap();

    let mut controller = peripherals.primary_controller;
    let mut display = peripherals.display;

    let mut imu = InertialSensor::new(peripherals.port_13);

    calibrate_imu(&mut controller, &mut display, &mut imu).await;

    let left_motors = shared_motors![
        Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_2, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_3, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_4, Gearset::Blue, Direction::Forward),
    ];

    let right_motors = shared_motors![
        Motor::new(peripherals.port_5, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_6, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_7, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),
    ];

    let bot = Bot {
        controller,
        drivetrain: Drivetrain::new(
            Differential::from_shared(left_motors.clone(), right_motors.clone()),
            WheeledTracking::forward_only(
                (0.0, 0.0),
                90.0.deg(),
                [
                    TrackingWheel::new(left_motors, 2.75, 0.0, None),
                    TrackingWheel::new(right_motors, 2.75, 0.0, None),
                ],
                Some(imu),
            ),
        ),
    };

    bot.compete().await;
}