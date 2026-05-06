use std::time::Duration;

use 3327C::{
    motion::basic::BasicExt,
    subsystems::intake::{ElementColor, HoodPosition, IntakeStage},
};
use evian::{
    motion::{Basic, Seeking},
    prelude::*,
};
use futures::future::join;
use vexide::time::sleep;

use crate::Robot;

impl Bot {
    pub async fn skills(&mut self) {
        let drivetrain = &mut self.drivetrain;
        
        let mut basic = Basic {
            linear_controller: Bot::LINEAR_PID,
            angular_controller: Bot::ANGUALR_PID,
            linear_tolerances: Bot::LINEAR_TOLERANCES,
            angular_tolerances: Bot::ANGULAR_TOLERANCES,
            timeout: Some(Duration::from_secs(5)),
        };
        let mut seeking = Seeking {
            linear_controller: Bot::LINEAR_PID,
            lateral_controller: Bot::LATERAL_PID,
            tolerances: Tolerances::new()
                                    .error(1.0)
                                    .duration(Duration::from_millis(100)),

            timeout: Some(Duration::from_secs(3)),
        };

        drivetrain.tracking.set_heading(180.0.deg());

        basic.drive_distance_at_heading(drivetrain, 33.5, 180.0.deg()).await;
        basic.turn_to_heading(drivetrain, 270.0.deg()).await;

        return;
    }
}