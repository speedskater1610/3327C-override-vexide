//! Right-side qualifier autonomous route.

use rambots::auton::{Action, Route};
use log::info;

use crate::Robot;

pub async fn run(robot: &mut Robot) {
    info!("Starting RIGHT QUAL autonomous");

    let _route = Route::new("right-qual")
        .then(Action::intake_for(300))
        .then(Action::drive_forward(400.0, 0.7, 2000))
        .then(Action::turn_to_heading(-45.0, 1500))
        .then(Action::drive_forward(300.0, 0.65, 2000))
        .then(Action::outtake_for(400));

    let _ = robot;

    info!("RIGHT QUAL autonomous complete");
}
