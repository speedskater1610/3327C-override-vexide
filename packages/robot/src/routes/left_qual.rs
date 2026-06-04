//! Left-side qualifier autonomous route.

use rambots::auton::{Action, Route};
use log::info;

use crate::Robot;

pub async fn run(robot: &mut Robot) {
    info!("Starting LEFT QUAL autonomous");

    let _route = Route::new("left-qual")
        .then(Action::intake_for(300))
        .then(Action::drive_forward(400.0, 0.7, 2000))
        .then(Action::turn_to_heading(45.0, 1500))
        .then(Action::drive_forward(300.0, 0.65, 2000))
        .then(Action::outtake_for(400));

    // TODO: execute route via executor (see skills.rs)
    let _ = robot;

    info!("LEFT QUAL autonomous complete");
}
