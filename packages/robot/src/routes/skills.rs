//! Skills autonomous route.
//!
//! Full 60-second skills run.  Edit the action sequence to match your
//! robot's capabilities and the field layout for Override 2026-27.

use rambots::auton::{Action, Route};
use log::info;

use crate::Robot;

/// Execute the skills route.
pub async fn run(robot: &mut Robot) {
    info!("Starting SKILLS autonomous");

    let route = Route::new("skills")
        // Score pre-load
        .then(Action::intake_for(400))
        // Drive to first scoring zone
        .then(Action::drive_forward(600.0, 0.75, 3000))
        .then(Action::turn_to_heading(90.0, 1500))
        .then(Action::drive_forward(300.0, 0.6, 2000))
        // Pick up ring stack
        .then(Action::intake_for(600))
        // Return to goal
        .then(Action::drive_backward(300.0, 0.6, 2000))
        .then(Action::turn_to_heading(0.0, 1500))
        .then(Action::drive_forward(800.0, 0.8, 3500))
        // Score
        .then(Action::lift_to(2000.0, 2000))
        .then(Action::outtake_for(500))
        .then(Action::lift_to(0.0, 2000));

    execute_route(route, robot).await;

    info!("SKILLS autonomous complete");
}

/// Interpret and execute a [`Route`] against the robot hardware.
///
/// This is the "executor" that maps [`Action`] variants to real hardware calls.
async fn execute_route(route: Route, robot: &mut Robot) {
    use rambots::auton::Action::*;
    use vexide::time::{sleep, Duration};

    for action in route.actions {
        match action {
            Wait { duration_ms } => {
                sleep(Duration::from_millis(duration_ms)).await;
            }
            IntakeFor { duration_ms } => {
                // TODO: call robot.intake.intake() when intake subsystem is wired up
                sleep(Duration::from_millis(duration_ms)).await;
            }
            OuttakeFor { duration_ms } => {
                sleep(Duration::from_millis(duration_ms)).await;
            }
            DriveForward { distance_mm, power, timeout_ms } => {
                // TODO: implement odometry-based drive-straight
                let _ = (distance_mm, power);
                sleep(Duration::from_millis(timeout_ms.min(3000))).await;
            }
            DriveBackward { distance_mm, power, timeout_ms } => {
                let _ = (distance_mm, power);
                sleep(Duration::from_millis(timeout_ms.min(3000))).await;
            }
            TurnToHeading { heading_deg, timeout_ms } => {
                // TODO: implement IMU-based turn-to-heading with PID
                let _ = heading_deg;
                sleep(Duration::from_millis(timeout_ms.min(2000))).await;
            }
            TurnByAngle { angle_deg, timeout_ms } => {
                let _ = angle_deg;
                sleep(Duration::from_millis(timeout_ms.min(2000))).await;
            }
            LiftTo { ticks, timeout_ms } => {
                let _ = ticks;
                sleep(Duration::from_millis(timeout_ms.min(3000))).await;
            }
            ClawOpen | ClawClose | ClawToggle => {
                // TODO: wire up claw subsystem
            }
            Sequence(actions) => {
                let inner = Route::new("inner").then_all(actions);
                execute_route(inner, robot).await;
            }
            // Parallel / Race would need vexide::task::spawn or select!
            _ => {}
        }
    }
}
