//! Autonomous routes.
//!
//! Add new routes as sub-modules and register them in [`run_autonomous`].

pub mod skills;
pub mod left_qual;
pub mod right_qual;

use crate::Robot;

/// Select and execute the autonomous route for the current match.
///
/// Route selection can be controlled via the controller screen or a
/// physical selector switch — for now it defaults to the left qualifier route.
pub async fn run_autonomous(robot: &mut Robot) {
    // TODO: wire up a selector (controller buttons / display menu)
    left_qual::run(robot).await;
}
