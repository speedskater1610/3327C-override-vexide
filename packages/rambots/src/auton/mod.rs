//! Autonomous route system.
//!
//! Provides a composable [`Route`] type that runs a sequence of [`Action`]s.
//! Actions can be sequential, parallel, or conditional.
//!
//! # Quick-start
//! ```rust
//! use rambots::auton::{Route, Action};
//!
//! async fn my_auton(robot: &mut Robot) {
//!     let route = Route::new()
//!         .then(Action::drive_forward(500.0, 1.0))
//!         .then(Action::turn_to_heading(90.0, 1.5))
//!         .then(Action::intake_for_ms(800));
//!
//!     route.execute(robot).await;
//! }
//! ```

pub mod action;
pub mod route;
pub mod timer;

pub use action::Action;
pub use route::Route;
pub use timer::ActionTimer;
