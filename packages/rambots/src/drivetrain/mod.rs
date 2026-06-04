//! Drivetrain module.
//!
//! Provides:
//! * [`config`] - all physical robot measurements (edit here to retune)
//! * [`imu_fusion`] - dual mirrored IMU heading fuser
//! * [`odometry`] - 2 wheel encoder odometry with offset compensation
//! * [`distance_localizer`] - 4 direction distance sensor wall localiser
//!
//! The actual motors and the arcade-drive loop live in `robot/src/main.rs`.

pub mod config;
pub mod distance_localizer;
pub mod imu_fusion;
pub mod odometry;

pub use distance_localizer::DistanceLocalizer;
pub use imu_fusion::ImuFusion;
pub use odometry::{Odometry, Pose};
