//! # Localization Framework
//!
//! Robot pose estimation fusing odometry, dual IMUs, and four distance sensors
//! using two complementary algorithms running in parallel:
//!
//!
//! ## Monte Carlo Localisation (MCL / Particle Filter)
//!
//! See [`mcl`].
//!
//! MCL maintains a cloud of N hypothetical robot poses ("particles"), each with
//! a probability weight. Every cycle:
//!
//! 1. **Motion update** - propagate each particle forward using the odometry
//!    delta, plus Gaussian noise that models wheel slip and IMU drift.
//! 2. **Sensor update** - re-weight each particle according to how well the
//!    four distance sensor readings match what *that* particle would predict,
//!    using a Gaussian likelihood model.
//! 3. **Normalise** - divide all weights by their sum.
//! 4. **Resample** - use the low-variance (systematic) resampler to replace
//!    unlikely particles with copies of likely ones.
//! 5. **Estimate** - weighted mean of all particles.
//!
//! MCL is robust to sudden large corrections but is expensive (O(N) per cycle).
//! We run 150 particles by default - enough for VRC field accuracy.
//!
//! ## Extended Kalman Filter (EKF)
//!
//! See [`ekf`].
//!
//! The EKF maintains a single Gaussian estimate (mean pose + 3x3 covariance
//! matrix). Every cycle:
//!
//! 1. **Predict** - advance the state using the odometry motion model; inflate
//!    the covariance by process noise Q.
//! 2. **Update** - incorporate each valid distance-sensor measurement via the
//!    Kalman gain, shrinking the covariance when sensors agree.
//!
//! The EKF is cheap (O(1)) and gives a smooth, continuous estimate ideal for
//! closed-loop drive control. It can lose track after large sudden jumps.
//!
//! ## Fusion
//!
//! [`LocalizerFusion`] runs both in parallel. The MCL estimate is used to
//! re-seed the EKF whenever the two estimates diverge by more than a threshold
//! (indicating the EKF has drifted), giving robustness of MCL + smoothness of EKF.
//!
//! ## Coordinate convention
//!
//! * Origin (0, 0) = **south-west corner** of the field.
//! * +X = east (right side when facing north).
//! * +Y = north (forward from south wall).
//! * Heading: **radians**, CCW positive (standard math).
//!   Use [`Pose::heading_degrees`] for the VEX clockwise-degrees display value.
//! * Field size: **144 x 144 inches** (12 ft x 12 ft).

pub mod ekf;
pub mod mcl;
pub mod fusion;

pub use ekf::Ekf;
pub use mcl::{Mcl, MclConfig, Particle};
pub use fusion::{LocalizerFusion, LocalizerConfig, SensorObs};
