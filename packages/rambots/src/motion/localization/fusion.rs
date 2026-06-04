//! # Fused Localizer — MCL + EKF running in parallel
//!
//! ## Why run both?
//!
//! | Property | MCL (particle filter) | EKF |
//! |---|---|---|
//! | Global convergence | ✓ can recover from anywhere | ✗ diverges if linearisation is wrong |
//! | Smooth output | ✗ noisy weighted mean | ✓ Gaussian, differentiable |
//! | CPU cost | O(N) per cycle | O(1) per cycle |
//! | Large corrections | ✓ handles naturally | Can lose track |
//!
//! [`LocalizerFusion`] runs both in every update cycle:
//!
//! * The **EKF** provides the primary pose estimate for closed-loop control
//!   (smooth, low-latency).
//! * The **MCL** provides a ground-truth check. If the MCL and EKF estimates
//!   diverge by more than [`LocalizerConfig::recovery_threshold_in`], the EKF
//!   is re-seeded from the MCL estimate with inflated covariance ("recovery
//!   injection").
//!
//! ## Sensor observation bundle
//!
//! Pack all sensor data into a [`SensorObs`] and pass it to [`LocalizerFusion::update`].
//! Fields are `Option` — set to `None` if a sensor is unavailable or out of range.
//!
//! ## Call pattern
//!
//! ```rust
//! use rambots::localization::fusion::{LocalizerFusion, LocalizerConfig, SensorObs};
//! use rambots::localization::mcl::MclConfig;
//! use rambots::localization::ekf::EkfConfig;
//! use rambots::drivetrain::odometry::Pose;
//!
//! let mut localizer = LocalizerFusion::new(
//!     Pose::new(24.0, 24.0, 0.0),
//!     LocalizerConfig::default(),
//!     MclConfig::default(),
//!     EkfConfig::default(),
//! );
//!
//! // In your 10 ms control loop:
//! let obs = SensorObs {
//!     imu_heading_rad: imu_fusion.heading_degrees().map(|d| -d.to_radians()),
//!     north_in:  distance_localizer.north().field_position_in.map(|p| FIELD_SIZE_IN - p),
//!     south_in:  distance_localizer.south().field_position_in,
//!     east_in:   distance_localizer.east().field_position_in.map(|p| FIELD_SIZE_IN - p),
//!     west_in:   distance_localizer.west().field_position_in,
//! };
//!
//! localizer.update(delta_fwd, delta_lat, delta_rot, obs);
//! let pose = localizer.pose();  // best estimate
//! ```

use crate::drivetrain::odometry::Pose;
use crate::localization::{
    ekf::{Ekf, EkfConfig, SensorDir},
    mcl::{Mcl, MclConfig, Observation},
};

// ─────────────────────────────────────────────────────────────────────────────
// Sensor observation bundle
// ─────────────────────────────────────────────────────────────────────────────

/// All sensor inputs for one localization cycle.
///
/// All fields are `Option` — set to `None` for unavailable / out-of-range sensors.
/// Distances are in **inches** from the sensor face to the nearest wall in that
/// direction.
#[derive(Debug, Clone, Copy, Default)]
pub struct SensorObs {
    /// Fused IMU heading in **radians** (CCW positive, standard math convention).
    /// Convert from VEX degrees: `heading_rad = -(vex_degrees * PI / 180.0)`
    pub imu_heading_rad: Option<f64>,

    /// North sensor: inches from sensor face to north wall.
    pub north_in: Option<f64>,
    /// East sensor: inches from sensor face to east wall.
    pub east_in:  Option<f64>,
    /// South sensor: inches from sensor face to south wall.
    pub south_in: Option<f64>,
    /// West sensor: inches from sensor face to west wall.
    pub west_in:  Option<f64>,
}

impl SensorObs {
    /// Count how many distance sensors have valid readings.
    pub fn valid_distance_count(&self) -> usize {
        [self.north_in, self.east_in, self.south_in, self.west_in]
            .iter()
            .filter(|v| v.is_some())
            .count()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Fusion config
// ─────────────────────────────────────────────────────────────────────────────

/// High-level configuration for the fused localizer.
#[derive(Debug, Clone, Copy)]
pub struct LocalizerConfig {
    /// If the MCL and EKF estimates differ by more than this (inches),
    /// re-seed the EKF from the MCL estimate.
    pub recovery_threshold_in: f64,

    /// EKF positional variance used when injecting from MCL (in²).
    /// Larger = EKF trusts the re-seed less; smaller = trusts it more.
    pub ekf_recovery_pos_variance: f64,

    /// EKF heading variance used when injecting from MCL (rad²).
    pub ekf_recovery_heading_variance: f64,

    /// Maximum valid distance sensor reading (in).
    /// Readings above this are discarded before passing to either filter.
    pub max_sensor_range_in: f64,

    /// Require at least this many valid distance readings before running
    /// the sensor update step. Set to 0 to always run (even with no sensors).
    pub min_sensors_for_update: usize,
}

impl Default for LocalizerConfig {
    fn default() -> Self {
        Self {
            recovery_threshold_in:        6.0,
            ekf_recovery_pos_variance:    4.0,  // 2 in std dev
            ekf_recovery_heading_variance: 0.01, // ~5.7° std dev
            max_sensor_range_in:           80.0,
            min_sensors_for_update:        1,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Fusion diagnostics
// ─────────────────────────────────────────────────────────────────────────────

/// Diagnostic snapshot from the last [`LocalizerFusion::update`] call.
#[derive(Debug, Clone, Copy, Default)]
pub struct LocalizerDiag {
    /// EKF positional standard deviation (inches).
    pub ekf_pos_std_in: f64,
    /// MCL effective sample size (0 = degenerate, N = uniform).
    pub mcl_ess: f64,
    /// MCL X standard deviation (inches) — measure of particle spread.
    pub mcl_x_std_in: f64,
    /// MCL Y standard deviation (inches).
    pub mcl_y_std_in: f64,
    /// Distance between EKF and MCL estimates (inches).
    pub ekf_mcl_divergence_in: f64,
    /// Whether a recovery injection happened this cycle.
    pub recovery_triggered: bool,
    /// Number of valid distance sensor readings used.
    pub valid_sensor_count: usize,
}

// ─────────────────────────────────────────────────────────────────────────────
// LocalizerFusion
// ─────────────────────────────────────────────────────────────────────────────

/// Combined MCL + EKF localizer.
///
/// Call [`update`][Self::update] every control loop cycle (10 ms), then read
/// [`pose`][Self::pose] for the best current estimate.
pub struct LocalizerFusion {
    pub ekf:    Ekf,
    pub mcl:    Mcl,
    pub config: LocalizerConfig,
    pub diag:   LocalizerDiag,
}

impl LocalizerFusion {
    // ── Construction ──────────────────────────────────────────────────────────

    /// Create a new fused localizer starting at `initial_pose`.
    pub fn new(
        initial_pose:  Pose,
        fusion_config: LocalizerConfig,
        mcl_config:    MclConfig,
        ekf_config:    EkfConfig,
    ) -> Self {
        let mut mcl = Mcl::new(mcl_config);
        mcl.set_pose(initial_pose, 2.0, 0.1); // 2 in pos std, 0.1 rad heading std

        let ekf = Ekf::new(initial_pose, ekf_config);

        Self {
            ekf,
            mcl,
            config: fusion_config,
            diag:   LocalizerDiag::default(),
        }
    }

    /// Reset both filters to a known pose (e.g. start of autonomous).
    pub fn set_pose(&mut self, pose: Pose) {
        self.mcl.set_pose(pose, 1.0, 0.05);
        self.ekf.set_pose(
            pose,
            self.config.ekf_recovery_pos_variance,
            self.config.ekf_recovery_heading_variance,
        );
        log::info!(
            "LocalizerFusion: pose reset to ({:.2}, {:.2}, {:.1}°)",
            pose.x, pose.y, pose.heading_degrees()
        );
    }

    // ── Main update ───────────────────────────────────────────────────────────

    /// Run one full localization cycle.
    ///
    /// # Parameters
    ///
    /// - `delta_fwd_in`   – forward displacement since last call (inches)
    /// - `delta_lat_in`   – lateral (strafe) displacement since last call (inches)
    /// - `delta_rot_rad`  – rotation since last call (radians, CCW positive)
    /// - `obs`            – sensor readings this cycle
    pub fn update(
        &mut self,
        delta_fwd_in:  f64,
        delta_lat_in:  f64,
        delta_rot_rad: f64,
        obs:           SensorObs,
    ) {
        let cfg = self.config;
        let max_r = cfg.max_sensor_range_in;

        // ── Filter observations ───────────────────────────────────────────────

        let north = obs.north_in.filter(|&r| r > 0.0 && r < max_r);
        let east  = obs.east_in .filter(|&r| r > 0.0 && r < max_r);
        let south = obs.south_in.filter(|&r| r > 0.0 && r < max_r);
        let west  = obs.west_in .filter(|&r| r > 0.0 && r < max_r);
        let valid_count = [north, east, south, west].iter().filter(|v| v.is_some()).count();

        // ── EKF predict ───────────────────────────────────────────────────────

        self.ekf.predict(delta_fwd_in, delta_lat_in, delta_rot_rad);

        // ── EKF sensor updates ────────────────────────────────────────────────

        if valid_count >= cfg.min_sensors_for_update {
            if let Some(r) = north { self.ekf.update_distance(SensorDir::North, r); }
            if let Some(r) = east  { self.ekf.update_distance(SensorDir::East,  r); }
            if let Some(r) = south { self.ekf.update_distance(SensorDir::South, r); }
            if let Some(r) = west  { self.ekf.update_distance(SensorDir::West,  r); }
        }

        // ── EKF IMU update ────────────────────────────────────────────────────

        if let Some(h) = obs.imu_heading_rad {
            self.ekf.update_imu_heading(h);
        }

        // ── MCL update ────────────────────────────────────────────────────────

        let mcl_obs = Observation {
            north_in:    north,
            east_in:     east,
            south_in:    south,
            west_in:     west,
            heading_rad: obs.imu_heading_rad,
        };
        self.mcl.update(delta_fwd_in, delta_lat_in, delta_rot_rad, &mcl_obs);

        // ── Recovery injection ────────────────────────────────────────────────

        let ekf_pose = self.ekf.pose();
        let mcl_pose = self.mcl.estimate;
        let divergence = ekf_pose.distance_to(mcl_pose);

        let recovery = divergence > cfg.recovery_threshold_in;
        if recovery {
            log::warn!(
                "LocalizerFusion: EKF/MCL diverged {:.2} in — re-seeding EKF from MCL",
                divergence
            );
            self.ekf.set_pose(
                mcl_pose,
                cfg.ekf_recovery_pos_variance,
                cfg.ekf_recovery_heading_variance,
            );
        }

        // ── Update diagnostics ────────────────────────────────────────────────

        self.diag = LocalizerDiag {
            ekf_pos_std_in:       self.ekf.position_std_dev(),
            mcl_ess:              self.mcl.effective_sample_size(),
            mcl_x_std_in:         self.mcl.x_std_dev(),
            mcl_y_std_in:         self.mcl.y_std_dev(),
            ekf_mcl_divergence_in: divergence,
            recovery_triggered:   recovery,
            valid_sensor_count:   valid_count,
        };
    }

    // ── Accessors ─────────────────────────────────────────────────────────────

    /// Best pose estimate (from EKF — smooth and low-latency).
    ///
    /// When the EKF is uncertain (high `diag.ekf_pos_std_in`), consider using
    /// `mcl_pose()` instead.
    pub fn pose(&self) -> Pose {
        self.ekf.pose()
    }

    /// MCL estimate (more robust to sudden large corrections).
    pub fn mcl_pose(&self) -> Pose {
        self.mcl.estimate
    }

    /// Returns the EKF pose when its uncertainty is low, otherwise the MCL pose.
    ///
    /// Threshold: if EKF std dev > `threshold_in`, use MCL.
    pub fn best_pose(&self, threshold_in: f64) -> Pose {
        if self.ekf.position_std_dev() < threshold_in {
            self.pose()
        } else {
            self.mcl_pose()
        }
    }

    /// Log a one-line status summary via the `log` crate (DEBUG level).
    pub fn log_status(&self) {
        let p = self.pose();
        let d = self.diag;
        log::debug!(
            "Localizer | pose ({:6.2}, {:6.2}, {:6.1}°) | \
             EKF σ {:.2} in | MCL ESS {:.0}/{} | \
             Δ EKF/MCL {:.2} in | sensors {}{}",
            p.x, p.y, p.heading_degrees(),
            d.ekf_pos_std_in,
            d.mcl_ess, self.mcl.config.num_particles,
            d.ekf_mcl_divergence_in,
            d.valid_sensor_count,
            if d.recovery_triggered { " [RECOVERY]" } else { "" },
        );
    }
}
