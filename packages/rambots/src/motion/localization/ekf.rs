//! # Extended Kalman Filter (EKF) for robot pose estimation
//!
//! ## Predict step (motion model)
//!
//! Given odometry inputs (delta forward, delta lateral, delta heading):
//!
//! ```text
//!  avg_h = theta + delta theta/2
//!
//!  X' = X + delta fwd x cos(avg_h) - delta lat x sin(avg_h)
//!  Y' = Y + delta fwd x sin(avg_h) + delta lat x cos(avg_h)
//!  theta' = theta + delta theta
//! ```
//!
//! ## Update step (measurement model)
//!
//! For each distance sensor pointing in direction D we get a measurement z.
//! The expected measurement is:
//!
//! ```text
//!  h_north(x) = FIELD - Y - OFFSET_N     H = [0, -1, 0]
//!  h_south(x) = Y - OFFSET_S             H = [0,  1, 0]
//!  h_east(x)  = FIELD - X - OFFSET_E     H = [-1, 0, 0]
//!  h_west(x)  = X - OFFSET_W             H = [ 1, 0, 0]
//! ```

use core::f64::consts::PI;

use crate::drivetrain::{
    config::{
        DIST_EAST_OFFSET_IN, DIST_NORTH_OFFSET_IN, DIST_SOUTH_OFFSET_IN,
        DIST_WEST_OFFSET_IN,
    },
    distance_localizer::FIELD_SIZE_IN,
    odometry::Pose,
};
use crate::localization::mcl::{angle_diff, wrap_angle};

// 3x3 matrix (row-major)
/// A 3x3 matrix stored in row-major order
/// We only need 3x3 ops for the EKF state, so this keeps the code no_std.
#[derive(Debug, Clone, Copy)]
struct Mat3([[f64; 3]; 3]);

impl Mat3 {
    const ZERO:     Self = Self([[0.0; 3]; 3]);
    const IDENTITY: Self = Self([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);

    fn get(&self, r: usize, c: usize) -> f64 { self.0[r][c] }
    fn set(&mut self, r: usize, c: usize, v: f64) { self.0[r][c] = v; }

    /// Matrix x matrix.
    fn mul(&self, rhs: &Mat3) -> Mat3 {
        let mut out = Mat3::ZERO;
        for i in 0..3 {
            for j in 0..3 {
                let mut s = 0.0;
                for k in 0..3 { s += self.0[i][k] * rhs.0[k][j]; }
                out.0[i][j] = s;
            }
        }
        out
    }

    /// Transpose.
    fn transpose(&self) -> Mat3 {
        let mut out = Mat3::ZERO;
        for i in 0..3 {
            for j in 0..3 { out.0[j][i] = self.0[i][j]; }
        }
        out
    }

    /// `A + B`.
    fn add(&self, rhs: &Mat3) -> Mat3 {
        let mut out = *self;
        for i in 0..3 { for j in 0..3 { out.0[i][j] += rhs.0[i][j]; } }
        out
    }

    /// Matrix x column vector (len 3).
    fn mul_vec(&self, v: [f64; 3]) -> [f64; 3] {
        let mut out = [0.0; 3];
        for i in 0..3 {
            for j in 0..3 { out[i] += self.0[i][j] * v[j]; }
        }
        out
    }

    /// Row vector (1x3) x matrix (3x3) -> row vector (1x3).
    fn premul_row(&self, h: [f64; 3]) -> [f64; 3] {
        let mut out = [0.0; 3];
        for j in 0..3 {
            for k in 0..3 { out[j] += h[k] * self.0[k][j]; }
        }
        out
    }

    /// `I - outer(k, h)`:  M[i][j] -= k[i]*h[j]
    fn sub_outer(mut self, k: [f64; 3], h: [f64; 3]) -> Mat3 {
        for i in 0..3 { for j in 0..3 { self.0[i][j] -= k[i] * h[j]; } }
        self
    }
}

// EKF noise parameters
/// Process and measurement noise tuning for the EKF.
#[derive(Debug, Clone, Copy)]
pub struct EkfConfig {
    /// Process noise variance for X (in^2).
    pub q_x: f64,
    /// Process noise variance for Y (in^2).
    pub q_y: f64,
    /// Process noise variance for heading (rad^2).
    pub q_theta: f64,

    /// Measurement noise variance for distance sensors (in^2).
    /// VEX V5 distance sensor std dev ~0.5-1.5 in -> variance 0.25-2.25 in^2.
    pub r_distance: f64,

    /// Measurement noise variance for IMU heading (rad^2).
    /// Dual fused IMU std dev ~0.5deg -> variance (0.5deg)^2 ~= 7.6e-5 rad^2
    pub r_imu_heading: f64,

    /// Maximum innovation (observed - expected) in inches before a distance
    /// reading is rejected as an outlier.
    pub outlier_gate_in: f64,
}

impl Default for EkfConfig {
    fn default() -> Self {
        Self {
            q_x:            0.01,   // ~0.1 in std dev per cycle
            q_y:            0.01,
            q_theta:        0.0001, // ~0.57deg std dev per cycle
            r_distance:     1.0,    // ~1 in std dev
            r_imu_heading:  0.0001, // ~0.57deg std dev (fused dual IMU)
            outlier_gate_in: 8.0,   // reject readings > 8 in from prediction
        }
    }
}

// EKF

/// Extended Kalman Filter for 2D robot pose estimation.
pub struct Ekf {
    /// State: [x_in, y_in, heading_rad]
    pub state: [f64; 3],

    /// 3x3 state covariance matrix.
    /// Diagonal elements = variance in X, Y, heading.
    /// Off-diagonal = cross-correlations.
    pub cov: Mat3,

    pub config: EkfConfig,
}

impl Ekf {
    /// Create a new EKF initialised at `initial_pose` with a high initial
    /// uncertainty (reflects that we don't know the exact starting pose).
    pub fn new(initial_pose: Pose, config: EkfConfig) -> Self {
        let mut cov = Mat3::ZERO;
        cov.set(0, 0, 25.0);   // 5 in std dev in X
        cov.set(1, 1, 25.0);   // 5 in std dev in Y
        cov.set(2, 2, 0.1);    // ~18deg std dev in heading

        Self {
            state: [initial_pose.x, initial_pose.y, initial_pose.heading],
            cov,
            config,
        }
    }

    /// Override with a known pose (e.g. from MCL recovery).
    /// Resets covariance to a tighter estimate.
    pub fn set_pose(&mut self, pose: Pose, pos_variance: f64, heading_variance: f64) {
        self.state = [pose.x, pose.y, pose.heading];
        self.cov = Mat3::ZERO;
        self.cov.set(0, 0, pos_variance);
        self.cov.set(1, 1, pos_variance);
        self.cov.set(2, 2, heading_variance);
    }

    // Predict step

    /// Propagate the state forward using the odometry motion model.
    ///
    /// - `delta_fwd_in`  - forward displacement (inches)
    /// - `delta_lat_in`  - lateral (strafe) displacement (inches)
    /// - `delta_rot_rad` - rotation (radians, CCW positive)
    pub fn predict(&mut self, delta_fwd_in: f64, delta_lat_in: f64, delta_rot_rad: f64) {
        let [x, y, theta] = self.state;
        let cfg = self.config;

        let avg_h = theta + delta_rot_rad / 2.0;
        let cos_h = avg_h.cos();
        let sin_h = avg_h.sin();

        // State prediction
        self.state[0] = x + delta_fwd_in * cos_h - delta_lat_in * sin_h;
        self.state[1] = y + delta_fwd_in * sin_h + delta_lat_in * cos_h;
        self.state[2] = wrap_angle(theta + delta_rot_rad);

        // Jacobian F of the motion model w.r.t. state
        // F = I + dg/dx
        let df_dtheta_x = -delta_fwd_in * sin_h - delta_lat_in * cos_h;
        let df_dtheta_y =  delta_fwd_in * cos_h - delta_lat_in * sin_h;

        let mut f = Mat3::IDENTITY;
        f.set(0, 2, df_dtheta_x);
        f.set(1, 2, df_dtheta_y);

        // Process noise Q (diagonal)
        let mut q = Mat3::ZERO;
        q.set(0, 0, cfg.q_x);
        q.set(1, 1, cfg.q_y);
        q.set(2, 2, cfg.q_theta);

        // P' = F x P x F^t + Q
        self.cov = f.mul(&self.cov).mul(&f.transpose()).add(&q);
    }

    // Update

    /// Incorporate a single distance sensor reading.
    ///
    /// `direction` selects which sensor model to use.
    /// `reading_in` is the raw distance in inches.
    pub fn update_distance(&mut self, direction: SensorDir, reading_in: f64) {
        let cfg = self.config;

        // Predicted measurement h(x)
        let (h_pred, h_jacobian) = self.distance_model(direction);

        // Innovation (scalar)
        let innovation = reading_in - h_pred;

        // Outlier gate
        if innovation.abs() > cfg.outlier_gate_in {
            log::debug!(
                "EKF: outlier gate rejected {:?} reading (innovation {:.2} in)",
                direction, innovation
            );
            return;
        }

        // S =  H x P x H^T + R  (scalar because H is 1x3)
        let hp = dot3(h_jacobian, self.cov.mul_vec(h_jacobian));
        let s  = hp + cfg.r_distance;

        if s.abs() < 1e-12 { return; }

        // Kalman gain K = P x H^T / S  (3x1 column vector)
        let ph_t = self.cov.mul_vec(h_jacobian); // P x H^T
        let k: [f64; 3] = [ph_t[0] / s, ph_t[1] / s, ph_t[2] / s];

        // State update
        self.state[0] = (self.state[0] + k[0] * innovation).clamp(0.0, FIELD_SIZE_IN);
        self.state[1] = (self.state[1] + k[1] * innovation).clamp(0.0, FIELD_SIZE_IN);
        self.state[2] = wrap_angle(self.state[2] + k[2] * innovation);

        // Covariance update: P = (I - K x H) x P
        self.cov = Mat3::IDENTITY.sub_outer(k, h_jacobian).mul(&self.cov);
    }

    /// Incorporate the fused IMU heading.
    ///
    /// `imu_heading_rad` - heading in radians (CCW positive).
    pub fn update_imu_heading(&mut self, imu_heading_rad: f64) {
        let cfg = self.config;

        // H = [0, 0, 1]
        let h: [f64; 3] = [0.0, 0.0, 1.0];

        // Innovation - must handle angle wrap-around
        let innovation = angle_diff(imu_heading_rad, self.state[2]);

        // S = H x P x H^T + R_imu
        let hp = dot3(h, self.cov.mul_vec(h));
        let s  = hp + cfg.r_imu_heading;

        if s.abs() < 1e-12 { return; }

        let ph_t = self.cov.mul_vec(h);
        let k: [f64; 3] = [ph_t[0] / s, ph_t[1] / s, ph_t[2] / s];

        self.state[0] += k[0] * innovation;
        self.state[1] += k[1] * innovation;
        self.state[2]  = wrap_angle(self.state[2] + k[2] * innovation);

        self.cov = Mat3::IDENTITY.sub_outer(k, h).mul(&self.cov);
    }

    // Accessors

    /// Current pose estimate.
    pub fn pose(&self) -> Pose {
        Pose {
            x:       self.state[0],
            y:       self.state[1],
            heading: self.state[2],
        }
    }

    /// Variance in X position (in^2). Square-root to get std dev
    pub fn var_x(&self) -> f64 { self.cov.get(0, 0) }
    /// Variance in Y position (in^2).
    pub fn var_y(&self) -> f64 { self.cov.get(1, 1) }
    /// Variance in heading (rad^2).
    pub fn var_theta(&self) -> f64 { self.cov.get(2, 2) }

    /// Combined positional uncertainty (inches)
    pub fn position_std_dev(&self) -> f64 {
        (self.var_x() + self.var_y()).sqrt()
    }

    // Internal: sensor models

    /// Returns `(predicted_reading_in, H_jacobian_row)` for a given direction.
    fn distance_model(&self, dir: SensorDir) -> (f64, [f64; 3]) {
        let [x, y, _] = self.state;
        match dir {
            SensorDir::North => (
                (FIELD_SIZE_IN - y - DIST_NORTH_OFFSET_IN).max(0.0),
                [0.0, -1.0, 0.0],
            ),
            SensorDir::South => (
                (y - DIST_SOUTH_OFFSET_IN).max(0.0),
                [0.0, 1.0, 0.0],
            ),
            SensorDir::East => (
                (FIELD_SIZE_IN - x - DIST_EAST_OFFSET_IN).max(0.0),
                [-1.0, 0.0, 0.0],
            ),
            SensorDir::West => (
                (x - DIST_WEST_OFFSET_IN).max(0.0),
                [1.0, 0.0, 0.0],
            ),
        }
    }
}

// Helpers
/// Which distance sensor to update.
#[derive(Debug, Clone, Copy)]
pub enum SensorDir { North, East, South, West }

/// Dot product of two 3-vectors.
#[inline]
fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
}
