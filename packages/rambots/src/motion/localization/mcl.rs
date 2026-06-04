//! # Monte Carlo Localisation (Particle Filter)
//!
//! ## Algorithm overview
//!
//! ```text
//!  ┌──────────────────────────────────────────────────────────────────────┐
//!  │  Particles: Vec<Particle>  (each = pose hypothesis + weight)         │
//!  │                                                                      │
//!  │  Every call to update():                                             │
//!  │                                                                      │
//!  │  1. Motion update ─────────────────────────────────────────────────  │
//!  │     for p in particles:                                              │
//!  │         p.pose += odom_delta + sample(motion_noise)                 │
//!  │                                                                      │
//!  │  2. Sensor update ─────────────────────────────────────────────────  │
//!  │     for p in particles:                                              │
//!  │         for each valid distance reading:                             │
//!  │             expected = predicted_wall_distance(p.pose, direction)   │
//!  │             p.weight *= gaussian(observed, expected, sigma)         │
//!  │                                                                      │
//!  │  3. Normalise weights → sum to 1.0                                   │
//!  │                                                                      │
//!  │  4. Resample (low-variance / systematic resampler)                   │
//!  │     prevents particle degeneracy                                     │
//!  │                                                                      │
//!  │  5. Estimate = weighted mean of particles                            │
//!  └──────────────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Motion noise model
//!
//! We use a four-parameter model (from Probabilistic Robotics, Thrun et al.):
//!
//! ```text
//!  σ_trans  = α1·|Δtrans| + α2·|Δrot|
//!  σ_rot    = α3·|Δtrans| + α4·|Δrot|
//! ```
//!
//! α1–α4 are tunable constants in [`MclConfig`].
//!
//! ## Observation model
//!
//! For each distance sensor pointing in direction D (N/E/S/W):
//!
//! ```text
//!  expected_dist = distance_from_particle_to_wall_in_direction_D(p.pose)
//!               - sensor_offset_in_D
//!
//!  likelihood = exp(-0.5 × ((observed - expected) / σ_sensor)²)
//!             / (σ_sensor × √(2π))
//! ```
//!
//! σ_sensor is the expected standard deviation of VEX distance sensor readings.
//! At close range (~10 cm) it is roughly 3–8 mm; we default to 1.0 inch (25.4 mm).
//!
//! ## Resampler
//!
//! Low-variance (systematic) resampling:
//!
//! 1. Draw a single uniform random number r ∈ [0, 1/N).
//! 2. Step through the cumulative weight distribution N times at spacing 1/N.
//! 3. At each step, copy the particle whose cumulative weight bracket contains
//!    the current position.
//!
//! This preserves particle diversity better than independent multinomial sampling.

use alloc::vec::Vec;
use core::f64::consts::PI;

use crate::drivetrain::{
    config::{
        DIST_EAST_OFFSET_IN, DIST_NORTH_OFFSET_IN, DIST_SOUTH_OFFSET_IN,
        DIST_WEST_OFFSET_IN,
    },
    distance_localizer::FIELD_SIZE_IN,
};
use crate::drivetrain::odometry::Pose;

// ─────────────────────────────────────────────────────────────────────────────
// Particle
// ─────────────────────────────────────────────────────────────────────────────

/// A single pose hypothesis with an associated probability weight.
#[derive(Debug, Clone, Copy)]
pub struct Particle {
    pub pose:   Pose,
    pub weight: f64,
}

impl Particle {
    pub fn new(pose: Pose, weight: f64) -> Self {
        Self { pose, weight }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Config
// ─────────────────────────────────────────────────────────────────────────────

/// Tuning parameters for the MCL algorithm.
///
/// # Defaults
///
/// The defaults are a reasonable starting point for a VRC robot on a standard
/// 12 ft field. Tune by:
/// * Printing the particle cloud spread and watching it converge.
/// * Increasing α values if the filter loses track during fast turns.
/// * Decreasing `sensor_std_dev_in` if distance readings are very reliable.
#[derive(Debug, Clone, Copy)]
pub struct MclConfig {
    /// Number of particles. More = better accuracy, more CPU.
    /// Recommended range: 75–300 for VRC.
    pub num_particles: usize,

    /// Motion noise: translational error per unit of translation.
    pub alpha1: f64,
    /// Motion noise: translational error per unit of rotation.
    pub alpha2: f64,
    /// Motion noise: rotational error per unit of translation.
    pub alpha3: f64,
    /// Motion noise: rotational error per unit of rotation.
    pub alpha4: f64,

    /// Standard deviation (inches) of distance sensor measurements.
    /// VEX V5 distance sensor: ~0.5–1.5 in is realistic.
    pub sensor_std_dev_in: f64,

    /// Distance readings above this are treated as invalid / out-of-range.
    pub max_valid_reading_in: f64,

    /// When the sum of all weights drops below this threshold after the
    /// sensor update, inject `injection_count` random particles to prevent
    /// total filter collapse ("particle deprivation recovery").
    pub min_weight_sum: f64,

    /// Number of random particles injected when weight sum is critically low.
    pub injection_count: usize,
}

impl Default for MclConfig {
    fn default() -> Self {
        Self {
            num_particles:      150,
            alpha1:             0.002,   // trans noise per trans
            alpha2:             0.001,   // trans noise per rot
            alpha3:             0.001,   // rot noise per trans
            alpha4:             0.002,   // rot noise per rot
            sensor_std_dev_in:  1.0,
            max_valid_reading_in: 80.0,
            min_weight_sum:     1e-6,
            injection_count:    20,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Sensor observation bundle
// ─────────────────────────────────────────────────────────────────────────────

/// One snapshot of all available sensor readings.
///
/// `None` = sensor unavailable or out of range for that direction.
#[derive(Debug, Clone, Copy, Default)]
pub struct Observation {
    pub north_in: Option<f64>,
    pub east_in:  Option<f64>,
    pub south_in: Option<f64>,
    pub west_in:  Option<f64>,
    /// Fused IMU heading in **radians** (CCW positive).
    pub heading_rad: Option<f64>,
}

// ─────────────────────────────────────────────────────────────────────────────
// MCL
// ─────────────────────────────────────────────────────────────────────────────

/// Monte Carlo Localisation particle filter.
pub struct Mcl {
    pub particles: Vec<Particle>,
    pub config:    MclConfig,
    /// Best current pose estimate (weighted mean).
    pub estimate:  Pose,
    /// PRNG state (xorshift64 — no_std compatible, no alloc).
    rng:           u64,
}

impl Mcl {
    // ── Construction ──────────────────────────────────────────────────────────

    /// Create a new MCL filter, spreading particles uniformly over the field.
    pub fn new(config: MclConfig) -> Self {
        let mut mcl = Self {
            particles: Vec::with_capacity(config.num_particles),
            config,
            estimate:  Pose::default(),
            rng:       12345678,
        };
        mcl.spread_uniform();
        mcl
    }

    /// Seed the filter with particles clustered around a known starting pose.
    ///
    /// Useful when you know the starting tile at match start.
    pub fn set_pose(&mut self, pose: Pose, pos_std_in: f64, heading_std_rad: f64) {
        self.particles.clear();
        let n = self.config.num_particles;
        let w = 1.0 / n as f64;
        for _ in 0..n {
            let x   = pose.x       + self.randn() * pos_std_in;
            let y   = pose.y       + self.randn() * pos_std_in;
            let h   = pose.heading + self.randn() * heading_std_rad;
            self.particles.push(Particle::new(Pose { x, y, heading: h }, w));
        }
        self.estimate = pose;
    }

    // ── Main update ───────────────────────────────────────────────────────────

    /// Run one full MCL cycle.
    ///
    /// # Parameters
    ///
    /// - `delta_fwd_in`   – forward translation since last update (inches)
    /// - `delta_lat_in`   – lateral (strafe) translation since last update (inches)
    /// - `delta_rot_rad`  – rotation since last update (radians, CCW positive)
    /// - `obs`            – sensor observations this cycle
    pub fn update(
        &mut self,
        delta_fwd_in:  f64,
        delta_lat_in:  f64,
        delta_rot_rad: f64,
        obs:           &Observation,
    ) {
        // 1. Motion update
        self.motion_update(delta_fwd_in, delta_lat_in, delta_rot_rad, obs.heading_rad);

        // 2. Sensor update
        let weight_sum = self.sensor_update(obs);

        // 3. Normalise
        if weight_sum > self.config.min_weight_sum {
            for p in &mut self.particles {
                p.weight /= weight_sum;
            }
        } else {
            // Particle deprivation: inject random particles
            log::warn!("MCL: weight sum critically low ({:.2e}) — injecting random particles", weight_sum);
            self.inject_random_particles();
            let new_sum: f64 = self.particles.iter().map(|p| p.weight).sum();
            if new_sum > 0.0 {
                for p in &mut self.particles { p.weight /= new_sum; }
            }
        }

        // 4. Resample
        self.low_variance_resample();

        // 5. Estimate
        self.estimate = self.weighted_mean();
    }

    // ── Step 1: Motion update ─────────────────────────────────────────────────

    fn motion_update(
        &mut self,
        delta_fwd:  f64,
        delta_lat:  f64,
        delta_rot:  f64,
        imu_heading: Option<f64>,
    ) {
        let delta_trans = (delta_fwd * delta_fwd + delta_lat * delta_lat).sqrt();
        let cfg = self.config;

        for p in &mut self.particles {
            // Sample translational noise
            let std_trans = (cfg.alpha1 * delta_trans + cfg.alpha2 * delta_rot.abs()).max(1e-9);
            let std_rot   = (cfg.alpha3 * delta_trans + cfg.alpha4 * delta_rot.abs()).max(1e-9);

            let noise_fwd  = self.randn() * std_trans;
            let noise_lat  = self.randn() * std_trans;
            let noise_rot  = self.randn() * std_rot;

            let noisy_fwd  = delta_fwd + noise_fwd;
            let noisy_lat  = delta_lat + noise_lat;
            let noisy_rot  = delta_rot + noise_rot;

            // Propagate particle through motion model
            let heading_before = p.pose.heading;
            let avg_heading    = heading_before + noisy_rot / 2.0;

            p.pose.x       += noisy_fwd * avg_heading.cos() - noisy_lat * avg_heading.sin();
            p.pose.y       += noisy_fwd * avg_heading.sin() + noisy_lat * avg_heading.cos();
            p.pose.heading  = wrap_angle(heading_before + noisy_rot);

            // Soft-blend IMU heading if available (prevents heading drift)
            if let Some(imu_h) = imu_heading {
                let imu_noise = self.randn() * std_rot * 0.3; // IMU is more accurate
                let blend = 0.7; // weight toward IMU
                let diff  = angle_diff(imu_h + imu_noise, p.pose.heading);
                p.pose.heading = wrap_angle(p.pose.heading + diff * blend);
            }

            // Clamp to field bounds with a small margin
            p.pose.x = p.pose.x.clamp(0.5, FIELD_SIZE_IN - 0.5);
            p.pose.y = p.pose.y.clamp(0.5, FIELD_SIZE_IN - 0.5);
        }
    }

    // ── Step 2: Sensor update ─────────────────────────────────────────────────

    /// Apply observation weights. Returns the sum of all weights.
    fn sensor_update(&mut self, obs: &Observation) -> f64 {
        let sigma = self.config.sensor_std_dev_in;
        let max_r = self.config.max_valid_reading_in;
        let mut sum = 0.0_f64;

        for p in &mut self.particles {
            let mut w = 1.0_f64;

            // North sensor: distance to north wall (y = FIELD_SIZE_IN)
            if let Some(r) = obs.north_in.filter(|&r| r < max_r) {
                let expected = expected_north(p.pose);
                w *= gaussian_likelihood(r, expected, sigma);
            }
            // South sensor: distance to south wall (y = 0)
            if let Some(r) = obs.south_in.filter(|&r| r < max_r) {
                let expected = expected_south(p.pose);
                w *= gaussian_likelihood(r, expected, sigma);
            }
            // East sensor: distance to east wall (x = FIELD_SIZE_IN)
            if let Some(r) = obs.east_in.filter(|&r| r < max_r) {
                let expected = expected_east(p.pose);
                w *= gaussian_likelihood(r, expected, sigma);
            }
            // West sensor: distance to west wall (x = 0)
            if let Some(r) = obs.west_in.filter(|&r| r < max_r) {
                let expected = expected_west(p.pose);
                w *= gaussian_likelihood(r, expected, sigma);
            }

            p.weight *= w;
            sum += p.weight;
        }

        sum
    }

    // ── Step 4: Low-variance resampler ────────────────────────────────────────

    /// Systematic (low-variance) resampler.
    ///
    /// Draws a single random offset r ∈ [0, 1/N) and steps through the
    /// cumulative weight distribution at uniform intervals 1/N. This
    /// minimises resampling variance compared to multinomial sampling.
    fn low_variance_resample(&mut self) {
        let n = self.particles.len();
        if n == 0 { return; }

        let inv_n = 1.0 / n as f64;
        let r     = self.rand_uniform() * inv_n;

        let mut new_particles: Vec<Particle> = Vec::with_capacity(n);
        let mut cumulative = 0.0_f64;
        let mut j = 0usize;

        for i in 0..n {
            let target = r + i as f64 * inv_n;
            while cumulative < target && j < n {
                cumulative += self.particles[j].weight;
                j += 1;
            }
            let src_idx = if j > 0 { j - 1 } else { 0 };
            new_particles.push(Particle::new(self.particles[src_idx].pose, inv_n));
        }

        self.particles = new_particles;
    }

    // ── Step 5: Weighted mean ─────────────────────────────────────────────────

    fn weighted_mean(&self) -> Pose {
        if self.particles.is_empty() {
            return Pose::default();
        }

        let mut x       = 0.0_f64;
        let mut y       = 0.0_f64;
        // Circular mean for heading (avoid wrap-around issues)
        let mut sin_sum = 0.0_f64;
        let mut cos_sum = 0.0_f64;

        for p in &self.particles {
            x       += p.weight * p.pose.x;
            y       += p.weight * p.pose.y;
            sin_sum += p.weight * p.pose.heading.sin();
            cos_sum += p.weight * p.pose.heading.cos();
        }

        Pose { x, y, heading: sin_sum.atan2(cos_sum) }
    }

    // ── Particle deprivation recovery ─────────────────────────────────────────

    /// Spread all particles uniformly over the field.
    fn spread_uniform(&mut self) {
        self.particles.clear();
        let n = self.config.num_particles;
        let w = 1.0 / n as f64;
        for _ in 0..n {
            let x = self.rand_uniform() * FIELD_SIZE_IN;
            let y = self.rand_uniform() * FIELD_SIZE_IN;
            let h = (self.rand_uniform() * 2.0 - 1.0) * PI;
            self.particles.push(Particle::new(Pose { x, y, heading: h }, w));
        }
    }

    /// Replace the lowest-weight `injection_count` particles with random ones.
    fn inject_random_particles(&mut self) {
        let count = self.config.injection_count.min(self.particles.len());
        let n     = self.particles.len();
        let w     = 1.0 / n as f64;

        // Sort ascending by weight so we replace the least useful particles
        // (can't use f64::partial_cmp directly without std, use workaround)
        for i in 0..count {
            // Find min-weight particle
            let mut min_idx = i;
            let mut min_w   = self.particles[i].weight;
            for j in (i + 1)..n {
                if self.particles[j].weight < min_w {
                    min_w   = self.particles[j].weight;
                    min_idx = j;
                }
            }
            self.particles.swap(i, min_idx);

            // Replace with random particle
            let x = self.rand_uniform() * FIELD_SIZE_IN;
            let y = self.rand_uniform() * FIELD_SIZE_IN;
            let h = (self.rand_uniform() * 2.0 - 1.0) * PI;
            self.particles[i] = Particle::new(Pose { x, y, heading: h }, w);
        }
    }

    // ── Diagnostics ───────────────────────────────────────────────────────────

    /// Effective sample size (ESS): a measure of particle diversity.
    ///
    /// ESS = 1 / Σ(wᵢ²). Ranges from 1 (all weight on one particle) to
    /// N (uniform weights). Low ESS → consider increasing particle count.
    pub fn effective_sample_size(&self) -> f64 {
        let sq_sum: f64 = self.particles.iter().map(|p| p.weight * p.weight).sum();
        if sq_sum < 1e-15 { 0.0 } else { 1.0 / sq_sum }
    }

    /// Standard deviation of particle X positions (measure of uncertainty).
    pub fn x_std_dev(&self) -> f64 {
        let mean = self.estimate.x;
        let var: f64 = self.particles.iter()
            .map(|p| p.weight * (p.pose.x - mean).powi(2))
            .sum();
        var.sqrt()
    }

    /// Standard deviation of particle Y positions.
    pub fn y_std_dev(&self) -> f64 {
        let mean = self.estimate.y;
        let var: f64 = self.particles.iter()
            .map(|p| p.weight * (p.pose.y - mean).powi(2))
            .sum();
        var.sqrt()
    }

    // ── PRNG: xorshift64 (no_std, no alloc) ──────────────────────────────────

    #[inline]
    fn rand_u64(&mut self) -> u64 {
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.rng = x;
        x
    }

    /// Uniform random f64 in [0, 1).
    #[inline]
    fn rand_uniform(&mut self) -> f64 {
        (self.rand_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Standard normal sample via Box-Muller (uses two uniform samples).
    fn randn(&mut self) -> f64 {
        let u1 = (self.rand_uniform()).max(1e-15);
        let u2 = self.rand_uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Sensor prediction helpers
// ─────────────────────────────────────────────────────────────────────────────
//
// These predict what a distance sensor *should* read given a particle's pose.
// They account for the sensor's offset from the robot centre.
//
// NOTE: These assume the robot is always axis-aligned (heading ≈ 0) when
// reading distance sensors. For a fully general model you would project
// the sensor ray into field coordinates using the current heading.
// For VRC match play this simplification is reasonable because distance
// sensors are typically only useful when facing a wall (near 0/90/180/270°).
// A heading-aware version can be added later by projecting each reading along
// heading + sensor_direction, finding the nearest wall intersection.

fn expected_north(pose: Pose) -> f64 {
    (FIELD_SIZE_IN - pose.y - DIST_NORTH_OFFSET_IN).max(0.0)
}
fn expected_south(pose: Pose) -> f64 {
    (pose.y - DIST_SOUTH_OFFSET_IN).max(0.0)
}
fn expected_east(pose: Pose) -> f64 {
    (FIELD_SIZE_IN - pose.x - DIST_EAST_OFFSET_IN).max(0.0)
}
fn expected_west(pose: Pose) -> f64 {
    (pose.x - DIST_WEST_OFFSET_IN).max(0.0)
}

// ─────────────────────────────────────────────────────────────────────────────
// Math helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Gaussian probability density (unnormalised — just the exp term).
///
/// Normalisation constant cancels during weight normalisation so we skip it
/// for efficiency.
#[inline]
fn gaussian_likelihood(observed: f64, expected: f64, sigma: f64) -> f64 {
    let diff = observed - expected;
    (-(diff * diff) / (2.0 * sigma * sigma)).exp()
}

/// Wrap an angle to [-π, π].
#[inline]
pub(crate) fn wrap_angle(a: f64) -> f64 {
    let mut r = a;
    while r >  PI { r -= 2.0 * PI; }
    while r < -PI { r += 2.0 * PI; }
    r
}

/// Shortest signed difference from `from` to `to` (radians).
#[inline]
pub(crate) fn angle_diff(to: f64, from: f64) -> f64 {
    wrap_angle(to - from)
}
