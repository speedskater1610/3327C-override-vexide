//! Four-direction distance sensor localiser.
//!
//! Uses four VEX V5 distance sensors pointing North, East, South, and West
//! to measure distances to field walls. Combined with knowledge of the
//! field dimensions, these measurements can correct accumulated odometry drift.
//!
//! # Coordinate system
//!
//! ```text
//!   (0, 144)-----------------(144, 144)
//!      |  <- 12 ft VRC field ->   |
//!      |                          |  +Y
//!      |                          |   ^
//!      |         ROBOT            |   |
//!      |           ^ N            |   ----> +X
//!      |                          |
//!   (0, 0)-------------------(144, 0)
//! ```
//!
//! The field is **144 x 144 inches** (12 ft x 12 ft).
//!
//! # Offset compensation
//!
//! Each sensor is mounted at a known offset from the robots geometric centre.
//! The raw sensor reading plus the offset gives the distance from the *centre*
//! to the wall, allowing us to compute the robot centres position in field
//! coordinates.
//!
//! Look in `../localization` for full MCL and EKF / other related things.

use vexide::smart::distance::DistanceSensor;

use crate::drivetrain::config::{
    DIST_EAST_OFFSET_IN, DIST_NORTH_OFFSET_IN, DIST_SOUTH_OFFSET_IN, DIST_WEST_OFFSET_IN,
};

/// VRC full-size field dimension in inches (12 ft).
pub const FIELD_SIZE_IN: f64 = 144.0;

/// Maximum plausible distance reading in inches.
/// Readings above this are treated as "no wall detected".
pub const MAX_VALID_READING_IN: f64 = 100.0;

/// A single corrected wall-distance measurement.
#[derive(Debug, Clone, Copy)]
pub struct WallReading {
    /// Raw sensor reading in inches.
    pub raw_in: f64,
    /// Distance from the robot **centre** to the wall (raw + sensor offset).
    pub centre_to_wall_in: f64,
    /// Inferred robot-centre position along the relevant axis, in field inches.
    /// `None` if the reading was above [`MAX_VALID_READING_IN`].
    pub field_position_in: Option<f64>,
}

/// Four-direction distance sensor array with offset-corrected localisation.
pub struct DistanceLocalizer {
    north: DistanceSensor,
    east: DistanceSensor,
    south: DistanceSensor,
    west: DistanceSensor,
}

impl DistanceLocalizer {
    pub fn new(
        north: DistanceSensor,
        east: DistanceSensor,
        south: DistanceSensor,
        west: DistanceSensor,
    ) -> Self {
        Self { north, east, south, west }
    }

    // Raw corrected readings

    /// Distance from robot centre to the **north** (forward) wall
    pub fn north(&self) -> WallReading {
        self.reading(
            &self.north,
            DIST_NORTH_OFFSET_IN,
            |centre_dist| FIELD_SIZE_IN - centre_dist, // Y position from south wall
        )
    }

    /// Distance from robot centre to the **south** (rear) wall
    pub fn south(&self) -> WallReading {
        self.reading(
            &self.south,
            DIST_SOUTH_OFFSET_IN,
            |centre_dist| centre_dist, // Y position from south wall
        )
    }

    /// Distance from robot centre to the **east** (right) wall
    pub fn east(&self) -> WallReading {
        self.reading(
            &self.east,
            DIST_EAST_OFFSET_IN,
            |centre_dist| FIELD_SIZE_IN - centre_dist, // X position from west wall
        )
    }

    /// Distance from robot centre to the **west** (left) wall
    pub fn west(&self) -> WallReading {
        self.reading( 
            &self.west,
            DIST_WEST_OFFSET_IN,
            |centre_dist| centre_dist, // X position from west wall
        )
    }

    // Fused position estimate

    /// Best estimate X position of robot centre (inches from west wall)
    ///
    /// Averages East and West readings if both are valid; falls back to one
    /// if the other is out of range. Returns `None` if neither is valid
    pub fn x_estimate(&self) -> Option<f64> {
        let e = self.east().field_position_in;
        let w = self.west().field_position_in;
        fuse_readings(e, w)
    }

    /// Best-estimate Y position of robot centre (inches from south wall)
    ///
    /// Averages North and South readings if both are valid.
    pub fn y_estimate(&self) -> Option<f64> {
        let n = self.north().field_position_in;
        let s = self.south().field_position_in;
        fuse_readings(n, s)
    }

    /// Both X and Y estimates as `Option<(x, y)>`.
    pub fn position_estimate(&self) -> Option<(f64, f64)> {
        Some((self.x_estimate()?, self.y_estimate()?))
    }

    fn reading(
        &self,
        sensor: &DistanceSensor,
        offset_in: f64,
        field_pos_fn: impl Fn(f64) -> f64,
    ) -> WallReading {
        // DistanceSensor::distance() returns mm then convert to inches
        let raw_mm = sensor.distance().unwrap_or(u32::MAX);
        let raw_in = raw_mm as f64 / 25.4;
        let centre_to_wall = raw_in + offset_in;

        let field_position = if raw_in < MAX_VALID_READING_IN {
            Some(field_pos_fn(centre_to_wall))
        } else {
            None
        };

        WallReading {
            raw_in,
            centre_to_wall_in: centre_to_wall,
            field_position_in: field_position,
        }
    }
}

/// Average two optional readings; prefer whichever is valid if only one is.
fn fuse_readings(a: Option<f64>, b: Option<f64>) -> Option<f64> {
    match (a, b) {
        (Some(va), Some(vb)) => Some((va + vb) / 2.0),
        (Some(v), None) | (None, Some(v)) => Some(v),
        (None, None) => None,
    }
}
