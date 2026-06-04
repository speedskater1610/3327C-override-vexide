//! Dual-IMU heading fusion for the stacked mirrored mount.
//!
//! # Why two IMUs?
//!
//! A single IMU is affected by vibration from motors and impacts.  Mounting
//! two IMUs back-to-back (rotated 180deg) means their **correlated noise** (real
//! rotation) adds together while their **anti-correlated noise** (vibration)
//! cancels out when you average the two readings.
//!
//! # Mount convention
//!
//! * **IMU A** - mounted normally; its heading is taken as-is.
//! * **IMU B** - mounted upside-down / rotated 180deg. Its raw heading is
//!     negated before fusion so both are in the same direction frame.
//!
//! # Output
//!
//! [`ImuFusion::heading_degrees`] returns a heading in **[0, 360)** degrees
//! where 0 = the heading at last calibration / zero.

use vexide::smart::imu::InertialSensor;

use crate::drivetrain::config::IMU_B_INVERTED;

/// Fuses two mirrored IMUs into a single low-noise heading.
pub struct ImuFusion {
    imu_a: InertialSensor,
    imu_b: InertialSensor,
}

impl ImuFusion {
    /// Construct from two [`InertialSensor`] objects.
    ///
    /// `imu_a` is the "normal" unit; `imu_b` is the inverted/mirrored one.
    pub fn new(imu_a: InertialSensor, imu_b: InertialSensor) -> Self {
        Self { imu_a, imu_b }
    }

    /// Calibrate both IMUs sequentially.
    ///
    /// This is async and should be called once at startup before any
    /// heading readings are used.
    pub async fn calibrate(&mut self) {
        // Calibrate both errors are logged but don't abort
        if self.imu_a.calibrate().await.is_err() {
            log::error!("IMU A calibration failed!");
        }
        if self.imu_b.calibrate().await.is_err() {
            log::error!("IMU B calibration failed!");
        }
        log::info!("IMU fusion calibrated.");
    }

    /// Raw fused heading in degrees, **not** normalised.
    ///
    /// Returns `None` if both sensors fail simultaneously.
    pub fn heading_raw(&self) -> Option<f64> {
        let a = self.imu_a.heading().ok();
        let b = self.imu_b.heading().ok().map(|h| {
            if IMU_B_INVERTED { -h } else { h }
        });

        match (a, b) {
            (Some(ha), Some(hb)) => Some((ha + hb) / 2.0),
            (Some(ha), None) => {
                log::warn!("IMU B unavailable - using IMU A only");
                Some(ha)
            }
            (None, Some(hb)) => {
                log::warn!("IMU A unavailable - using IMU B only");
                Some(hb)
            }
            (None, None) => {
                log::error!("Both IMUs unavailable!");
                None
            }
        }
    }

    /// Fused heading in **[0, 360)** degrees.
    ///
    /// 0deg = initial calibrated orientation.
    /// Increases clockwise
    pub fn heading_degrees(&self) -> Option<f64> {
        let raw = self.heading_raw()?;
        Some(((raw % 360.0) + 360.0) % 360.0)
    }

    /// Heading in **[-180, 180)** degrees.  Useful for computing the
    /// shortest turn direction.
    pub fn heading_signed(&self) -> Option<f64> {
        let h = self.heading_degrees()?;
        Some(if h > 180.0 { h - 360.0 } else { h })
    }

    /// Rate of rotation in degrees per second (yaw rate).
    pub fn rotation_rate_deg_s(&self) -> Option<f64> {
        let a = self.imu_a.gyro_rate().ok().map(|r| r.z);
        let b = self.imu_b.gyro_rate().ok().map(|r| {
            if IMU_B_INVERTED { -r.z } else { r.z }
        });
        match (a, b) {
            (Some(ra), Some(rb)) => Some((ra + rb) / 2.0),
            (Some(r), None) | (None, Some(r)) => Some(r),
            (None, None) => None,
        }
    }

    /// mut access to IMU A (for re calibration etc.)
    pub fn imu_a_mut(&mut self) -> &mut InertialSensor { &mut self.imu_a }
    /// mut access to IMU B
    pub fn imu_b_mut(&mut self) -> &mut InertialSensor { &mut self.imu_b }
}
