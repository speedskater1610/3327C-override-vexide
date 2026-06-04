//! Intake subsystem.
//!
//! Drives one or more intake motors at a fixed voltage or spins them
//! in reverse to outtake.

use vexide::devices::smart::motor::Motor;

const INTAKE_VOLTAGE: f64 = 12000.0;

pub struct Intake {
    motors: alloc::vec::Vec<Motor>,
}

impl Intake {
    pub fn new(motors: alloc::vec::Vec<Motor>) -> Self {
        Self { motors }
    }

    /// Run the intake inward (collect rings/balls).
    pub fn intake(&mut self) {
        for m in &mut self.motors {
            let _ = m.set_voltage(INTAKE_VOLTAGE);
        }
    }

    /// Run the intake outward (eject).
    pub fn outtake(&mut self) {
        for m in &mut self.motors {
            let _ = m.set_voltage(-INTAKE_VOLTAGE);
        }
    }

    /// Stop all intake motors.
    pub fn stop(&mut self) {
        for m in &mut self.motors {
            let _ = m.set_voltage(0.0);
        }
    }

    /// Run at a custom voltage fraction in `[-1.0, 1.0]`.
    pub fn set_power(&mut self, fraction: f64) {
        let v = fraction.clamp(-1.0, 1.0) * INTAKE_VOLTAGE;
        for m in &mut self.motors {
            let _ = m.set_voltage(v);
        }
    }
}
