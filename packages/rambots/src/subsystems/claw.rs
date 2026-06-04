//! Claw subsystem (pneumatic).
//!
//! Wraps a [`Solenoid`] so the claw can be opened/closed with named methods.

use crate::hardware::pneumatics::Solenoid;

pub struct Claw {
    solenoid: Solenoid,
}

impl Claw {
    pub fn new(solenoid: Solenoid) -> Self {
        Self { solenoid }
    }

    /// Open the claw (release game object).
    pub fn open(&mut self) {
        self.solenoid.retract();
    }

    /// Close the claw (grab game object).
    pub fn close(&mut self) {
        self.solenoid.extend();
    }

    /// Toggle open/close state.
    pub fn toggle(&mut self) {
        self.solenoid.toggle();
    }

    /// Returns `true` if the claw is currently closed.
    pub fn is_closed(&self) -> bool {
        self.solenoid.is_extended()
    }
}
