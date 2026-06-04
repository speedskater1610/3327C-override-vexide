//! Motion control algorithms.
//!
//! - [`controllers`] - Feedback controllers (PID, bang-bang, feedforward)
//! - [`basic`]       - Simple open-loop drive helpers
//! - [`distance_sensor`] - Distance sensor gated motion futures

pub mod basic;
pub mod controllers;
pub mod distance_sensor;
