//! # Rambots Controls Library
//!
//! Hardware abstraction, motion control, auton utilities, and subsystems
//! for Team 3327C (Rambots)'s VEX V5 Override robot. Inspired by aubie2-push-back.
//!
//! ## Modules
//!
//! - [`hardware`] - Low-level hardware wrappers (encoders, IMU calibration, pneumatics)
//! - [`motion`]- Motion algos: cartesian drive, PID, pure pursuit, etc.
//! - [`subsystems`] - Robot-specific subsystems (intake, claw, DR4B, cascade)
//! - [`auton`] - Autonomous route builder and action primitives
//! - [`logger`] - serial logger
//! - [`theme`]  - Display colors / branding

#![no_std]
extern crate alloc;

pub mod auton;
pub mod drivetrain;
pub mod hardware;
pub mod localization;
pub mod logger;
pub mod motion;
pub mod subsystems;
pub mod theme;
