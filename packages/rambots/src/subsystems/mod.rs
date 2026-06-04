//! Robot specific subsystems.
//!
//! Each subsystem owns the hardware it controls and exposes a clean API.
//!
//! | Module | Contents |
//! |---|---|
//! | [`claw`] | Pneumatic claw (`open` / `close` / `toggle`) |
//! | [`intake`] | Multi-motor intake (`intake` / `outtake` / `set_power`) |
//! | [`lift`] | Cascade and DR4B lifts inch-based API with presets |

pub mod claw;
pub mod intake;
pub mod lift;