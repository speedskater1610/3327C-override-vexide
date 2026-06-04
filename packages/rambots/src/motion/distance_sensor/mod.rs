//! Distance sensor gated motion utils.
//!
//! Provides async futures that resolve when a VEX V5 distance sensor
//! crosses a threshold, allowing motion code to `await` proximity events.

pub mod future;

pub use future::UntilDistance;
