//! Named height presets for lifts.
//!
//! Presets let you define meaningful lift positions once and refer to them
//! by name throughout your robot code:
//!
//! ```rust
//! let presets = LiftPresets::new()
//!     .add("lowered",    0.0)
//!     .add("intake",     2.5)
//!     .add("score_low",  12.0)
//!     .add("score_mid",  20.0)
//!     .add("score_high", 28.5)
//!     .add("max",        32.0);
//!
//! lift.go_to_preset(&presets, "score_high");
//! ```

use alloc::{string::String, vec::Vec};

/// A collection of named lift heights (in inches).
#[derive(Debug, Default, Clone)]
pub struct LiftPresets {
    entries: Vec<(String, f64)>,
}

impl LiftPresets {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a named preset and return `self` for chaining.
    pub fn add(mut self, name: impl Into<String>, height_in: f64) -> Self {
        self.entries.push((name.into(), height_in));
        self
    }

    /// Look up a preset by name. Returns `None` if not found.
    pub fn get(&self, name: &str) -> Option<f64> {
        self.entries
            .iter()
            .find(|(n, _)| n.as_str() == name)
            .map(|(_, h)| *h)
    }

    /// Returns all preset names in insertion order.
    pub fn names(&self) -> impl Iterator<Item = &str> {
        self.entries.iter().map(|(n, _)| n.as_str())
    }

    /// Returns (name, height_in) pairs in insertion order.
    pub fn iter(&self) -> impl Iterator<Item = (&str, f64)> {
        self.entries.iter().map(|(n, h)| (n.as_str(), *h))
    }

    /// Number of defined presets.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Find the preset whose height is closest to `height_in`.
    pub fn nearest(&self, height_in: f64) -> Option<&str> {
        self.entries
            .iter()
            .min_by(|(_, a), (_, b)| {
                (a - height_in)
                    .abs()
                    .partial_cmp(&(b - height_in).abs())
                    .unwrap_or(core::cmp::Ordering::Equal)
            })
            .map(|(n, _)| n.as_str())
    }
}
