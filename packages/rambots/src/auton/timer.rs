//! deadline timer for autonomous actions.

use vexide::time::{Duration, Instant};

/// monotonic deadline used to time box autonomous actions.
pub struct ActionTimer {
    start: Instant,
    timeout: Duration,
}

impl ActionTimer {
    pub fn new(timeout: Duration) -> Self {
        Self {
            start: Instant::now(),
            timeout,
        }
    }

    pub fn from_millis(ms: u64) -> Self {
        Self::new(Duration::from_millis(ms))
    }

    /// Returns `true` if the deadline has been exceeded.
    pub fn expired(&self) -> bool {
        self.start.elapsed() >= self.timeout
    }

    /// Returns time elapsed since the timer was created.
    pub fn elapsed(&self) -> Duration {
        self.start.elapsed()
    }

    /// Remaining time before expiry.  Returns `Duration::ZERO` if already expired.
    pub fn remaining(&self) -> Duration {
        self.timeout.saturating_sub(self.start.elapsed())
    }
}
