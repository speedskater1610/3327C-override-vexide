//! Async future that resolves when a distance sensor reads below a threshold.

use core::{
    future::Future,
    pin::Pin,
    task::{Context, Poll},
};

use vexide::smart::distance::DistanceSensor;

/// Comparison direction for the threshold.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DistanceOp {
    /// Resolve when `reading < threshold_mm`.
    LessThan,
    /// Resolve when `reading > threshold_mm`.
    GreaterThan,
}

/// An async future that polls a distance sensor every executor tick and
/// resolves once the condition `op(reading, threshold_mm)` is satisfied.
///
/// # Example
/// ```rust
/// // Wait until something is within 100 mm.
/// UntilDistance::new(&mut sensor, 100, DistanceOp::LessThan).await;
/// ```
pub struct UntilDistance<'a> {
    sensor: &'a mut DistanceSensor,
    threshold_mm: u32,
    op: DistanceOp,
}

impl<'a> UntilDistance<'a> {
    pub fn new(sensor: &'a mut DistanceSensor, threshold_mm: u32, op: DistanceOp) -> Self {
        Self { sensor, threshold_mm, op }
    }
}

impl<'a> Future for UntilDistance<'a> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();

        let dist = match this.sensor.distance() {
            Ok(d) => d,
            Err(_) => {
                cx.waker().wake_by_ref();
                return Poll::Pending;
            }
        };

        let satisfied = match this.op {
            DistanceOp::LessThan => dist < this.threshold_mm,
            DistanceOp::GreaterThan => dist > this.threshold_mm,
        };

        if satisfied {
            Poll::Ready(())
        } else {
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    }
}
