//! auton route: a named sequence of [`Action`]s.

use alloc::{string::String, vec::Vec};

use super::Action;

/// a named auton route - a list of actions run in order.
///
/// # Example
/// ```rust
/// let route = Route::new("skills")
///     .then(Action::drive_forward(600.0, 0.8, 3000))
///     .then(Action::turn_to_heading(90.0, 2000))
///     .then(Action::intake_for(500));
/// ```
pub struct Route {
    pub name: String,
    pub actions: Vec<Action>,
}

impl Route {
    /// Create an empty route with the given name
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            actions: Vec::new(),
        }
    }

    /// Append an action and return `self` for chaining
    pub fn then(mut self, action: Action) -> Self {
        self.actions.push(action);
        self
    }

    /// Append multiple actions.
    pub fn then_all(mut self, actions: impl IntoIterator<Item = Action>) -> Self {
        self.actions.extend(actions);
        self
    }

    /// Returns the number of actions in this route
    pub fn len(&self) -> usize {
        self.actions.len()
    }

    pub fn is_empty(&self) -> bool {
        self.actions.is_empty()
    }
}
