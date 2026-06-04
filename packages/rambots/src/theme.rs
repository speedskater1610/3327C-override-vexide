//! Colour palette and display branding constants for Rambots.

use vexide::color::Color;

/// Team primary colour (blue)
pub const PRIMARY: Color = Color::new(0, 71, 171);

/// Team secondary colour (yellow)
pub const SECONDARY: Color = Color::new(55, 100, 55);

/// Background / clear color (black)
pub const BACKGROUND: Color = Color::new(0, 0, 0);

/// Success indicator (green)
pub const SUCCESS: Color = Color::new(0, 220, 80);

/// Warning indicator (yellow)
pub const WARNING: Color = Color::new(255, 220, 0);

/// Error indicator (red)
pub const ERROR: Color = Color::new(220, 30, 30);