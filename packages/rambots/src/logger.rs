//! Kernel level serial logger
//!
//! # Usage
//! ```rust
//! static LOGGER: RobotLogger = RobotLogger;
//! LOGGER.init(log::LevelFilter::Debug).unwrap();
//! log::info!("Hello from Rambots!!");
//! ```

use log::{max_level, set_logger, set_max_level, LevelFilter, Log, Metadata, SetLoggerError};

/// ANSI escape sequences indexed by `log::Level as usize` (1-5).
/// Index 0 is unused (Level is 1-based).
const LEVEL_ESCAPES: [Option<&str>; 6] = [
    None,
    Some("\x1B[31m"), // ERROR - red
    Some("\x1B[33m"), // WARN  - yellow
    Some("\x1B[34m"), // INFO  - blue
    Some("\x1B[36m"), // DEBUG - cyan
    Some("\x1B[37m"), // TRACE - white
];

/// Singleton logger. Declare `static LOGGER: RobotLogger = RobotLogger;` and
/// call [`RobotLogger::init`] once at startup.
pub struct RobotLogger;

impl RobotLogger {
    /// Register this logger with the `log` crate and set the maximum level.
    pub fn init(&'static self, level: LevelFilter) -> Result<(), SetLoggerError> {
        set_logger(self)?;
        set_max_level(level);
        Ok(())
    }
}

impl Log for RobotLogger {
    fn enabled(&self, metadata: &Metadata<'_>) -> bool {
        metadata.level() <= max_level()
    }

    fn log(&self, record: &log::Record<'_>) {
        if !self.enabled(record.metadata()) {
            return;
        }

        let uptime = vexide::time::user_uptime();
        let mins = uptime.as_secs() / 60;
        let secs = uptime.as_secs() % 60;
        let ms = uptime.subsec_millis();

        let colour = LEVEL_ESCAPES[record.level() as usize].unwrap_or_default();

        // Format: MM:SS:mmm [LEVEL] message
        println!(
            "{:02}:{:02}:{:03} {}[{}]\x1B[0m {}",
            mins,
            secs,
            ms,
            colour,
            record.level(),
            record.args(),
        );
    }

    fn flush(&self) {}
}
