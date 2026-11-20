//! Driver for Yesense IMUs (YIS series) speaking the standard binary output protocol over a serial port.
//!
//! [`Decoder`] turns a byte stream into [`Frame`]s (checksummed, resynchronizing on garbage or partial
//! frames); [`Imu`] opens a serial device and yields [`Sample`]s (a frame plus its host receive time).
//!
//! ```no_run
//! let mut imu = yesense_imu::Imu::open(&yesense_imu::Config::new("/dev/ttyS9", 115200))?;
//! for sample in &mut imu {
//!     let sample = sample?;
//!     println!("{:?} {:?}", sample.frame.accel_mps2, sample.frame.gyro_rad_s);
//! }
//! # Ok::<(), std::io::Error>(())
//! ```

#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used, clippy::panic))]

pub mod protocol;
#[cfg(unix)]
pub mod serial;

pub use protocol::{
    checksum, encode_frame, try_encode_frame, Decoder, DecoderConfig, DecoderStats, Euler, Frame, Position, Velocity,
};
#[cfg(unix)]
pub use serial::{Config, Imu, Sample};
