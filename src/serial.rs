//! A Yesense IMU on a serial device (raw 8N1, no flow control), read with blocking reads.

use std::fs::File;
use std::io::{self, Read};
use std::os::fd::AsRawFd;
use std::os::unix::fs::OpenOptionsExt;
use std::time::SystemTime;

use crate::protocol::{Decoder, DecoderConfig, DecoderStats, Frame};

/// How to open the device.
#[derive(Clone, Debug)]
pub struct Config {
    pub path: String,
    pub baud: u32,
    /// Bytes asked for per read(2); a read returns as soon as any bytes arrive.
    pub read_chunk_bytes: usize,
    pub decoder: DecoderConfig,
}

impl Config {
    pub fn new(path: impl Into<String>, baud: u32) -> Self {
        Self {
            path: path.into(),
            baud,
            read_chunk_bytes: 256,
            decoder: DecoderConfig::default(),
        }
    }
}

/// One frame and when the read holding its last byte returned.
#[derive(Clone, Debug, PartialEq)]
pub struct Sample {
    pub frame: Frame,
    pub host_time: SystemTime,
}

/// An open Yesense IMU; iterate it (or call [`next_sample`](Imu::next_sample)) for samples.
pub struct Imu {
    file: File,
    decoder: Decoder,
    chunk: Vec<u8>,
    last_read_time: SystemTime,
}

impl Imu {
    pub fn open(config: &Config) -> io::Result<Self> {
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_NOCTTY)
            .open(&config.path)?;
        configure_raw(&file, config.baud)?;
        Ok(Self {
            file,
            decoder: Decoder::new(config.decoder),
            chunk: vec![0; config.read_chunk_bytes.max(1)],
            last_read_time: SystemTime::now(),
        })
    }

    /// Blocks until a whole valid frame has arrived.
    pub fn next_sample(&mut self) -> io::Result<Sample> {
        loop {
            if let Some(frame) = self.decoder.next_frame() {
                return Ok(Sample {
                    frame,
                    host_time: self.last_read_time,
                });
            }
            let count = self.read_raw()?;
            self.decoder.push(&self.chunk[..count]);
        }
    }

    /// One read(2) into the internal chunk.
    fn read_raw(&mut self) -> io::Result<usize> {
        let count = read_retrying(&mut self.file, &mut self.chunk)?;
        self.last_read_time = SystemTime::now();
        Ok(count)
    }

    /// Raw bytes straight off the device, bypassing the decoder (for recording fixtures); like
    /// [`next_sample`](Imu::next_sample) it retries on EINTR and turns end-of-file (an unplugged device) into an error.
    pub fn read_into(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
        read_retrying(&mut self.file, buffer)
    }

    pub fn stats(&self) -> DecoderStats {
        self.decoder.stats()
    }
}

impl Iterator for Imu {
    type Item = io::Result<Sample>;

    fn next(&mut self) -> Option<Self::Item> {
        Some(self.next_sample())
    }
}

/// One read(2), retried on EINTR; end-of-file (the device went away) is an `UnexpectedEof` error.
fn read_retrying(source: &mut impl Read, buffer: &mut [u8]) -> io::Result<usize> {
    if buffer.is_empty() {
        return Ok(0);
    }
    loop {
        match source.read(buffer) {
            Ok(0) => return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "serial device closed")),
            Ok(count) => return Ok(count),
            Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
            Err(error) => return Err(error),
        }
    }
}

fn check(result: libc::c_int) -> io::Result<()> {
    if result < 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(())
    }
}

/// Raw 8N1, receiver on, modem lines ignored, reads block until at least one byte.
fn configure_raw(file: &File, baud: u32) -> io::Result<()> {
    let fd = file.as_raw_fd();
    let speed = speed_of(baud)?;
    unsafe {
        let mut tio: libc::termios = std::mem::zeroed();
        check(libc::tcgetattr(fd, &mut tio))?;
        libc::cfmakeraw(&mut tio);
        tio.c_cflag |= libc::CLOCAL | libc::CREAD;
        tio.c_cflag &= !(libc::CSTOPB | libc::PARENB | libc::CRTSCTS);
        tio.c_iflag &= !(libc::IXON | libc::IXOFF | libc::IXANY);
        // block until a byte arrives, no inter-byte timer: reads wake on data, never on a timeout
        tio.c_cc[libc::VMIN] = 1;
        tio.c_cc[libc::VTIME] = 0;
        check(libc::cfsetispeed(&mut tio, speed))?;
        check(libc::cfsetospeed(&mut tio, speed))?;
        check(libc::tcsetattr(fd, libc::TCSANOW, &tio))?;
        check(libc::tcflush(fd, libc::TCIFLUSH))?;
    }
    Ok(())
}

#[cfg(target_os = "linux")]
fn speed_of(baud: u32) -> io::Result<libc::speed_t> {
    Ok(match baud {
        9600 => libc::B9600,
        19200 => libc::B19200,
        38400 => libc::B38400,
        57600 => libc::B57600,
        115200 => libc::B115200,
        230400 => libc::B230400,
        460800 => libc::B460800,
        500000 => libc::B500000,
        576000 => libc::B576000,
        921600 => libc::B921600,
        1000000 => libc::B1000000,
        1500000 => libc::B1500000,
        2000000 => libc::B2000000,
        _ => {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("unsupported baud rate {baud}"),
            ))
        }
    })
}

// BSD-style termios takes the rate itself
#[cfg(not(target_os = "linux"))]
fn speed_of(baud: u32) -> io::Result<libc::speed_t> {
    Ok(baud as libc::speed_t)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A reader that is interrupted once, then returns its bytes, then end-of-file.
    struct Interrupted {
        interrupted: bool,
        bytes: Vec<u8>,
    }

    impl Read for Interrupted {
        fn read(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
            if !self.interrupted {
                self.interrupted = true;
                return Err(io::Error::from(io::ErrorKind::Interrupted));
            }
            let count = self.bytes.len().min(buffer.len());
            buffer[..count].copy_from_slice(&self.bytes[..count]);
            self.bytes.drain(..count);
            Ok(count)
        }
    }

    #[test]
    fn reads_retry_on_eintr_and_end_of_file_is_an_error() {
        let mut source = Interrupted {
            interrupted: false,
            bytes: vec![1, 2, 3],
        };
        let mut buffer = [0u8; 8];
        assert_eq!(read_retrying(&mut source, &mut buffer).unwrap(), 3);
        assert_eq!(&buffer[..3], &[1, 2, 3]);
        let error = read_retrying(&mut source, &mut buffer).unwrap_err();
        assert_eq!(error.kind(), io::ErrorKind::UnexpectedEof);
    }
}
