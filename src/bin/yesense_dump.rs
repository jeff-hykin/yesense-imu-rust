//! `yesense-dump <device> <baud> [--count N] [--raw FILE]`: print decoded samples, or record the raw bytes to FILE.

#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used, clippy::panic))]

use std::io::Write;
use std::time::UNIX_EPOCH;

use yesense_imu::{Config, Decoder, Imu};

const USAGE: &str = "usage: yesense-dump <device> <baud> [--count N] [--raw FILE]";

fn usage_error(message: &str) -> ! {
    eprintln!("{message}\n{USAGE}");
    std::process::exit(2);
}

fn main() -> std::io::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let (Some(device), Some(baud)) = (args.get(1), args.get(2)) else {
        usage_error("missing <device> or <baud>");
    };
    let Ok(baud) = baud.parse::<u32>() else {
        usage_error("baud must be a number");
    };
    let flag = |name: &str| {
        args.iter()
            .position(|arg| arg == name)
            .and_then(|at| args.get(at + 1))
            .cloned()
    };
    let count: u64 = match flag("--count").map(|value| value.parse()) {
        None => u64::MAX,
        Some(Ok(count)) => count,
        Some(Err(_)) => usage_error("--count must be a number"),
    };
    let mut imu = Imu::open(&Config::new(device, baud))?;

    if let Some(path) = flag("--raw") {
        // record raw bytes, decoding alongside only to count frames
        let mut out = std::fs::File::create(path)?;
        let mut decoder = Decoder::default();
        let mut buffer = vec![0u8; 4096];
        while decoder.stats().frames < count {
            let read = imu.read_into(&mut buffer)?;
            out.write_all(&buffer[..read])?;
            decoder.push(&buffer[..read]);
            while decoder.next_frame().is_some() {}
        }
        eprintln!("{:?}", decoder.stats());
        return Ok(());
    }

    for _ in 0..count {
        let sample = imu.next_sample()?;
        let host_s = sample
            .host_time
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs_f64();
        println!("{host_s:.6} {:?}", sample.frame);
    }
    eprintln!("{:?}", imu.stats());
    Ok(())
}
