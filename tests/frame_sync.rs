//! The M20 yesense_node's frame read (next_synced_frame): one frame per 'Y' 'S' sync, its boundaries the vendor's.

use std::io;

use yesense_imu::protocol::{ACCEL_ID, EULER_ID, GYRO_ID};
use yesense_imu::{encode_frame, next_synced_frame, Decoder, DecoderConfig};

fn s32s(values: &[i32]) -> Vec<u8> {
    values.iter().flat_map(|value| value.to_le_bytes()).collect()
}

/// The M20 body IMU's 49-byte frame (accel, gyro, euler).
fn m20_frame(tid: u16) -> Vec<u8> {
    encode_frame(
        tid,
        &[
            (ACCEL_ID, &s32s(&[120_000, 9_806_650, -50_000])),
            (GYRO_ID, &s32s(&[1_000_000, -2_000_000, i32::from(tid)])),
            (EULER_ID, &s32s(&[1_500_000, 90_250_000, -45_000_000])),
        ],
    )
}

/// A byte stream read like the vendor read: each call gets exactly what it asks for while bytes remain.
fn reader(bytes: Vec<u8>) -> impl FnMut(&mut [u8]) -> io::Result<usize> {
    let mut at = 0;
    move |buffer: &mut [u8]| {
        let count = buffer.len().min(bytes.len() - at);
        buffer[..count].copy_from_slice(&bytes[at..at + count]);
        at += count;
        Ok(count)
    }
}

#[test]
fn every_frame_after_junk_and_false_syncs_is_read_alone() {
    let length = m20_frame(0).len();
    assert_eq!(length, 0x31, "yesense_node reads 2 + 0x2f bytes");
    let mut stream = vec![0x00, 0x59, 0x00, 0x53, 0x59, 0x59];
    for tid in 1..=5u16 {
        stream.extend(m20_frame(tid));
        // junk between frames, including a lone 'Y'
        stream.extend([0x11, 0x59, 0x22]);
    }
    let mut read = reader(stream);
    let mut decoder = Decoder::new(DecoderConfig::default());
    let mut tids = Vec::new();
    loop {
        match next_synced_frame(&mut read, length, &mut decoder).unwrap() {
            Some(frame) => tids.push(frame.tid),
            None => break,
        }
    }
    // "Y Y" before the first frame: the second 'Y' is not 'S', so the sync starts over at the next 'Y', the frame's
    assert_eq!(tids, [1, 2, 3, 4, 5]);
}

#[test]
fn a_corrupt_frame_is_dropped_and_the_next_sync_reads_on() {
    let length = m20_frame(0).len();
    let mut bad = m20_frame(7);
    bad[20] ^= 0xff;
    let mut stream = bad;
    stream.extend(m20_frame(8));
    let mut read = reader(stream);
    let mut decoder = Decoder::new(DecoderConfig::default());
    assert!(next_synced_frame(&mut read, length, &mut decoder).unwrap().is_none());
    assert_eq!(next_synced_frame(&mut read, length, &mut decoder).unwrap().map(|f| f.tid), Some(8));
}
