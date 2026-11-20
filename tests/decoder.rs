//! Frames built from the protocol spec (no live capture yet): the M20's 200 Hz accel + gyro + euler frame,
//! checksum, every split point, garbage, corrupt frames and false headers.

use yesense_imu::protocol::{ACCEL_ID, EULER_ID, GYRO_ID, QUATERNION_ID, SAMPLE_TIMESTAMP_ID, SENSOR_TEMP_ID};
use yesense_imu::{checksum, encode_frame, Decoder, DecoderConfig, Euler};

fn s32s(values: &[i32]) -> Vec<u8> {
    values.iter().flat_map(|value| value.to_le_bytes()).collect()
}

/// What the M20's body IMU sends: accel (1e-6 m/s2), gyro (1e-6 deg/s), euler pitch/roll/yaw (1e-6 deg).
fn m20_frame(tid: u16) -> Vec<u8> {
    encode_frame(
        tid,
        &[
            (ACCEL_ID, &s32s(&[120_000, 9_806_650, -50_000])),
            (GYRO_ID, &s32s(&[1_000_000, -2_000_000, 180_000_000])),
            (EULER_ID, &s32s(&[1_500_000, 90_250_000, -45_000_000])),
        ],
    )
}

fn decode_all(decoder: &mut Decoder) -> Vec<yesense_imu::Frame> {
    std::iter::from_fn(|| decoder.next_frame()).collect()
}

fn close(a: f64, b: f64, tolerance: f64) -> bool {
    (a - b).abs() <= tolerance
}

#[test]
fn checksum_is_the_running_byte_sums() {
    // tid 1, len 0: ck1 = 1 + 0 + 0, ck2 = 1 + 1 + 1
    assert_eq!(checksum(&[0x01, 0x00, 0x00]), [0x01, 0x03]);
    // wraps modulo 256
    assert_eq!(checksum(&[0xff, 0x02]), [0x01, 0x00]);
    let frame = encode_frame(1, &[]);
    assert_eq!(frame, vec![0x59, 0x53, 0x01, 0x00, 0x00, 0x01, 0x03]);
}

#[test]
fn m20_frame_is_49_bytes_and_decodes_in_si_units() {
    let bytes = m20_frame(4321);
    // 49 bytes at 200 Hz is the 9800 B/s seen on the M20's /dev/ttyS9
    assert_eq!(bytes.len(), 49);
    let mut decoder = Decoder::default();
    decoder.push(&bytes);
    let frame = decoder.next_frame().expect("a frame");
    assert_eq!(frame.tid, 4321);
    let accel = frame.accel_mps2.unwrap();
    assert!(close(accel[0], 0.12, 1e-12) && close(accel[1], 9.80665, 1e-12) && close(accel[2], -0.05, 1e-12));
    let gyro = frame.gyro_rad_s.unwrap();
    assert!(close(gyro[0], 1f64.to_radians(), 1e-12));
    assert!(close(gyro[1], -2f64.to_radians(), 1e-12));
    assert!(close(gyro[2], std::f64::consts::PI, 1e-12));
    let euler = frame.euler.unwrap();
    assert!(close(euler.pitch, 1.5f64.to_radians(), 1e-12));
    assert!(close(euler.roll, 90.25f64.to_radians(), 1e-12));
    assert!(close(euler.yaw, -45f64.to_radians(), 1e-12));
    assert_eq!(frame.quaternion, None);
    assert_eq!(frame.temperature_c, None);
    assert!(decoder.next_frame().is_none());
    assert_eq!(decoder.stats().frames, 1);
    assert_eq!(decoder.buffered(), 0);
}

#[test]
fn other_blocks_decode() {
    let quaternion = s32s(&[500_000, -250_000, 0, 125_000]);
    let bytes = encode_frame(
        7,
        &[
            (SENSOR_TEMP_ID, &(-1234i16).to_le_bytes()),
            (QUATERNION_ID, &quaternion),
            (SAMPLE_TIMESTAMP_ID, &123_456_789u32.to_le_bytes()),
            (0x99, &[1, 2, 3]),
        ],
    );
    let mut decoder = Decoder::default();
    decoder.push(&bytes);
    let frame = decoder.next_frame().unwrap();
    assert!(close(frame.temperature_c.unwrap(), -12.34, 1e-12));
    let decoded = frame.quaternion.unwrap();
    for (ours, expected) in decoded.iter().zip([0.5, -0.25, 0.0, 0.125]) {
        assert!(close(*ours, expected, 1e-12));
    }
    assert_eq!(frame.orientation(), frame.quaternion);
    assert_eq!(frame.sample_timestamp, Some(123_456_789));
    assert_eq!(decoder.stats().unknown_blocks, 1);
}

#[test]
fn every_split_point_gives_the_same_frames() {
    let stream: Vec<u8> = (0..3).flat_map(m20_frame).collect();
    for split in 0..=stream.len() {
        let mut decoder = Decoder::default();
        decoder.push(&stream[..split]);
        let mut frames = decode_all(&mut decoder);
        decoder.push(&stream[split..]);
        frames.extend(decode_all(&mut decoder));
        assert_eq!(
            frames.iter().map(|frame| frame.tid).collect::<Vec<_>>(),
            [0, 1, 2],
            "split at {split}"
        );
        assert_eq!(decoder.stats().checksum_errors, 0);
    }
}

#[test]
fn byte_at_a_time() {
    let stream: Vec<u8> = (10..20).flat_map(m20_frame).collect();
    let mut decoder = Decoder::default();
    let mut tids = Vec::new();
    for byte in stream {
        decoder.push(&[byte]);
        tids.extend(decode_all(&mut decoder).iter().map(|frame| frame.tid));
    }
    assert_eq!(tids, (10..20).collect::<Vec<_>>());
}

#[test]
fn garbage_and_a_corrupt_frame_are_skipped() {
    let mut stream = vec![0x00, 0x59, 0x12, 0x53, 0xff, 0x59];
    stream.extend(m20_frame(1));
    let mut corrupt = m20_frame(2);
    corrupt[20] ^= 0x40;
    stream.extend(&corrupt);
    stream.extend(m20_frame(3));
    let mut decoder = Decoder::default();
    decoder.push(&stream);
    let tids: Vec<u16> = decode_all(&mut decoder).iter().map(|frame| frame.tid).collect();
    assert_eq!(tids, [1, 3]);
    let stats = decoder.stats();
    assert_eq!(stats.checksum_errors, 1);
    assert_eq!(stats.skipped_bytes, 6 + 49);
}

#[test]
fn a_false_header_claiming_a_long_payload_does_not_swallow_real_frames() {
    // 0x59 0x53 then a length of 250: the decoder waits for 257 bytes, fails the checksum, and rescans from there
    let mut stream = vec![0x59, 0x53, 0x00, 0x00, 250];
    let real: Vec<u8> = (100..110).flat_map(m20_frame).collect();
    stream.extend(&real);
    let mut decoder = Decoder::default();
    decoder.push(&stream);
    let tids: Vec<u16> = decode_all(&mut decoder).iter().map(|frame| frame.tid).collect();
    assert_eq!(tids, (100..110).collect::<Vec<_>>());
    assert_eq!(decoder.stats().checksum_errors, 1);
}

#[test]
fn a_truncated_frame_followed_by_a_good_one_recovers() {
    let mut stream = m20_frame(1);
    stream.truncate(30);
    stream.extend(m20_frame(2));
    stream.extend(m20_frame(3));
    let mut decoder = Decoder::default();
    decoder.push(&stream);
    let tids: Vec<u16> = decode_all(&mut decoder).iter().map(|frame| frame.tid).collect();
    assert_eq!(tids, [2, 3]);
}

#[test]
fn a_block_running_past_the_payload_keeps_the_blocks_before_it() {
    // payload: accel block, then a block header claiming 40 bytes with only 2 left
    let accel = s32s(&[1, 2, 3]);
    let mut payload = vec![ACCEL_ID, 12];
    payload.extend(&accel);
    payload.extend([GYRO_ID, 40, 0, 0]);
    let mut frame = vec![0x59, 0x53, 5, 0, payload.len() as u8];
    frame.extend(&payload);
    let crc = checksum(&frame[2..]);
    frame.extend(crc);
    let mut decoder = Decoder::default();
    decoder.push(&frame);
    let decoded = decoder.next_frame().unwrap();
    assert!(decoded.accel_mps2.is_some());
    assert!(decoded.gyro_rad_s.is_none());
    assert_eq!(decoder.stats().malformed_payloads, 1);
}

#[test]
fn the_buffer_is_capped() {
    let mut decoder = Decoder::new(DecoderConfig { max_buffered_bytes: 64 });
    decoder.push(&[0x59; 1000]);
    assert!(decoder.buffered() <= 64);
    assert_eq!(decoder.stats().overflow_bytes, 1000 - 64);
    // the flood clears and a frame after it decodes
    while decoder.next_frame().is_some() {}
    decoder.push(&m20_frame(9));
    assert_eq!(decoder.next_frame().map(|frame| frame.tid), Some(9));
}

#[test]
fn euler_to_quaternion_matches_the_m20_vendor_driver() {
    // the vendor yesense_node's published orientation for euler (pitch, roll, yaw) = (10, 20, 30) deg, from
    // emulating its publish_msg: Rz(yaw) Ry(pitch) Rx(roll), then its fixed Rx(-90 deg) mount correction
    let euler = Euler {
        roll: 20f64.to_radians(),
        pitch: 10f64.to_radians(),
        yaw: 30f64.to_radians(),
    };
    let [w, x, y, z] = euler.to_quaternion();
    let half = std::f64::consts::FRAC_1_SQRT_2;
    // q * (cos(-45), sin(-45), 0, 0)
    let (mw, mx) = (half, -half);
    let out = [w * mw - x * mx, w * mx + x * mw, y * mw + z * mx, z * mw - y * mx];
    let vendor = [0.77529, -0.5704, -0.07893, 0.25949];
    for (ours, theirs) in out.iter().zip(vendor) {
        assert!(close(*ours, theirs, 1e-4), "{out:?} vs {vendor:?}");
    }
    // identity at zero
    assert_eq!(Euler::default().to_quaternion(), [1.0, 0.0, 0.0, 0.0]);
}
