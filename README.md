# yesense-imu

Rust driver for Yesense IMUs (YIS series) on a serial port: the standard binary output protocol decoded into
acceleration (m/s²), angular rate (rad/s), Euler angles and quaternion, temperature, sensor timestamps, and the host
time each frame arrived. Pure Rust (only `libc` for termios), Linux and macOS.

```rust
let mut imu = yesense_imu::Imu::open(&yesense_imu::Config::new("/dev/ttyS9", 115200))?;
for sample in &mut imu {
    let sample = sample?;
    println!("{:?} {:?} {:?}", sample.host_time, sample.frame.accel_mps2, sample.frame.orientation());
}
```

`yesense_imu::Decoder` is the transport-free part: `push` bytes, take `next_frame`s. It resynchronizes on garbage,
false headers, truncated and corrupt frames (bad checksum: step one byte and hunt for `0x59 0x53` again), and
counts what it dropped (`stats()`). `yesense-dump <device> <baud> [--count N] [--raw FILE]` prints samples or
records raw bytes.

Every field is `Option`: a frame carries only the blocks the device is configured to send. `Frame::orientation()`
gives the device's quaternion, or the quaternion of its Euler angles (`Rz(yaw) Ry(pitch) Rx(roll)`) when it sends
only Euler angles.

## Protocol source

The frame layout, data ids, lengths, scale factors and checksum follow Yesense's own decoder,
`yis_std_out_decoder` from the Yesense ROS 2 SDK
([yanjingang/yesense_ros2 @ 87417c3](https://github.com/yanjingang/yesense_ros2/blob/87417c3eacbab3164cdc740ea591d89d4150fb62/yesense_std_ros2/src/yesense_std_out_decoder.cpp),
[header](https://github.com/yanjingang/yesense_ros2/blob/87417c3eacbab3164cdc740ea591d89d4150fb62/yesense_std_ros2/include/yesense_std_out_decoder.h)).
Two deliberate differences: a block with an unknown id or a wrong length is skipped by its length (the SDK steps
one byte), and after a checksum failure the decoder steps one byte past the header (the SDK clears up to a stale
end index).

The Euler composition is the one Deep Robotics' M20 `yesense_node` uses, checked against an emulation of that
binary.

Tested against frames built from the spec; not yet against a live capture.

## License

Apache-2.0
