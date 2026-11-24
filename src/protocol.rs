//! The Yesense standard output protocol (`yis_std_out_decoder` in Yesense's SDK).
//!
//! A frame is `0x59 0x53 | tid (u16 LE) | len (u8) | payload (len bytes) | ck1 ck2`. The payload is a run of
//! blocks `data_id (u8) | data_len (u8) | data`. The checksum is a Fletcher-style pair of byte sums over tid, len
//! and payload. Values are little-endian fixed point; the factors below are the SDK's.

pub const HEADER: [u8; 2] = [0x59, 0x53];
/// header (2) + tid (2) + len (1)
pub const PREFIX_LEN: usize = 5;
/// prefix + ck1 + ck2
pub const MIN_FRAME_LEN: usize = PREFIX_LEN + 2;

pub const SENSOR_TEMP_ID: u8 = 0x01;
pub const ACCEL_ID: u8 = 0x10;
pub const GYRO_ID: u8 = 0x20;
pub const MAGNETIC_NORM_ID: u8 = 0x30;
pub const RAW_MAGNETIC_ID: u8 = 0x31;
pub const EULER_ID: u8 = 0x40;
pub const QUATERNION_ID: u8 = 0x41;
pub const UTC_ID: u8 = 0x50;
pub const SAMPLE_TIMESTAMP_ID: u8 = 0x51;
pub const DATA_READY_TIMESTAMP_ID: u8 = 0x52;
pub const LOCATION_ID: u8 = 0x68;
pub const SPEED_ID: u8 = 0x70;
pub const STATUS_ID: u8 = 0x80;

const SENSOR_TEMP_FACTOR: f64 = 0.01;
const ACC_FACTOR: f64 = 1e-6;
const GYRO_FACTOR: f64 = 1e-6;
const MAG_NORM_FACTOR: f64 = 1e-6;
const MAG_RAW_FACTOR: f64 = 1e-3;
const EULER_FACTOR: f64 = 1e-6;
const QUATERNION_FACTOR: f64 = 1e-6;
const LAT_LON_FACTOR: f64 = 1e-10;
const ALT_FACTOR: f64 = 1e-3;
const VEL_FACTOR: f64 = 1e-3;

/// Euler angles in radians as the device reports them (it sends pitch, roll, yaw in degrees).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Euler {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

impl Euler {
    /// `Rz(yaw) * Ry(pitch) * Rx(roll)` as `[w, x, y, z]`: the composition Deep Robotics' M20 driver uses.
    pub fn to_quaternion(&self) -> [f64; 4] {
        let (sr, cr) = (self.roll * 0.5).sin_cos();
        let (sp, cp) = (self.pitch * 0.5).sin_cos();
        let (sy, cy) = (self.yaw * 0.5).sin_cos();
        [
            cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            cy * sp * cr + sy * cp * sr,
            sy * cp * cr - cy * sp * sr,
        ]
    }
}

/// Position from a navigation product (data id 0x68).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Position {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_m: f64,
}

/// East/north/up velocity in m/s (data id 0x70).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Velocity {
    pub east: f64,
    pub north: f64,
    pub up: f64,
}

/// One decoded frame; each field is `Some` only when the device sent that block.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Frame {
    /// Frame counter from the device (1..=60000, wraps).
    pub tid: u16,
    pub temperature_c: Option<f64>,
    pub accel_mps2: Option<[f64; 3]>,
    pub gyro_rad_s: Option<[f64; 3]>,
    /// Normalized magnetic field (unitless).
    pub mag_norm: Option<[f64; 3]>,
    pub mag_raw_mgauss: Option<[f64; 3]>,
    pub euler: Option<Euler>,
    /// Orientation quaternion `[w, x, y, z]` as the device computed it.
    pub quaternion: Option<[f64; 4]>,
    /// The raw 11-byte UTC block (the SDK does not decode it).
    pub utc_raw: Option<[u8; 11]>,
    /// Sensor sample time, in device ticks (us or 100 us depending on configuration).
    pub sample_timestamp: Option<u32>,
    /// Data-ready time, in device ticks (us or 100 us depending on configuration).
    pub data_ready_timestamp: Option<u32>,
    pub position: Option<Position>,
    pub velocity: Option<Velocity>,
    /// Navigation status byte: fusion status in the low nibble, position status in the high nibble.
    pub nav_status: Option<u8>,
}

impl Frame {
    /// The device's quaternion if it sent one, else the quaternion of its Euler angles.
    pub fn orientation(&self) -> Option<[f64; 4]> {
        self.quaternion
            .or_else(|| self.euler.map(|euler| euler.to_quaternion()))
    }
}

/// The checksum over `bytes` (tid, len and payload): `ck1` is the byte sum, `ck2` the sum of the running `ck1`s.
pub fn checksum(bytes: &[u8]) -> [u8; 2] {
    let (mut ck1, mut ck2) = (0u8, 0u8);
    for byte in bytes {
        ck1 = ck1.wrapping_add(*byte);
        ck2 = ck2.wrapping_add(ck1);
    }
    [ck1, ck2]
}

/// A full frame from `blocks` of `(data_id, data)`, the inverse of [`Decoder`] (for tests and simulators).
///
/// # Panics
/// When the blocks do not fit one frame (a payload over 255 bytes); [`try_encode_frame`] returns `None` instead.
#[allow(clippy::expect_used)]
pub fn encode_frame(tid: u16, blocks: &[(u8, &[u8])]) -> Vec<u8> {
    try_encode_frame(tid, blocks).expect("payload over 255 bytes")
}

/// [`encode_frame`], or `None` when the blocks do not fit one frame (a payload over 255 bytes).
pub fn try_encode_frame(tid: u16, blocks: &[(u8, &[u8])]) -> Option<Vec<u8>> {
    let mut payload = Vec::new();
    for (id, data) in blocks {
        payload.push(*id);
        payload.push(u8::try_from(data.len()).ok()?);
        payload.extend_from_slice(data);
    }
    let payload_len = u8::try_from(payload.len()).ok()?;
    let mut frame = HEADER.to_vec();
    frame.extend_from_slice(&tid.to_le_bytes());
    frame.push(payload_len);
    frame.extend_from_slice(&payload);
    let crc = checksum(frame.get(2..).unwrap_or_default());
    frame.extend_from_slice(&crc);
    Some(frame)
}

/// Tunables for [`Decoder`].
#[derive(Clone, Copy, Debug)]
pub struct DecoderConfig {
    /// Bytes held while waiting for a frame to complete; past this the oldest bytes are dropped. One frame is at
    /// most 262 bytes, so anything above that only matters for how much a burst can queue.
    pub max_buffered_bytes: usize,
}

impl Default for DecoderConfig {
    fn default() -> Self {
        Self {
            max_buffered_bytes: 4096,
        }
    }
}

/// Counters of what the decoder kept and dropped.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DecoderStats {
    pub frames: u64,
    pub checksum_errors: u64,
    /// Bytes discarded while hunting for a header (garbage, or the first byte of a frame that failed its checksum).
    pub skipped_bytes: u64,
    /// Blocks skipped by their length because the id is unknown or its length does not match the id.
    pub unknown_blocks: u64,
    /// Frames whose block list ran past the payload (the blocks before that point are kept).
    pub malformed_payloads: u64,
    /// Bytes dropped because the buffer passed `max_buffered_bytes`.
    pub overflow_bytes: u64,
}

/// Streaming decoder: [`push`](Decoder::push) bytes as they arrive, take frames with [`next_frame`](Decoder::next_frame).
#[derive(Debug)]
pub struct Decoder {
    config: DecoderConfig,
    buffer: Vec<u8>,
    stats: DecoderStats,
}

impl Default for Decoder {
    fn default() -> Self {
        Self::new(DecoderConfig::default())
    }
}

impl Decoder {
    pub fn new(config: DecoderConfig) -> Self {
        Self {
            config,
            buffer: Vec::new(),
            stats: DecoderStats::default(),
        }
    }

    pub fn stats(&self) -> DecoderStats {
        self.stats
    }

    /// Bytes held waiting for the rest of a frame.
    pub fn buffered(&self) -> usize {
        self.buffer.len()
    }

    /// Drop the bytes held (counts stay).
    pub fn clear(&mut self) {
        self.buffer.clear();
    }

    pub fn push(&mut self, bytes: &[u8]) {
        self.buffer.extend_from_slice(bytes);
        let limit = self.config.max_buffered_bytes.max(MIN_FRAME_LEN);
        if self.buffer.len() > limit {
            let excess = self.buffer.len() - limit;
            self.buffer.drain(..excess);
            self.stats.overflow_bytes += excess as u64;
        }
    }

    /// The next complete, checksum-valid frame, or `None` until more bytes arrive.
    pub fn next_frame(&mut self) -> Option<Frame> {
        loop {
            let start = match self.buffer.windows(2).position(|pair| pair == HEADER) {
                Some(start) => start,
                None => {
                    // keep a trailing 0x59, it may be the first half of a header
                    let keep = usize::from(self.buffer.last() == Some(&HEADER[0]));
                    let drop = self.buffer.len() - keep;
                    self.skip(drop);
                    return None;
                }
            };
            self.skip(start);
            let &payload_len = self.buffer.get(PREFIX_LEN - 1)?;
            let payload_end = PREFIX_LEN + usize::from(payload_len);
            let frame_len = payload_end + 2;
            let (Some(covered), Some(received_crc)) =
                (self.buffer.get(2..payload_end), self.buffer.get(payload_end..frame_len))
            else {
                return None;
            };
            if checksum(covered) != received_crc {
                // a false header or a corrupt frame: step past this 0x59 and hunt again
                self.stats.checksum_errors += 1;
                self.skip(1);
                continue;
            }
            let &[tid_low, tid_high, _len, ref payload @ ..] = covered else {
                return None;
            };
            let (frame, unknown_blocks, malformed) = parse_payload(u16::from_le_bytes([tid_low, tid_high]), payload);
            self.stats.unknown_blocks += unknown_blocks;
            self.stats.malformed_payloads += u64::from(malformed);
            self.buffer.drain(..frame_len);
            self.stats.frames += 1;
            return Some(frame);
        }
    }

    fn skip(&mut self, count: usize) {
        self.buffer.drain(..count);
        self.stats.skipped_bytes += count as u64;
    }
}

/// The frame from a checksum-valid payload, with how many blocks were skipped and whether the block list overran it.
fn parse_payload(tid: u16, payload: &[u8]) -> (Frame, u64, bool) {
    let mut frame = Frame {
        tid,
        ..Frame::default()
    };
    let mut unknown_blocks = 0;
    let mut at = 0;
    while at < payload.len() {
        let Some(&[id, len]) = payload.get(at..at + 2) else {
            return (frame, unknown_blocks, true);
        };
        let data_end = at + 2 + usize::from(len);
        let Some(data) = payload.get(at + 2..data_end) else {
            return (frame, unknown_blocks, true);
        };
        // the SDK steps one byte past a block it cannot parse; the checksum already passed, so skip by its length
        if !parse_block(id, data, &mut frame) {
            unknown_blocks += 1;
        }
        at = data_end;
    }
    (frame, unknown_blocks, false)
}

fn le_bytes<const N: usize>(data: &[u8], at: usize) -> Option<[u8; N]> {
    data.get(at..at.checked_add(N)?)?.try_into().ok()
}

fn i16_at(data: &[u8], at: usize) -> Option<i16> {
    le_bytes(data, at).map(i16::from_le_bytes)
}

fn i32_at(data: &[u8], at: usize) -> Option<i32> {
    le_bytes(data, at).map(i32::from_le_bytes)
}

fn u32_at(data: &[u8], at: usize) -> Option<u32> {
    le_bytes(data, at).map(u32::from_le_bytes)
}

fn i64_at(data: &[u8], at: usize) -> Option<i64> {
    le_bytes(data, at).map(i64::from_le_bytes)
}

fn scaled<const N: usize>(data: &[u8], factor: f64) -> Option<[f64; N]> {
    let mut values = [0.0; N];
    for (axis, value) in values.iter_mut().enumerate() {
        *value = f64::from(i32_at(data, axis * 4)?) * factor;
    }
    Some(values)
}

/// Fill `frame` from one block; false when the id is unknown or the length does not match it.
fn parse_block(id: u8, data: &[u8], frame: &mut Frame) -> bool {
    parse_known_block(id, data, frame).is_some()
}

fn parse_known_block(id: u8, data: &[u8], frame: &mut Frame) -> Option<()> {
    match (id, data.len()) {
        (SENSOR_TEMP_ID, 2) => frame.temperature_c = Some(f64::from(i16_at(data, 0)?) * SENSOR_TEMP_FACTOR),
        (ACCEL_ID, 12) => frame.accel_mps2 = Some(scaled(data, ACC_FACTOR)?),
        (GYRO_ID, 12) => frame.gyro_rad_s = Some(scaled::<3>(data, GYRO_FACTOR)?.map(f64::to_radians)),
        (MAGNETIC_NORM_ID, 12) => frame.mag_norm = Some(scaled(data, MAG_NORM_FACTOR)?),
        (RAW_MAGNETIC_ID, 12) => frame.mag_raw_mgauss = Some(scaled(data, MAG_RAW_FACTOR)?),
        (EULER_ID, 12) => {
            // the device sends pitch, roll, yaw
            let [pitch, roll, yaw] = scaled::<3>(data, EULER_FACTOR)?.map(f64::to_radians);
            frame.euler = Some(Euler { roll, pitch, yaw });
        }
        (QUATERNION_ID, 16) => frame.quaternion = Some(scaled(data, QUATERNION_FACTOR)?),
        (UTC_ID, 11) => frame.utc_raw = Some(le_bytes(data, 0)?),
        (SAMPLE_TIMESTAMP_ID, 4) => frame.sample_timestamp = Some(u32_at(data, 0)?),
        (DATA_READY_TIMESTAMP_ID, 4) => frame.data_ready_timestamp = Some(u32_at(data, 0)?),
        (LOCATION_ID, 20) => {
            frame.position = Some(Position {
                latitude_deg: i64_at(data, 0)? as f64 * LAT_LON_FACTOR,
                longitude_deg: i64_at(data, 8)? as f64 * LAT_LON_FACTOR,
                altitude_m: f64::from(i32_at(data, 16)?) * ALT_FACTOR,
            })
        }
        (SPEED_ID, 12) => {
            let [east, north, up] = scaled(data, VEL_FACTOR)?;
            frame.velocity = Some(Velocity { east, north, up });
        }
        (STATUS_ID, 1) => frame.nav_status = Some(*data.first()?),
        _ => return None,
    }
    Some(())
}
