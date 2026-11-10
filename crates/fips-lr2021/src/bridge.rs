//! Host↔bridge protocol over USB CDC.
//!
//! ```text
//! [A5 5A][type:1][len:2 LE][payload][CRC16:2 LE]
//! ```

use std::io::{self, Read, Write};
use std::time::Duration;

use thiserror::Error;

use crate::config::{Band, RadioProfile};
use crate::crc::crc16;

pub const MAGIC: [u8; 2] = [0xA5, 0x5A];
pub const MAX_PAYLOAD: usize = 2048;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BridgeKind {
    Hello = 1,
    HelloAck = 2,
    Cfg = 3,
    CfgAck = 4,
    Tx = 5,
    Rx = 6,
    Stat = 7,
    Evt = 8,
    Ping = 9,
    Pong = 10,
    CfgExt = 11,
}

impl BridgeKind {
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            1 => Some(Self::Hello),
            2 => Some(Self::HelloAck),
            3 => Some(Self::Cfg),
            4 => Some(Self::CfgAck),
            5 => Some(Self::Tx),
            6 => Some(Self::Rx),
            7 => Some(Self::Stat),
            8 => Some(Self::Evt),
            9 => Some(Self::Ping),
            10 => Some(Self::Pong),
            11 => Some(Self::CfgExt),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BridgeFrame {
    pub kind: BridgeKind,
    pub payload: Vec<u8>,
}

pub fn encode(kind: BridgeKind, payload: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(6 + payload.len() + 2);
    out.extend_from_slice(&MAGIC);
    out.push(kind as u8);
    out.extend_from_slice(&(payload.len() as u16).to_le_bytes());
    out.extend_from_slice(payload);
    let crc = crc16(&out[2..]);
    out.extend_from_slice(&crc.to_le_bytes());
    out
}

/// Incremental decoder. It is deliberately tolerant of leading garbage, which
/// matters after a USB reconnect or a bridge reset.
#[derive(Debug, Default)]
pub struct BridgeDecoder {
    buf: Vec<u8>,
}

impl BridgeDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, data: &[u8]) -> Vec<BridgeFrame> {
        self.buf.extend_from_slice(data);
        let mut out = Vec::new();
        loop {
            if self.buf.len() < 5 {
                break;
            }
            if self.buf[0..2] != MAGIC {
                let next = self
                    .buf
                    .windows(2)
                    .position(|w| w == MAGIC)
                    .unwrap_or(self.buf.len());
                self.buf.drain(..next.max(1));
                continue;
            }
            let len = u16::from_le_bytes([self.buf[3], self.buf[4]]) as usize;
            if len > MAX_PAYLOAD {
                self.buf.drain(..1);
                continue;
            }
            let total = 7 + len;
            if self.buf.len() < total {
                break;
            }
            let expected = u16::from_le_bytes([self.buf[total - 2], self.buf[total - 1]]);
            let actual = crc16(&self.buf[2..total - 2]);
            if expected != actual {
                self.buf.drain(..1);
                continue;
            }
            if let Some(kind) = BridgeKind::from_u8(self.buf[2]) {
                out.push(BridgeFrame {
                    kind,
                    payload: self.buf[5..total - 2].to_vec(),
                });
            }
            self.buf.drain(..total);
        }
        out
    }
}

#[derive(Debug, Error)]
pub enum BridgeError {
    #[error("serial port: {0}")]
    Serial(#[from] serialport::Error),
    #[error("io: {0}")]
    Io(#[from] io::Error),
    #[error("protocol: {0}")]
    Protocol(String),
}

/// Blocking serial wrapper. The caller owns the read and write loops.
pub struct SerialBridge {
    port: Box<dyn serialport::SerialPort>,
    decoder: BridgeDecoder,
}

impl SerialBridge {
    pub fn open(path: &str, baud: u32) -> Result<Self, BridgeError> {
        let port = serialport::new(path, baud)
            .timeout(Duration::from_millis(50))
            .open()?;
        Ok(Self {
            port,
            decoder: BridgeDecoder::new(),
        })
    }

    pub fn try_clone(&self) -> Result<Self, BridgeError> {
        Ok(Self {
            port: self.port.try_clone()?,
            decoder: BridgeDecoder::new(),
        })
    }

    pub fn write_frame(&mut self, kind: BridgeKind, payload: &[u8]) -> Result<(), BridgeError> {
        let bytes = encode(kind, payload);
        self.port.write_all(&bytes)?;
        self.port.flush()?;
        Ok(())
    }

    /// Read whatever is available and return complete bridge frames.
    pub fn read_frames(&mut self, timeout_ms: u64) -> Result<Vec<BridgeFrame>, BridgeError> {
        self.port
            .set_timeout(Duration::from_millis(timeout_ms.max(1)))?;
        let mut buf = [0u8; 512];
        let mut out = Vec::new();
        match self.port.read(&mut buf) {
            Ok(0) => {}
            Ok(n) => out.extend(self.decoder.push(&buf[..n])),
            Err(err) if err.kind() == io::ErrorKind::TimedOut => {}
            Err(err) if err.kind() == io::ErrorKind::Interrupted => {}
            Err(err) => return Err(BridgeError::Io(err)),
        }
        Ok(out)
    }
}

/// Radio configuration sent in a `Cfg` frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CfgPayload {
    pub profile: RadioProfile,
    pub relay: bool,
}

impl CfgPayload {
    pub fn encode(&self) -> Vec<u8> {
        let mut flags = 0u8;
        if self.relay {
            flags |= 0x01;
        }
        if self.profile.sub_ghz {
            flags |= 0x02;
        }
        let mut out = Vec::with_capacity(15);
        out.push(band_to_u8(self.profile.band));
        out.extend_from_slice(&self.profile.frequency_hz.to_le_bytes());
        out.extend_from_slice(&self.profile.bitrate_kbps.to_le_bytes());
        out.push(self.profile.tx_power_dbm as u8);
        out.extend_from_slice(&self.profile.sync_word);
        out.extend_from_slice(&self.profile.radio_payload.to_le_bytes());
        out.push(flags);
        out
    }

    pub fn decode(payload: &[u8]) -> Option<Self> {
        if payload.len() != 15 {
            return None;
        }
        let band = band_from_u8(payload[0])?;
        let frequency_hz = u32::from_le_bytes(payload[1..5].try_into().ok()?);
        let bitrate_kbps = u16::from_le_bytes(payload[5..7].try_into().ok()?);
        let tx_power_dbm = payload[7] as i8;
        let sync_word = payload[8..12].try_into().ok()?;
        let radio_payload = u16::from_le_bytes(payload[12..14].try_into().ok()?);
        let flags = payload[14];
        let profile = RadioProfile {
            band,
            frequency_hz,
            bitrate_kbps,
            tx_power_dbm,
            sync_word,
            radio_payload,
            sub_ghz: flags & 0x02 != 0,
        };
        profile.validate().ok()?;
        Some(Self {
            profile,
            relay: flags & 0x01 != 0,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CfgAckPayload {
    pub status: u8,
    pub frequency_hz: u32,
    pub bitrate_kbps: u16,
    pub tx_power_dbm: i8,
    pub radio_payload: u16,
}

impl CfgAckPayload {
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(10);
        out.push(self.status);
        out.extend_from_slice(&self.frequency_hz.to_le_bytes());
        out.extend_from_slice(&self.bitrate_kbps.to_le_bytes());
        out.push(self.tx_power_dbm as u8);
        out.extend_from_slice(&self.radio_payload.to_le_bytes());
        out
    }

    pub fn decode(payload: &[u8]) -> Option<Self> {
        if payload.len() != 10 {
            return None;
        }
        Some(Self {
            status: payload[0],
            frequency_hz: u32::from_le_bytes(payload[1..5].try_into().ok()?),
            bitrate_kbps: u16::from_le_bytes(payload[5..7].try_into().ok()?),
            tx_power_dbm: payload[7] as i8,
            radio_payload: u16::from_le_bytes(payload[8..10].try_into().ok()?),
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HelloAckPayload {
    pub protocol: u8,
    pub firmware: [u8; 3],
    pub chip_id: u16,
    pub capabilities: u16,
}

impl HelloAckPayload {
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(8);
        out.push(self.protocol);
        out.extend_from_slice(&self.firmware);
        out.extend_from_slice(&self.chip_id.to_le_bytes());
        out.extend_from_slice(&self.capabilities.to_le_bytes());
        out
    }

    pub fn decode(payload: &[u8]) -> Option<Self> {
        if payload.len() != 8 {
            return None;
        }
        Some(Self {
            protocol: payload[0],
            firmware: payload[1..4].try_into().ok()?,
            chip_id: u16::from_le_bytes(payload[4..6].try_into().ok()?),
            capabilities: u16::from_le_bytes(payload[6..8].try_into().ok()?),
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RxPayload {
    pub rssi_dbm: i16,
    pub snr_db: i8,
    pub crc_ok: bool,
    pub data: Vec<u8>,
}

impl RxPayload {
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(6 + self.data.len());
        out.extend_from_slice(&self.rssi_dbm.to_le_bytes());
        out.push(self.snr_db as u8);
        out.push(u8::from(self.crc_ok));
        out.extend_from_slice(&(self.data.len() as u16).to_le_bytes());
        out.extend_from_slice(&self.data);
        out
    }

    pub fn decode(payload: &[u8]) -> Option<Self> {
        if payload.len() < 6 {
            return None;
        }
        let len = u16::from_le_bytes(payload[4..6].try_into().ok()?) as usize;
        if payload.len() != 6 + len {
            return None;
        }
        Some(Self {
            rssi_dbm: i16::from_le_bytes(payload[0..2].try_into().ok()?),
            snr_db: payload[2] as i8,
            crc_ok: payload[3] != 0,
            data: payload[6..].to_vec(),
        })
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct StatPayload {
    pub tx_packets: u32,
    pub rx_packets: u32,
    pub crc_errors: u32,
    pub drops: u32,
    pub tx_timeouts: u32,
    pub resets: u32,
}

impl StatPayload {
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(24);
        for value in [
            self.tx_packets,
            self.rx_packets,
            self.crc_errors,
            self.drops,
            self.tx_timeouts,
            self.resets,
        ] {
            out.extend_from_slice(&value.to_le_bytes());
        }
        out
    }

    pub fn decode(payload: &[u8]) -> Option<Self> {
        if payload.len() != 24 {
            return None;
        }
        let mut values = [0u32; 6];
        for (i, value) in values.iter_mut().enumerate() {
            *value = u32::from_le_bytes(payload[i * 4..i * 4 + 4].try_into().ok()?);
        }
        Some(Self {
            tx_packets: values[0],
            rx_packets: values[1],
            crc_errors: values[2],
            drops: values[3],
            tx_timeouts: values[4],
            resets: values[5],
        })
    }
}

pub fn band_to_u8(band: Band) -> u8 {
    match band {
        Band::Eu868 => 0,
        Band::Eu868High => 1,
        Band::Us915 => 2,
        Band::Us915Robust => 3,
        Band::Eu2400 => 4,
        Band::Us2400 => 5,
        Band::Custom => 6,
    }
}

pub fn band_from_u8(value: u8) -> Option<Band> {
    match value {
        0 => Some(Band::Eu868),
        1 => Some(Band::Eu868High),
        2 => Some(Band::Us915),
        3 => Some(Band::Us915Robust),
        4 => Some(Band::Eu2400),
        5 => Some(Band::Us2400),
        6 => Some(Band::Custom),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Band;

    fn sample_frame(kind: BridgeKind, payload: &[u8]) -> Vec<u8> {
        encode(kind, payload)
    }

    #[test]
    fn cfg_ext_frame_roundtrips_through_decoder() {
        let payload = [1u8, 0, 0, 0, 0x10, 0x20, 0, 0, 0, 0, 0, 0];
        let mut decoder = BridgeDecoder::new();
        let frames = decoder.push(&encode(BridgeKind::CfgExt, &payload));
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].kind, BridgeKind::CfgExt);
        assert_eq!(frames[0].payload, payload);
    }

    #[test]
    fn decodes_split_and_coalesced_frames() {
        let a = sample_frame(BridgeKind::Tx, b"hello");
        let b = sample_frame(BridgeKind::Ping, b"");
        let mut stream = a.clone();
        stream.extend_from_slice(&b);
        let mut decoder = BridgeDecoder::new();
        assert!(decoder.push(&stream[..3]).is_empty());
        let frames = decoder.push(&stream[3..]);
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].kind, BridgeKind::Tx);
        assert_eq!(frames[0].payload, b"hello");
        assert_eq!(frames[1].kind, BridgeKind::Ping);
    }

    #[test]
    fn skips_leading_garbage_and_bad_crc() {
        let mut stream = vec![0x00, 0xFF, 0x11];
        stream.extend_from_slice(&sample_frame(BridgeKind::Tx, b"ok"));
        let mut decoder = BridgeDecoder::new();
        let frames = decoder.push(&stream);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].payload, b"ok");
    }

    #[test]
    fn cfg_roundtrip() {
        let cfg = CfgPayload {
            profile: Band::Us915.default_profile(),
            relay: true,
        };
        let encoded = cfg.encode();
        assert_eq!(CfgPayload::decode(&encoded), Some(cfg));
    }

    #[test]
    fn rx_roundtrip() {
        let rx = RxPayload {
            rssi_dbm: -42,
            snr_db: 9,
            crc_ok: true,
            data: vec![1, 2, 3, 4, 5],
        };
        assert_eq!(RxPayload::decode(&rx.encode()), Some(rx));
    }
}
