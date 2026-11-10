//! On-air link frame format.
//!
//! ```text
//! magic F1 50 | version/type | flags | src(4) | dst(4) |
//! epoch(2) | frame_seq(2) | frag_idx(1) | frag_count(1) |
//! payload_len(2) | payload | CRC32(4)
//! ```

use crate::crc::crc32;
use thiserror::Error;

pub const MAGIC: [u8; 2] = [0xF1, 0x50];
pub const VERSION: u8 = 1;
pub const HEADER_SIZE: usize = 20;
pub const CRC_SIZE: usize = 4;
pub const MAX_FRAGMENTS: u8 = 255;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FrameKind {
    Data = 0,
    Ack = 1,
    Nack = 2,
    Beacon = 3,
}

impl FrameKind {
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Data),
            1 => Some(Self::Ack),
            2 => Some(Self::Nack),
            3 => Some(Self::Beacon),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame<'a> {
    pub kind: FrameKind,
    pub flags: u8,
    pub src: u32,
    pub dst: u32,
    pub epoch: u16,
    pub seq: u16,
    pub frag_idx: u8,
    pub frag_count: u8,
    pub payload: &'a [u8],
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum FrameError {
    #[error("frame too short: {0} bytes")]
    TooShort(usize),
    #[error("bad magic")]
    BadMagic,
    #[error("unsupported frame version {0}")]
    BadVersion(u8),
    #[error("unknown frame type {0}")]
    BadKind(u8),
    #[error("payload length {declared} does not match frame length {actual}")]
    LengthMismatch { declared: usize, actual: usize },
    #[error("payload length {0} exceeds u16")]
    PayloadTooLong(usize),
    #[error("bad crc32")]
    BadCrc,
}

impl<'a> Frame<'a> {
    pub fn data(
        src: u32,
        dst: u32,
        epoch: u16,
        seq: u16,
        frag_idx: u8,
        frag_count: u8,
        payload: &'a [u8],
    ) -> Self {
        Self {
            kind: FrameKind::Data,
            flags: 0,
            src,
            dst,
            epoch,
            seq,
            frag_idx,
            frag_count,
            payload,
        }
    }

    pub fn encode(&self, out: &mut Vec<u8>) -> Result<(), FrameError> {
        if self.payload.len() > u16::MAX as usize {
            return Err(FrameError::PayloadTooLong(self.payload.len()));
        }
        out.clear();
        out.reserve(HEADER_SIZE + self.payload.len() + CRC_SIZE);
        out.extend_from_slice(&MAGIC);
        out.push((VERSION << 4) | (self.kind as u8));
        out.push(self.flags);
        out.extend_from_slice(&self.src.to_le_bytes());
        out.extend_from_slice(&self.dst.to_le_bytes());
        out.extend_from_slice(&self.epoch.to_le_bytes());
        out.extend_from_slice(&self.seq.to_le_bytes());
        out.push(self.frag_idx);
        out.push(self.frag_count);
        out.extend_from_slice(&(self.payload.len() as u16).to_le_bytes());
        out.extend_from_slice(self.payload);
        let crc = crc32(out);
        out.extend_from_slice(&crc.to_le_bytes());
        Ok(())
    }

    pub fn decode(buf: &'a [u8]) -> Result<Self, FrameError> {
        if buf.len() < HEADER_SIZE + CRC_SIZE {
            return Err(FrameError::TooShort(buf.len()));
        }
        if buf[0..2] != MAGIC {
            return Err(FrameError::BadMagic);
        }
        let version = buf[2] >> 4;
        if version != VERSION {
            return Err(FrameError::BadVersion(version));
        }
        let kind = FrameKind::from_u8(buf[2] & 0x0F).ok_or(FrameError::BadKind(buf[2] & 0x0F))?;
        let flags = buf[3];
        let src = u32::from_le_bytes(buf[4..8].try_into().unwrap());
        let dst = u32::from_le_bytes(buf[8..12].try_into().unwrap());
        let epoch = u16::from_le_bytes(buf[12..14].try_into().unwrap());
        let seq = u16::from_le_bytes(buf[14..16].try_into().unwrap());
        let frag_idx = buf[16];
        let frag_count = buf[17];
        let declared = u16::from_le_bytes(buf[18..20].try_into().unwrap()) as usize;
        let actual = buf.len() - HEADER_SIZE - CRC_SIZE;
        if declared != actual {
            return Err(FrameError::LengthMismatch { declared, actual });
        }
        let crc_offset = HEADER_SIZE + declared;
        let expected = u32::from_le_bytes(buf[crc_offset..crc_offset + 4].try_into().unwrap());
        if crc32(&buf[..crc_offset]) != expected {
            return Err(FrameError::BadCrc);
        }
        Ok(Self {
            kind,
            flags,
            src,
            dst,
            epoch,
            seq,
            frag_idx,
            frag_count,
            payload: &buf[HEADER_SIZE..crc_offset],
        })
    }
}

/// Build the payload of an ACK/NACK frame: fragment count plus received bitmap.
pub fn ack_payload(frag_count: u8, bitmap: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(1 + bitmap.len());
    out.push(frag_count);
    out.extend_from_slice(bitmap);
    out
}

/// Parse an ACK/NACK payload into `(frag_count, bitmap)`.
pub fn parse_ack_payload(payload: &[u8]) -> Option<(u8, &[u8])> {
    let (&frag_count, bitmap) = payload.split_first()?;
    if frag_count == 0 {
        return None;
    }
    let needed = frag_count.div_ceil(8) as usize;
    if bitmap.len() != needed {
        return None;
    }
    Some((frag_count, bitmap))
}

pub fn bitmap_len(frag_count: u8) -> usize {
    frag_count.div_ceil(8) as usize
}

pub fn bitmap_get(bitmap: &[u8], index: u8) -> bool {
    let byte = index as usize / 8;
    let bit = index as usize % 8;
    bitmap.get(byte).is_some_and(|b| b & (1 << bit) != 0)
}

pub fn bitmap_set(bitmap: &mut [u8], index: u8) {
    let byte = index as usize / 8;
    let bit = index as usize % 8;
    if let Some(b) = bitmap.get_mut(byte) {
        *b |= 1 << bit;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample<'a>(payload: &'a [u8]) -> Frame<'a> {
        Frame::data(0x0102_0304, 0xFFFF_FFFF, 7, 42, 1, 3, payload)
    }

    #[test]
    fn roundtrip() {
        let payload = b"hello lr2021";
        let frame = sample(payload);
        let mut buf = Vec::new();
        frame.encode(&mut buf).unwrap();
        assert_eq!(buf.len(), HEADER_SIZE + payload.len() + CRC_SIZE);
        let decoded = Frame::decode(&buf).unwrap();
        assert_eq!(decoded, frame);
    }

    #[test]
    fn rejects_bad_crc() {
        let mut buf = Vec::new();
        sample(b"hello").encode(&mut buf).unwrap();
        let last = buf.len() - 1;
        buf[last] ^= 0xFF;
        assert_eq!(Frame::decode(&buf), Err(FrameError::BadCrc));
    }

    #[test]
    fn rejects_wrong_length() {
        let mut buf = Vec::new();
        sample(b"hello").encode(&mut buf).unwrap();
        buf[18] = 99;
        assert!(matches!(
            Frame::decode(&buf),
            Err(FrameError::LengthMismatch { .. })
        ));
    }

    #[test]
    fn ack_bitmap_helpers() {
        let mut bitmap = [0u8; 2];
        bitmap_set(&mut bitmap, 0);
        bitmap_set(&mut bitmap, 9);
        assert!(bitmap_get(&bitmap, 0));
        assert!(!bitmap_get(&bitmap, 1));
        assert!(bitmap_get(&bitmap, 9));
        let payload = ack_payload(10, &bitmap);
        let (count, parsed) = parse_ack_payload(&payload).unwrap();
        assert_eq!(count, 10);
        assert_eq!(parsed, &bitmap);
    }
}
