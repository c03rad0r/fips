//! Fragmentation, reassembly, ACK/NACK retransmission and deduplication.
//!
//! The link is deliberately sans-IO: callers submit frames, feed received
//! radio packets, and poll for retransmits. That makes every path testable
//! without a radio or a serial port.

use std::collections::{BTreeMap, HashMap};

use thiserror::Error;

use crate::config::LinkConfig;
use crate::frame::{
    CRC_SIZE, Frame, FrameError, FrameKind, HEADER_SIZE, MAX_FRAGMENTS, ack_payload, bitmap_get,
    bitmap_len, bitmap_set, parse_ack_payload,
};

pub const BROADCAST_ID: u32 = 0xFFFF_FFFF;

#[derive(Debug, Error)]
pub enum LinkError {
    #[error("frame too large: {size} > mtu {mtu}")]
    FrameTooLarge { size: usize, mtu: usize },
    #[error("too many fragments: {0}")]
    TooManyFragments(usize),
    #[error("pending queue is full")]
    QueueFull,
    #[error(transparent)]
    Frame(#[from] FrameError),
    #[error("invalid link configuration: {0}")]
    Config(String),
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct LinkStats {
    pub frames_tx: u64,
    pub frames_rx: u64,
    pub fragments_tx: u64,
    pub fragments_rx: u64,
    pub retransmits: u64,
    pub dropped_tx: u64,
    pub dropped_rx: u64,
    pub duplicates: u64,
    pub crc_errors: u64,
    pub acks_tx: u64,
    pub acks_rx: u64,
}

#[derive(Debug, Default, Clone)]
pub struct RxOutcome {
    /// Complete FIPS frames, with the source link id.
    pub delivered: Vec<(u32, Vec<u8>)>,
    /// Encoded packets that must be written to the bridge.
    pub outbound: Vec<Vec<u8>>,
}

struct Pending {
    dst: u32,
    encoded: Vec<Vec<u8>>,
    acked: Vec<bool>,
    sent_at: u64,
    retries: u8,
}

struct Reassembly {
    frag_count: u8,
    bitmap: Vec<u8>,
    data: Vec<u8>,
    lens: Vec<usize>,
    last_update: u64,
}

pub struct Link {
    cfg: LinkConfig,
    epoch: u16,
    next_seq: u16,
    pending: BTreeMap<u16, Pending>,
    reassembly: HashMap<(u32, u16, u16), Reassembly>,
    completed: HashMap<(u32, u16, u16), u64>,
    stats: LinkStats,
}

impl Link {
    pub fn new(cfg: LinkConfig) -> Result<Self, LinkError> {
        if cfg.max_fragment_payload() < 32 {
            return Err(LinkError::Config(
                "radio_payload too small for link header".to_string(),
            ));
        }
        if cfg.mtu < 64 {
            return Err(LinkError::Config("mtu must be at least 64".to_string()));
        }
        let epoch = (std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_millis() as u16)
            .unwrap_or(1))
        .max(1);
        Ok(Self {
            cfg,
            epoch,
            next_seq: 1,
            pending: BTreeMap::new(),
            reassembly: HashMap::new(),
            completed: HashMap::new(),
            stats: LinkStats::default(),
        })
    }

    pub fn local_id(&self) -> u32 {
        self.cfg.local_id
    }

    pub fn epoch(&self) -> u16 {
        self.epoch
    }

    pub fn mtu(&self) -> u16 {
        self.cfg.mtu
    }

    pub fn stats(&self) -> &LinkStats {
        &self.stats
    }

    pub fn pending_len(&self) -> usize {
        self.pending.len()
    }

    pub fn reassembly_len(&self) -> usize {
        self.reassembly.len()
    }

    /// Fragment and encode a FIPS frame. The returned packets should be sent
    /// immediately; `poll` later returns retransmits.
    pub fn submit(&mut self, data: &[u8], dst: u32, now: u64) -> Result<Vec<Vec<u8>>, LinkError> {
        if data.len() > self.cfg.mtu as usize {
            return Err(LinkError::FrameTooLarge {
                size: data.len(),
                mtu: self.cfg.mtu as usize,
            });
        }
        if self.pending.len() >= 64 {
            return Err(LinkError::QueueFull);
        }
        let max_payload = self.cfg.max_fragment_payload();
        let frag_count = if data.is_empty() {
            1
        } else {
            data.len().div_ceil(max_payload)
        };
        if frag_count > MAX_FRAGMENTS as usize {
            return Err(LinkError::TooManyFragments(frag_count));
        }

        let mut seq = self.next_seq;
        while self.pending.contains_key(&seq) {
            seq = seq.wrapping_add(1);
        }
        self.next_seq = seq.wrapping_add(1);

        let mut encoded = Vec::with_capacity(frag_count);
        for idx in 0..frag_count {
            let start = idx * max_payload;
            let end = (start + max_payload).min(data.len());
            let payload = if start <= end { &data[start..end] } else { &[] };
            let frame = Frame::data(
                self.cfg.local_id,
                dst,
                self.epoch,
                seq,
                idx as u8,
                frag_count as u8,
                payload,
            );
            let mut buf = Vec::with_capacity(HEADER_SIZE + payload.len() + CRC_SIZE);
            frame.encode(&mut buf)?;
            encoded.push(buf);
        }

        self.stats.frames_tx += 1;
        self.stats.fragments_tx += frag_count as u64;
        self.pending.insert(
            seq,
            Pending {
                dst,
                acked: vec![false; frag_count],
                encoded: encoded.clone(),
                sent_at: now,
                retries: 0,
            },
        );
        Ok(encoded)
    }

    /// Process one received radio packet.
    pub fn process(&mut self, packet: &[u8], now: u64) -> Result<RxOutcome, LinkError> {
        let frame = match Frame::decode(packet) {
            Ok(frame) => frame,
            Err(err) => {
                if matches!(err, FrameError::BadCrc) {
                    self.stats.crc_errors += 1;
                }
                return Err(LinkError::Frame(err));
            }
        };

        if frame.src == self.cfg.local_id {
            return Ok(RxOutcome::default());
        }
        if frame.dst != self.cfg.local_id && frame.dst != BROADCAST_ID {
            return Ok(RxOutcome::default());
        }

        match frame.kind {
            FrameKind::Data => self.process_data(&frame, now),
            FrameKind::Ack | FrameKind::Nack => self.process_ack(&frame, now),
            FrameKind::Beacon => Ok(RxOutcome::default()),
        }
    }

    fn process_data(&mut self, frame: &Frame<'_>, now: u64) -> Result<RxOutcome, LinkError> {
        let frag_count = frame.frag_count;
        if frag_count == 0 || frame.frag_idx >= frag_count {
            self.stats.dropped_rx += 1;
            return Ok(RxOutcome::default());
        }
        let max_payload = self.cfg.max_fragment_payload();
        let max_frags = self.cfg.mtu.div_ceil(max_payload as u16) + 1;
        if frag_count as u16 > max_frags {
            self.stats.dropped_rx += 1;
            return Ok(RxOutcome::default());
        }
        if frame.payload.len() > max_payload {
            self.stats.dropped_rx += 1;
            return Ok(RxOutcome::default());
        }

        let key = (frame.src, frame.epoch, frame.seq);
        if self.completed.contains_key(&key) {
            self.stats.duplicates += 1;
            return self.ack_outcome(
                frame,
                frag_count,
                &vec![0xFF; bitmap_len(frag_count)],
                false,
            );
        }

        let entry = self.reassembly.entry(key).or_insert_with(|| Reassembly {
            frag_count,
            bitmap: vec![0u8; bitmap_len(frag_count)],
            data: vec![0u8; frag_count as usize * max_payload],
            lens: vec![0usize; frag_count as usize],
            last_update: now,
        });
        if entry.frag_count != frag_count {
            self.stats.dropped_rx += 1;
            self.reassembly.remove(&key);
            return Ok(RxOutcome::default());
        }
        let start = frame.frag_idx as usize * max_payload;
        let end = start + frame.payload.len();
        if end > entry.data.len() {
            self.stats.dropped_rx += 1;
            self.reassembly.remove(&key);
            return Ok(RxOutcome::default());
        }
        entry.data[start..end].copy_from_slice(frame.payload);
        entry.lens[frame.frag_idx as usize] = frame.payload.len();
        bitmap_set(&mut entry.bitmap, frame.frag_idx);
        entry.last_update = now;
        self.stats.fragments_rx += 1;

        let complete = (0..frag_count).all(|idx| bitmap_get(&entry.bitmap, idx));
        if !complete {
            // Do not answer every intermediate fragment: its ACK would contend
            // with the rest of the batch still being transmitted. Once the
            // tail arrives, one NACK reports the complete receive bitmap and
            // lets the sender repair exactly the missing fragments.
            if frame.frag_idx + 1 != frag_count {
                return Ok(RxOutcome::default());
            }
            let bitmap = entry.bitmap.clone();
            return self.ack_outcome(frame, frag_count, &bitmap, true);
        }

        let last_len = entry.lens[(frag_count - 1) as usize];
        let total_len = (frag_count as usize - 1) * max_payload + last_len;
        let data = entry.data[..total_len].to_vec();
        let bitmap = entry.bitmap.clone();
        self.reassembly.remove(&key);
        self.completed.insert(key, now);
        self.stats.frames_rx += 1;

        let mut outcome = self.ack_outcome(frame, frag_count, &bitmap, false)?;
        outcome.delivered.push((frame.src, data));
        Ok(outcome)
    }

    fn process_ack(&mut self, frame: &Frame<'_>, now: u64) -> Result<RxOutcome, LinkError> {
        let Some((frag_count, bitmap)) = parse_ack_payload(frame.payload) else {
            return Ok(RxOutcome::default());
        };
        if frame.epoch != self.epoch {
            return Ok(RxOutcome::default());
        }
        self.stats.acks_rx += 1;

        let Some(pending) = self.pending.get_mut(&frame.seq) else {
            return Ok(RxOutcome::default());
        };
        // Both directions start their sequence counters at one. Do not let an
        // identically numbered frame from another peer ACK this transmission.
        if frame.src != pending.dst {
            return Ok(RxOutcome::default());
        }
        if frag_count as usize != pending.encoded.len() {
            return Ok(RxOutcome::default());
        }
        for idx in 0..frag_count {
            if bitmap_get(bitmap, idx) {
                if let Some(slot) = pending.acked.get_mut(idx as usize) {
                    *slot = true;
                }
            }
        }
        if pending.acked.iter().all(|&acked| acked) {
            self.pending.remove(&frame.seq);
            return Ok(RxOutcome::default());
        }
        if frame.kind == FrameKind::Ack {
            // A partial cumulative ACK is progress information, not a request
            // to duplicate an in-flight batch. The ACK timer is the backstop;
            // only an explicit tail NACK triggers immediate selective repair.
            return Ok(RxOutcome::default());
        }

        // A tail NACK identifies exactly which fragments are still missing.
        let mut outbound = Vec::new();
        for (idx, packet) in pending.encoded.iter().enumerate() {
            if !pending.acked[idx] {
                outbound.push(packet.clone());
            }
        }
        pending.sent_at = now;
        self.stats.retransmits += outbound.len() as u64;
        Ok(RxOutcome {
            delivered: Vec::new(),
            outbound,
        })
    }

    fn ack_outcome(
        &mut self,
        frame: &Frame<'_>,
        frag_count: u8,
        bitmap: &[u8],
        nack: bool,
    ) -> Result<RxOutcome, LinkError> {
        let payload = ack_payload(frag_count, bitmap);
        let ack = Frame {
            kind: if nack {
                FrameKind::Nack
            } else {
                FrameKind::Ack
            },
            flags: 0,
            src: self.cfg.local_id,
            dst: frame.src,
            epoch: frame.epoch,
            seq: frame.seq,
            frag_idx: 0,
            frag_count: 0,
            payload: &payload,
        };
        let mut buf = Vec::with_capacity(HEADER_SIZE + payload.len() + CRC_SIZE);
        ack.encode(&mut buf)?;
        self.stats.acks_tx += 1;
        Ok(RxOutcome {
            delivered: Vec::new(),
            outbound: vec![buf],
        })
    }

    /// Return retransmits and expire stale reassemblies / dedup entries.
    pub fn poll(&mut self, now: u64) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        let mut drop_seqs = Vec::new();
        for (&seq, pending) in self.pending.iter_mut() {
            if now.saturating_sub(pending.sent_at) < self.cfg.ack_timeout_ms {
                continue;
            }
            if pending.retries >= self.cfg.max_retries {
                drop_seqs.push(seq);
                continue;
            }
            pending.retries += 1;
            pending.sent_at = now;
            for (idx, packet) in pending.encoded.iter().enumerate() {
                if !pending.acked[idx] {
                    out.push(packet.clone());
                }
            }
        }
        for seq in drop_seqs {
            if let Some(pending) = self.pending.remove(&seq) {
                self.stats.dropped_tx += pending.encoded.len() as u64;
            }
        }
        self.stats.retransmits += out.len() as u64;

        let stale: Vec<_> = self
            .reassembly
            .iter()
            .filter(|(_, entry)| {
                now.saturating_sub(entry.last_update) > self.cfg.reassembly_timeout_ms
            })
            .map(|(key, _)| *key)
            .collect();
        for key in stale {
            self.reassembly.remove(&key);
            self.stats.dropped_rx += 1;
        }

        self.completed
            .retain(|_, at| now.saturating_sub(*at) <= self.cfg.dedup_window_ms);
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg(id: u32) -> LinkConfig {
        LinkConfig {
            local_id: id,
            ..LinkConfig::default()
        }
    }

    fn pair() -> (Link, Link) {
        let mut a = Link::new(cfg(1)).unwrap();
        let mut b = Link::new(cfg(2)).unwrap();
        // Give the two ends different epochs so keys cannot collide in a test.
        a.epoch = 10;
        b.epoch = 20;
        (a, b)
    }

    #[test]
    fn roundtrip_one_frame() {
        let (mut a, mut b) = pair();
        let data = vec![0x5A; 500];
        let packets = a.submit(&data, 2, 0).unwrap();
        let mut delivered = Vec::new();
        let mut acks = Vec::new();
        for packet in &packets {
            let mut outcome = b.process(packet, 1).unwrap();
            delivered.append(&mut outcome.delivered);
            acks.append(&mut outcome.outbound);
        }
        assert_eq!(delivered.len(), 1);
        assert_eq!(delivered[0].1, data);
        assert_eq!(b.stats().frames_rx, 1);
        assert!(!acks.is_empty());
        for packet in &acks {
            a.process(packet, 2).unwrap();
        }
        assert_eq!(a.pending_len(), 0);
    }

    #[test]
    fn delivers_large_frame() {
        let (mut a, mut b) = pair();
        let data: Vec<u8> = (0..1400).map(|i| (i % 251) as u8).collect();
        let packets = a.submit(&data, 2, 0).unwrap();
        let mut delivered = Vec::new();
        for packet in &packets {
            delivered.extend(b.process(packet, 1).unwrap().delivered);
        }
        assert_eq!(delivered.len(), 1);
        assert_eq!(delivered[0].0, 1);
        assert_eq!(delivered[0].1, data);
    }

    #[test]
    fn rejects_ack_for_wrong_peer_or_epoch() {
        let (mut a, mut b) = pair();
        let packets = a.submit(b"authenticated ack", 2, 0).unwrap();
        let outcome = b.process(&packets[0], 1).unwrap();
        assert_eq!(outcome.outbound.len(), 1);
        let ack = Frame::decode(&outcome.outbound[0]).unwrap();

        let mut encoded = Vec::new();
        Frame { src: 3, ..ack }.encode(&mut encoded).unwrap();
        a.process(&encoded, 2).unwrap();
        assert_eq!(a.pending_len(), 1, "another peer must not ACK this frame");

        Frame {
            epoch: ack.epoch.wrapping_add(1),
            ..ack
        }
        .encode(&mut encoded)
        .unwrap();
        a.process(&encoded, 3).unwrap();
        assert_eq!(a.pending_len(), 1, "another epoch must not ACK this frame");

        a.process(&outcome.outbound[0], 4).unwrap();
        assert_eq!(a.pending_len(), 0, "the real peer and epoch still complete");
    }

    #[test]
    fn nacks_once_at_batch_tail_and_repairs_missing_fragment() {
        let (mut a, mut b) = pair();
        let data = vec![0x77; 700];
        let packets = a.submit(&data, 2, 0).unwrap();
        assert!(packets.len() >= 3);

        let mut tail_nack = None;
        for (idx, packet) in packets.iter().enumerate() {
            if idx == 1 {
                continue; // Lose one intermediate fragment.
            }
            let outcome = b.process(packet, 1).unwrap();
            assert!(outcome.delivered.is_empty());
            if idx + 1 < packets.len() {
                assert!(
                    outcome.outbound.is_empty(),
                    "intermediate fragments must not contend with the batch"
                );
            } else {
                tail_nack = Some(outcome.outbound);
            }
        }

        let nacks = tail_nack.expect("tail must report the incomplete bitmap");
        assert_eq!(nacks.len(), 1);
        assert_eq!(Frame::decode(&nacks[0]).unwrap().kind, FrameKind::Nack);

        let repair = a.process(&nacks[0], 2).unwrap().outbound;
        assert_eq!(repair.len(), 1, "only the missing fragment is repeated");
        assert_eq!(repair[0], packets[1]);

        let outcome = b.process(&repair[0], 3).unwrap();
        assert_eq!(outcome.delivered.len(), 1);
        assert_eq!(outcome.delivered[0].1, data);
        assert_eq!(outcome.outbound.len(), 1, "one final completion ACK");
        assert_eq!(
            Frame::decode(&outcome.outbound[0]).unwrap().kind,
            FrameKind::Ack
        );
    }

    #[test]
    fn retransmits_missing_fragment() {
        let (mut a, mut b) = pair();
        let data = vec![0x77; 700];
        let packets = a.submit(&data, 2, 0).unwrap();
        assert!(packets.len() >= 3);

        // Drop the second fragment.
        let mut delivered = Vec::new();
        for (idx, packet) in packets.iter().enumerate() {
            if idx == 1 {
                continue;
            }
            delivered.extend(b.process(packet, 1).unwrap().delivered);
        }
        assert!(delivered.is_empty());

        let retransmit = a.poll(41);
        assert!(!retransmit.is_empty());
        for packet in &retransmit {
            delivered.extend(b.process(packet, 42).unwrap().delivered);
        }
        assert_eq!(delivered.len(), 1);
        assert_eq!(delivered[0].1, data);
    }

    #[test]
    fn ignores_duplicate_frame() {
        let (mut a, mut b) = pair();
        let data = b"duplicate me".to_vec();
        let packets = a.submit(&data, 2, 0).unwrap();
        let mut delivered = Vec::new();
        for packet in &packets {
            delivered.extend(b.process(packet, 1).unwrap().delivered);
        }
        for packet in &packets {
            delivered.extend(b.process(packet, 2).unwrap().delivered);
        }
        assert_eq!(delivered.len(), 1);
        assert_eq!(b.stats().duplicates, packets.len() as u64);
    }

    #[test]
    fn rejects_oversized_frame() {
        let mut a = Link::new(cfg(1)).unwrap();
        let data = vec![0u8; 2000];
        assert!(matches!(
            a.submit(&data, 2, 0),
            Err(LinkError::FrameTooLarge { .. })
        ));
    }

    #[test]
    fn expires_incomplete_reassembly() {
        let (mut a, mut b) = pair();
        let data = vec![0u8; 700];
        let packets = a.submit(&data, 2, 0).unwrap();
        for packet in packets.iter().skip(1) {
            b.process(packet, 1).unwrap();
        }
        assert_eq!(b.reassembly_len(), 1);
        b.poll(1000);
        assert_eq!(b.reassembly_len(), 0);
        assert_eq!(b.stats().dropped_rx, 1);
    }

    #[test]
    fn out_of_order_fragments_reassemble() {
        let (mut a, mut b) = pair();
        let data = vec![0x42; 900];
        let packets = a.submit(&data, 2, 0).unwrap();
        let mut delivered = Vec::new();
        for packet in packets.iter().rev() {
            delivered.extend(b.process(packet, 1).unwrap().delivered);
        }
        assert_eq!(delivered.len(), 1);
        assert_eq!(delivered[0].1, data);
    }

    #[test]
    fn bad_crc_is_counted() {
        let (mut a, mut b) = pair();
        let packets = a.submit(b"hello", 2, 0).unwrap();
        let mut packet = packets[0].clone();
        let last = packet.len() - 1;
        packet[last] ^= 0xFF;
        assert!(b.process(&packet, 1).is_err());
        assert_eq!(b.stats().crc_errors, 1);
    }
}
