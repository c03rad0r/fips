//! Convenience wrapper for opening and configuring a bridge.

use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::bridge::{
    BridgeError, BridgeKind, CfgAckPayload, CfgPayload, HelloAckPayload, RxPayload, SerialBridge,
};
use crate::{BRIDGE_PROTOCOL_VERSION, BridgeConfig, Link, LinkConfig, RxOutcome};

/// Open a serial bridge, perform the HELLO/CFG handshake, and return a device
/// with a link layer attached.
pub fn open_device(
    bridge_cfg: &BridgeConfig,
    link_cfg: LinkConfig,
) -> Result<(SerialBridge, Link), BridgeError> {
    let mut bridge = SerialBridge::open(&bridge_cfg.port, bridge_cfg.baud)?;
    handshake(&mut bridge, bridge_cfg, 2_000)?;
    let link = Link::new(link_cfg).map_err(|e| BridgeError::Protocol(e.to_string()))?;
    Ok((bridge, link))
}

/// Perform HELLO and CFG against an already-open bridge.
pub fn handshake(
    bridge: &mut SerialBridge,
    cfg: &BridgeConfig,
    timeout_ms: u64,
) -> Result<(), BridgeError> {
    bridge.write_frame(BridgeKind::Hello, &[BRIDGE_PROTOCOL_VERSION])?;
    let hello = wait_for(bridge, BridgeKind::HelloAck, timeout_ms)?;
    let ack = HelloAckPayload::decode(&hello)
        .ok_or_else(|| BridgeError::Protocol("malformed HELLO_ACK".to_string()))?;
    if ack.protocol != BRIDGE_PROTOCOL_VERSION {
        return Err(BridgeError::Protocol(format!(
            "bridge protocol {} does not match host {}",
            ack.protocol, BRIDGE_PROTOCOL_VERSION
        )));
    }

    // Optional extension: older bridges ignore it. New bridges use the link ID
    // for a TX slot and the host Unix epoch for a common slot phase.
    let mut ext = Vec::with_capacity(12);
    ext.extend_from_slice(&cfg.local_id.to_le_bytes());
    let epoch_ms = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0);
    ext.extend_from_slice(&epoch_ms.to_le_bytes());
    bridge.write_frame(BridgeKind::CfgExt, &ext)?;

    let payload = CfgPayload {
        profile: cfg.profile,
        relay: cfg.relay,
    }
    .encode();
    bridge.write_frame(BridgeKind::Cfg, &payload)?;
    let cfg_ack = wait_for(bridge, BridgeKind::CfgAck, timeout_ms)?;
    let ack = CfgAckPayload::decode(&cfg_ack)
        .ok_or_else(|| BridgeError::Protocol("malformed CFG_ACK".to_string()))?;
    if ack.status != 0 {
        return Err(BridgeError::Protocol(format!(
            "bridge rejected configuration (status {})",
            ack.status
        )));
    }
    if ack.frequency_hz != cfg.profile.frequency_hz
        || ack.bitrate_kbps != cfg.profile.bitrate_kbps
        || ack.radio_payload != cfg.profile.radio_payload
    {
        return Err(BridgeError::Protocol(format!(
            "bridge applied {}/{}kbps/{}B, requested {}/{}kbps/{}B",
            ack.frequency_hz,
            ack.bitrate_kbps,
            ack.radio_payload,
            cfg.profile.frequency_hz,
            cfg.profile.bitrate_kbps,
            cfg.profile.radio_payload
        )));
    }
    Ok(())
}

fn wait_for(
    bridge: &mut SerialBridge,
    wanted: BridgeKind,
    timeout_ms: u64,
) -> Result<Vec<u8>, BridgeError> {
    let deadline = Instant::now() + Duration::from_millis(timeout_ms);
    while Instant::now() < deadline {
        for frame in bridge.read_frames(50)? {
            if frame.kind == wanted {
                return Ok(frame.payload);
            }
            if frame.kind == BridgeKind::Evt {
                eprintln!(
                    "lr2021 bridge event: {}",
                    String::from_utf8_lossy(&frame.payload)
                );
            }
        }
    }
    Err(BridgeError::Protocol(format!(
        "timed out waiting for {wanted:?}"
    )))
}

/// Write a batch of raw radio packets as `Tx` frames.
pub fn send_packets(bridge: &mut SerialBridge, packets: &[Vec<u8>]) -> Result<(), BridgeError> {
    send_packets_paced(bridge, packets, Duration::ZERO)
}

/// Write a fragment batch with an optional gap between radio packets. This is
/// useful for diagnosing receive-side exhaustion on half-duplex bridges: the
/// default remains back-to-back, while a test can give each receiver time to
/// finish RX before the next packet arrives without changing its firmware.
pub fn send_packets_paced(
    bridge: &mut SerialBridge,
    packets: &[Vec<u8>],
    gap: Duration,
) -> Result<(), BridgeError> {
    for (index, packet) in packets.iter().enumerate() {
        if index > 0 && !gap.is_zero() {
            std::thread::sleep(gap);
        }
        bridge.write_frame(BridgeKind::Tx, packet)?;
    }
    Ok(())
}

/// Read and process pending `Rx` frames. ACKs are written back immediately.
pub fn poll_rx(
    bridge: &mut SerialBridge,
    link: &mut Link,
    timeout_ms: u64,
    now: u64,
) -> Result<RxOutcome, BridgeError> {
    let mut result = RxOutcome::default();
    for frame in bridge.read_frames(timeout_ms)? {
        match frame.kind {
            BridgeKind::Rx => {
                if let Some(rx) = RxPayload::decode(&frame.payload) {
                    match link.process(&rx.data, now) {
                        Ok(mut outcome) => {
                            result.delivered.append(&mut outcome.delivered);
                            result.outbound.append(&mut outcome.outbound);
                        }
                        Err(err) => {
                            // Corrupt or malformed packets are expected on a
                            // radio link; the link layer already counted them.
                            eprintln!("lr2021 rx drop: {err}");
                        }
                    }
                }
            }
            BridgeKind::Stat => {
                if let Some(stat) = crate::bridge::StatPayload::decode(&frame.payload) {
                    eprintln!(
                        "lr2021 bridge stats: tx={} rx={} crc={} drops={} timeouts={} resets={}",
                        stat.tx_packets,
                        stat.rx_packets,
                        stat.crc_errors,
                        stat.drops,
                        stat.tx_timeouts,
                        stat.resets
                    );
                }
            }
            BridgeKind::Evt => {
                eprintln!(
                    "lr2021 bridge event: {}",
                    String::from_utf8_lossy(&frame.payload)
                );
            }
            _ => {}
        }
    }
    if !result.outbound.is_empty() {
        send_packets(bridge, &result.outbound)?;
        result.outbound.clear();
    }
    Ok(result)
}
