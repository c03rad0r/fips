//! LR2021/FLRC transport for FIPS.
//!
//! The host owns the link layer (fragmentation, reassembly, ACK/NACK and
//! deduplication); the RP2040 bridge is a raw packet pipe. See
//! `crates/fips-lr2021` for the protocol and tests.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{SystemTime, UNIX_EPOCH};

use crossbeam_channel::{Sender, unbounded};

use crate::config::Lr2021Config;
use crate::transport::{
    DiscoveredPeer, PacketTx, ReceivedPacket, Transport, TransportAddr, TransportError,
    TransportId, TransportState, TransportType,
};
use fips_lr2021::bridge::{BridgeKind, RxPayload};
use fips_lr2021::{Link, send_packets};

/// Native FIPS transport over an LR2021 USB bridge.
pub struct Lr2021Transport {
    transport_id: TransportId,
    name: Option<String>,
    config: Lr2021Config,
    state: TransportState,
    packet_tx: PacketTx,
    mtu: u16,
    link: Arc<Mutex<Option<Link>>>,
    tx_packets: Sender<Vec<Vec<u8>>>,
    stop: Arc<AtomicBool>,
    threads: Vec<JoinHandle<()>>,
}

impl Lr2021Transport {
    pub fn new(
        transport_id: TransportId,
        name: Option<String>,
        config: Lr2021Config,
        packet_tx: PacketTx,
    ) -> Self {
        let mtu = config.mtu.unwrap_or(1400);
        let (tx_packets, _rx_packets) = unbounded();
        Self {
            transport_id,
            name,
            config,
            state: TransportState::Configured,
            packet_tx,
            mtu,
            link: Arc::new(Mutex::new(None)),
            tx_packets,
            stop: Arc::new(AtomicBool::new(false)),
            threads: Vec::new(),
        }
    }

    pub fn name(&self) -> Option<&str> {
        self.name.as_deref()
    }

    pub fn set_state(&mut self, state: TransportState) {
        self.state = state;
    }

    fn start_inner(&mut self) -> Result<(), TransportError> {
        let bridge_cfg = self
            .config
            .bridge_config()
            .map_err(TransportError::StartFailed)?;
        let link_cfg = self
            .config
            .link_config()
            .map_err(TransportError::StartFailed)?;

        let (reader, link) = fips_lr2021::open_device(&bridge_cfg, link_cfg)
            .map_err(|e| TransportError::StartFailed(e.to_string()))?;
        let mut writer = reader
            .try_clone()
            .map_err(|e| TransportError::StartFailed(format!("serial clone: {e}")))?;

        *self.link.lock().unwrap() = Some(link);
        let link = Arc::clone(&self.link);
        let stop = Arc::clone(&self.stop);
        let packet_tx = self.packet_tx.clone();
        let transport_id = self.transport_id;
        let (tx_packets, rx_packets) = unbounded::<Vec<Vec<u8>>>();
        self.tx_packets = tx_packets.clone();

        // Writer: every radio packet and ACK is serialized here.
        let writer_stop = Arc::clone(&stop);
        self.threads.push(thread::spawn(move || {
            while !writer_stop.load(Ordering::Relaxed) {
                match rx_packets.recv_timeout(std::time::Duration::from_millis(200)) {
                    Ok(packets) => {
                        if let Err(e) = send_packets(&mut writer, &packets) {
                            tracing::warn!(error = %e, "lr2021 serial write failed");
                        }
                    }
                    Err(_) => {}
                }
            }
        }));

        // Reader: radio packets -> link -> ACKs / FIPS packets.
        let reader_link = Arc::clone(&link);
        let reader_stop = Arc::clone(&stop);
        let reader_tx = tx_packets.clone();
        let reader_packet_tx = packet_tx.clone();
        self.threads.push(thread::spawn(move || {
            let mut bridge = reader;
            while !reader_stop.load(Ordering::Relaxed) {
                let frames = match bridge.read_frames(50) {
                    Ok(frames) => frames,
                    Err(e) => {
                        tracing::warn!(error = %e, "lr2021 serial read failed");
                        thread::sleep(std::time::Duration::from_millis(100));
                        continue;
                    }
                };
                for frame in frames {
                    if frame.kind != BridgeKind::Rx {
                        continue;
                    }
                    let Some(rx) = RxPayload::decode(&frame.payload) else {
                        continue;
                    };
                    let mut guard = reader_link.lock().unwrap();
                    let Some(link) = guard.as_mut() else {
                        continue;
                    };
                    match link.process(&rx.data, now_ms()) {
                        Ok(outcome) => {
                            if !outcome.outbound.is_empty() {
                                let _ = reader_tx.send(outcome.outbound);
                            }
                            for (src, data) in outcome.delivered {
                                let addr = TransportAddr::from_bytes(&src.to_le_bytes());
                                if reader_packet_tx
                                    .try_send(ReceivedPacket::new(transport_id, addr, data))
                                    .is_err()
                                {
                                    tracing::debug!("lr2021 packet channel full; dropping");
                                }
                            }
                        }
                        Err(e) => tracing::debug!(error = %e, "lr2021 rx drop"),
                    }
                }
            }
        }));

        // Retransmit / expiry timer.
        let timer_link = Arc::clone(&link);
        let timer_stop = Arc::clone(&stop);
        let timer_tx = tx_packets;
        self.threads.push(thread::spawn(move || {
            while !timer_stop.load(Ordering::Relaxed) {
                thread::sleep(std::time::Duration::from_millis(10));
                let mut guard = timer_link.lock().unwrap();
                if let Some(link) = guard.as_mut() {
                    let packets = link.poll(now_ms());
                    if !packets.is_empty() {
                        let _ = timer_tx.send(packets);
                    }
                }
            }
        }));

        self.state = TransportState::Up;
        Ok(())
    }
}

impl Transport for Lr2021Transport {
    fn transport_id(&self) -> TransportId {
        self.transport_id
    }

    fn transport_type(&self) -> &TransportType {
        &TransportType::LR2021
    }

    fn state(&self) -> TransportState {
        self.state
    }

    fn mtu(&self) -> u16 {
        self.mtu
    }

    fn start(&mut self) -> Result<(), TransportError> {
        if self.state == TransportState::Up {
            return Err(TransportError::AlreadyStarted);
        }
        self.state = TransportState::Starting;
        match self.start_inner() {
            Ok(()) => Ok(()),
            Err(err) => {
                self.state = TransportState::Failed;
                Err(err)
            }
        }
    }

    fn stop(&mut self) -> Result<(), TransportError> {
        self.stop.store(true, Ordering::Relaxed);
        for handle in self.threads.drain(..) {
            let _ = handle.join();
        }
        *self.link.lock().unwrap() = None;
        self.state = TransportState::Down;
        Ok(())
    }

    fn send(&self, addr: &TransportAddr, data: &[u8]) -> Result<(), TransportError> {
        if self.state != TransportState::Up {
            return Err(TransportError::NotStarted);
        }
        let dst = addr_to_id(addr);
        let now = now_ms();
        let mut guard = self.link.lock().unwrap();
        let link = guard.as_mut().ok_or(TransportError::NotStarted)?;
        if data.len() > link.mtu() as usize {
            return Err(TransportError::MtuExceeded {
                packet_size: data.len(),
                mtu: link.mtu(),
            });
        }
        let packets = link
            .submit(data, dst, now)
            .map_err(|e| TransportError::SendFailed(e.to_string()))?;
        self.tx_packets
            .send(packets)
            .map_err(|_| TransportError::SendFailed("writer stopped".to_string()))?;
        Ok(())
    }

    fn discover(&self) -> Result<Vec<DiscoveredPeer>, TransportError> {
        Ok(Vec::new())
    }

    fn auto_connect(&self) -> bool {
        self.config.auto_connect()
    }

    fn accept_connections(&self) -> bool {
        true
    }
}

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

fn addr_to_id(addr: &TransportAddr) -> u32 {
    let bytes = addr.as_bytes();
    if bytes.is_empty() {
        return fips_lr2021::link::BROADCAST_ID;
    }
    let mut buf = [0u8; 4];
    let n = bytes.len().min(4);
    buf[..n].copy_from_slice(&bytes[..n]);
    u32::from_le_bytes(buf)
}
