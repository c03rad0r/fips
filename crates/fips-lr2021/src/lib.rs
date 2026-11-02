//! LR2021/FLRC link layer and USB-CDC bridge protocol for FIPS.
//!
//! The radio bridge is deliberately dumb: it moves raw FLRC packets between a
//! USB CDC port and the LR2021. This crate owns the hard parts — framing,
//! fragmentation, reassembly, ACK/NACK retransmission, deduplication and the
//! serial bridge protocol — so they can be tested without hardware.

pub mod bridge;
pub mod config;
pub mod crc;
pub mod device;
pub mod frame;
pub mod link;

pub use bridge::{BridgeDecoder, BridgeFrame, BridgeKind, SerialBridge};
pub use config::{Band, BridgeConfig, LinkConfig, Lr2021Config, RadioProfile};
pub use device::{handshake, open_device, poll_rx, send_packets};
pub use frame::{Frame, FrameKind};
pub use link::{Link, LinkError, LinkStats, RxOutcome};

/// Version of the on-air link protocol.
pub const LINK_PROTOCOL_VERSION: u8 = 1;

/// Version of the host↔bridge protocol.
pub const BRIDGE_PROTOCOL_VERSION: u8 = 1;
