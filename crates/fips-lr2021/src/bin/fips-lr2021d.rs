//! `fips-lr2021d` — stock-FIPS UDP fallback daemon for the LR2021 bridge.
//!
//! FIPS is configured with a normal UDP transport pointing at this daemon.
//! Every UDP datagram becomes a link-layer frame; every reassembled radio
//! frame becomes a UDP datagram back to FIPS.

use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use clap::Parser;
use crossbeam_channel::unbounded;
use fips_lr2021::bridge::{BridgeKind, RxPayload};
use fips_lr2021::{Band, Link, Lr2021Config, SerialBridge, send_packets};

#[derive(Debug, Parser)]
#[command(about = "FIPS over LR2021/FLRC UDP daemon")]
struct Args {
    #[arg(long, default_value = "/dev/ttyACM0")]
    serial: String,
    #[arg(long, default_value_t = 2_000_000)]
    baud: u32,
    #[arg(long, default_value = "us2400")]
    band: String,
    #[arg(long)]
    frequency_hz: Option<u32>,
    #[arg(long)]
    bitrate_kbps: Option<u16>,
    #[arg(long)]
    power_dbm: Option<i8>,
    #[arg(long, default_value_t = 1)]
    local_id: u32,
    #[arg(long, default_value_t = 2)]
    peer_id: u32,
    #[arg(long, default_value = "127.0.0.1:2122")]
    udp_bind: String,
    /// Fixed FIPS UDP address. If omitted, replies go to the last sender.
    #[arg(long)]
    fips_peer: Option<String>,
    #[arg(long)]
    relay: bool,
    #[arg(long)]
    mtu: Option<u16>,
    #[arg(long, default_value_t = 255)]
    radio_payload: u16,
}

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

fn main() {
    let args = Args::parse();
    let band = Band::parse(&args.band).unwrap_or_else(|| {
        eprintln!("unknown band '{}', using us2400", args.band);
        Band::Us2400
    });
    let cfg = Lr2021Config {
        port: Some(args.serial.clone()),
        baud: Some(args.baud),
        band: Some(band),
        frequency_hz: args.frequency_hz,
        bitrate_kbps: args.bitrate_kbps,
        tx_power_dbm: args.power_dbm,
        local_id: Some(args.local_id),
        relay: Some(args.relay),
        mtu: args.mtu,
        radio_payload: Some(args.radio_payload),
        ..Lr2021Config::default()
    };
    let bridge_cfg = cfg.bridge_config().unwrap_or_else(|e| fatal(&e));
    let link_cfg = cfg.link_config().unwrap_or_else(|e| fatal(&e));

    eprintln!(
        "opening {} at {} baud, band {} ({} Hz, FLRC {} kbps, {} dBm, payload {}B)",
        bridge_cfg.port,
        bridge_cfg.baud,
        bridge_cfg.profile.band.as_str(),
        bridge_cfg.profile.frequency_hz,
        bridge_cfg.profile.bitrate_kbps,
        bridge_cfg.profile.tx_power_dbm,
        bridge_cfg.profile.radio_payload
    );

    let (reader, _) = fips_lr2021::open_device(&bridge_cfg, link_cfg.clone())
        .unwrap_or_else(|e| fatal(&e.to_string()));
    let mut writer = reader
        .try_clone()
        .unwrap_or_else(|e| fatal(&format!("cannot clone serial port: {e}")));
    eprintln!(
        "bridge ready; local id {:#010x}, peer id {:#010x}",
        args.local_id, args.peer_id
    );

    let link = Arc::new(Mutex::new(
        Link::new(link_cfg.clone()).unwrap_or_else(|e| fatal(&e.to_string())),
    ));
    let stop = Arc::new(AtomicBool::new(false));
    let socket =
        Arc::new(UdpSocket::bind(&args.udp_bind).unwrap_or_else(|e| fatal(&e.to_string())));
    socket
        .set_read_timeout(Some(Duration::from_millis(200)))
        .unwrap_or_else(|e| fatal(&e.to_string()));
    let fixed_peer: Option<SocketAddr> = args
        .fips_peer
        .as_deref()
        .and_then(|s: &str| s.parse::<SocketAddr>().ok());
    let last_peer: Arc<Mutex<Option<SocketAddr>>> = Arc::new(Mutex::new(fixed_peer));

    let (tx_out, rx_out) = unbounded::<Vec<Vec<u8>>>();
    let (tx_udp, rx_udp) = unbounded::<Vec<u8>>();

    // Single serial writer: all radio packets and ACKs go through here.
    let writer_stop = Arc::clone(&stop);
    thread::spawn(move || {
        while !writer_stop.load(Ordering::Relaxed) {
            match rx_out.recv_timeout(Duration::from_millis(200)) {
                Ok(packets) => {
                    if let Err(e) = send_packets(&mut writer, &packets) {
                        eprintln!("serial write error: {e}");
                    }
                }
                Err(_) => {}
            }
        }
    });

    // Serial reader: RX packets -> link -> ACKs/delivered.
    let reader_link = Arc::clone(&link);
    let reader_stop = Arc::clone(&stop);
    let reader_tx_out = tx_out.clone();
    let reader_tx_udp = tx_udp.clone();
    let reader_socket = Arc::clone(&socket);
    let reader_peer = Arc::clone(&last_peer);
    thread::spawn(move || {
        let mut bridge: SerialBridge = reader;
        while !reader_stop.load(Ordering::Relaxed) {
            let frames = match bridge.read_frames(50) {
                Ok(frames) => frames,
                Err(e) => {
                    eprintln!("serial read error: {e}");
                    thread::sleep(Duration::from_millis(100));
                    continue;
                }
            };
            for frame in frames {
                match frame.kind {
                    BridgeKind::Rx => {
                        let Some(rx) = RxPayload::decode(&frame.payload) else {
                            continue;
                        };
                        let mut link = reader_link.lock().unwrap();
                        match link.process(&rx.data, now_ms()) {
                            Ok(outcome) => {
                                if !outcome.outbound.is_empty() {
                                    let _ = reader_tx_out.send(outcome.outbound);
                                }
                                for (_, data) in outcome.delivered {
                                    let peer = *reader_peer.lock().unwrap();
                                    if let Some(peer) = peer {
                                        if let Err(e) = reader_socket.send_to(&data, peer) {
                                            eprintln!("udp send error: {e}");
                                        }
                                    } else {
                                        let _ = reader_tx_udp.send(data);
                                    }
                                }
                            }
                            Err(e) => eprintln!("link rx drop: {e}"),
                        }
                    }
                    BridgeKind::Evt => {
                        eprintln!("bridge event: {}", String::from_utf8_lossy(&frame.payload))
                    }
                    BridgeKind::Stat => {
                        if let Some(stat) = fips_lr2021::bridge::StatPayload::decode(&frame.payload)
                        {
                            eprintln!(
                                "bridge stats tx={} rx={} crc={} drops={} timeouts={} resets={}",
                                stat.tx_packets,
                                stat.rx_packets,
                                stat.crc_errors,
                                stat.drops,
                                stat.tx_timeouts,
                                stat.resets
                            );
                        }
                    }
                    _ => {}
                }
            }
        }
    });

    // Retransmit timer.
    let timer_link = Arc::clone(&link);
    let timer_stop = Arc::clone(&stop);
    let timer_tx = tx_out.clone();
    thread::spawn(move || {
        while !timer_stop.load(Ordering::Relaxed) {
            thread::sleep(Duration::from_millis(10));
            let packets = timer_link.lock().unwrap().poll(now_ms());
            if !packets.is_empty() {
                let _ = timer_tx.send(packets);
            }
        }
    });

    // UDP side: FIPS datagrams become link frames.
    let udp_link = Arc::clone(&link);
    let udp_stop = Arc::clone(&stop);
    let udp_tx = tx_out.clone();
    let udp_peer = Arc::clone(&last_peer);
    thread::spawn(move || {
        let mut buf = [0u8; 2048];
        while !udp_stop.load(Ordering::Relaxed) {
            match socket.recv_from(&mut buf) {
                Ok((n, src)) => {
                    let mut peer = udp_peer.lock().unwrap();
                    if peer.is_none() {
                        *peer = Some(src);
                    }
                    drop(peer);
                    if n > link_cfg.mtu as usize {
                        eprintln!("dropping oversized UDP datagram: {n} bytes");
                        continue;
                    }
                    let packets =
                        match udp_link
                            .lock()
                            .unwrap()
                            .submit(&buf[..n], args.peer_id, now_ms())
                        {
                            Ok(packets) => packets,
                            Err(e) => {
                                eprintln!("link tx drop: {e}");
                                continue;
                            }
                        };
                    let _ = udp_tx.send(packets);
                }
                Err(e)
                    if e.kind() == std::io::ErrorKind::WouldBlock
                        || e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(e) => {
                    eprintln!("udp recv error: {e}");
                    thread::sleep(Duration::from_millis(100));
                }
            }
            while let Ok(data) = rx_udp.try_recv() {
                let peer = *udp_peer.lock().unwrap();
                if let Some(peer) = peer {
                    if let Err(e) = socket.send_to(&data, peer) {
                        eprintln!("udp send error: {e}");
                    }
                }
            }
        }
    });

    eprintln!(
        "UDP listening on {}; configure FIPS transports.udp to point here",
        args.udp_bind
    );
    while !stop.load(Ordering::Relaxed) {
        thread::sleep(Duration::from_millis(500));
    }
}

fn fatal(message: &str) -> ! {
    eprintln!("fatal: {message}");
    std::process::exit(1);
}
