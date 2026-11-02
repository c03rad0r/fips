//! `fips-lr2021ctl` — configure and exercise an LR2021 bridge.

use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use clap::{Parser, Subcommand};
use fips_lr2021::bridge::{BridgeKind, HelloAckPayload, RxPayload};
use fips_lr2021::{Band, Lr2021Config, SerialBridge, send_packets};

#[derive(Debug, Parser)]
#[command(about = "Configure and test an LR2021/FLRC bridge")]
struct Args {
    #[arg(long, default_value = "/dev/ttyACM0")]
    serial: String,
    #[arg(long, default_value_t = 2_000_000)]
    baud: u32,
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Send HELLO and print the bridge identity.
    Hello,
    /// Configure the radio and print the applied values.
    Cfg {
        #[arg(long, default_value = "us2400")]
        band: String,
        #[arg(long)]
        frequency_hz: Option<u32>,
        #[arg(long)]
        bitrate_kbps: Option<u16>,
        #[arg(long)]
        power_dbm: Option<i8>,
        #[arg(long)]
        radio_payload: Option<u16>,
        #[arg(long)]
        relay: bool,
    },
    /// Send PING and print bridge statistics.
    Status,
    /// Print received radio packets.
    Monitor {
        #[arg(long, default_value = "us2400")]
        band: String,
        #[arg(long, default_value_t = 10)]
        seconds: u64,
    },
    /// Send a small PER burst and report delivery.
    Per {
        #[arg(long, default_value = "us2400")]
        band: String,
        #[arg(long, default_value_t = 100)]
        count: u32,
        #[arg(long, default_value_t = 64)]
        size: usize,
        #[arg(long, default_value_t = 2)]
        peer_id: u32,
        #[arg(long, default_value_t = 1000)]
        timeout_ms: u64,
    },
    /// Run the link-layer responder: ACK received frames and print them.
    Echo {
        #[arg(long, default_value = "us2400")]
        band: String,
        #[arg(long, default_value_t = 30)]
        seconds: u64,
    },
}

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

fn main() {
    let args = Args::parse();
    if let Err(err) = run(args) {
        eprintln!("error: {err}");
        std::process::exit(1);
    }
}

fn run(args: Args) -> Result<(), String> {
    match args.command {
        Command::Hello => hello(&args.serial, args.baud),
        Command::Cfg {
            band,
            frequency_hz,
            bitrate_kbps,
            power_dbm,
            radio_payload,
            relay,
        } => {
            let band = Band::parse(&band).ok_or_else(|| format!("unknown band '{band}'"))?;
            let cfg = Lr2021Config {
                port: Some(args.serial),
                baud: Some(args.baud),
                band: Some(band),
                frequency_hz,
                bitrate_kbps,
                tx_power_dbm: power_dbm,
                radio_payload,
                relay: Some(relay),
                ..Lr2021Config::default()
            };
            let bridge_cfg = cfg.bridge_config()?;
            let link_cfg = cfg.link_config()?;
            let _ = fips_lr2021::open_device(&bridge_cfg, link_cfg).map_err(|e| e.to_string())?;
            println!(
                "configured band={} freq={} Hz flrc={} kbps power={} dBm payload={}B relay={}",
                bridge_cfg.profile.band.as_str(),
                bridge_cfg.profile.frequency_hz,
                bridge_cfg.profile.bitrate_kbps,
                bridge_cfg.profile.tx_power_dbm,
                bridge_cfg.profile.radio_payload,
                bridge_cfg.relay
            );
            Ok(())
        }
        Command::Status => status(&args.serial, args.baud),
        Command::Monitor { band, seconds } => {
            let band = Band::parse(&band).ok_or_else(|| format!("unknown band '{band}'"))?;
            monitor(&args.serial, args.baud, band, seconds)
        }
        Command::Per {
            band,
            count,
            size,
            peer_id,
            timeout_ms,
        } => {
            let band = Band::parse(&band).ok_or_else(|| format!("unknown band '{band}'"))?;
            per(
                &args.serial,
                args.baud,
                band,
                count,
                size,
                peer_id,
                timeout_ms,
            )
        }
        Command::Echo { band, seconds } => {
            let band = Band::parse(&band).ok_or_else(|| format!("unknown band '{band}'"))?;
            echo(&args.serial, args.baud, band, seconds)
        }
    }
}

fn hello(serial: &str, baud: u32) -> Result<(), String> {
    let mut bridge = SerialBridge::open(serial, baud).map_err(|e| e.to_string())?;
    bridge
        .write_frame(BridgeKind::Hello, &[fips_lr2021::BRIDGE_PROTOCOL_VERSION])
        .map_err(|e| e.to_string())?;
    let deadline = Instant::now() + Duration::from_secs(2);
    while Instant::now() < deadline {
        for frame in bridge.read_frames(100).map_err(|e| e.to_string())? {
            if frame.kind == BridgeKind::HelloAck {
                let ack = HelloAckPayload::decode(&frame.payload)
                    .ok_or_else(|| "malformed HELLO_ACK".to_string())?;
                println!(
                    "protocol={} firmware={}.{}.{} chip=0x{:04x} capabilities=0x{:04x}",
                    ack.protocol,
                    ack.firmware[0],
                    ack.firmware[1],
                    ack.firmware[2],
                    ack.chip_id,
                    ack.capabilities
                );
                return Ok(());
            }
        }
    }
    Err("no HELLO_ACK (is the bridge firmware running?)".to_string())
}

fn status(serial: &str, baud: u32) -> Result<(), String> {
    let mut bridge = SerialBridge::open(serial, baud).map_err(|e| e.to_string())?;
    bridge
        .write_frame(BridgeKind::Ping, &[])
        .map_err(|e| e.to_string())?;
    let deadline = Instant::now() + Duration::from_secs(1);
    let mut saw = false;
    while Instant::now() < deadline {
        for frame in bridge.read_frames(100).map_err(|e| e.to_string())? {
            match frame.kind {
                BridgeKind::Pong => {
                    println!("pong");
                    saw = true;
                }
                BridgeKind::Stat => {
                    if let Some(stat) = fips_lr2021::bridge::StatPayload::decode(&frame.payload) {
                        println!(
                            "tx={} rx={} crc={} drops={} timeouts={} resets={}",
                            stat.tx_packets,
                            stat.rx_packets,
                            stat.crc_errors,
                            stat.drops,
                            stat.tx_timeouts,
                            stat.resets
                        );
                        saw = true;
                    }
                }
                BridgeKind::Evt => {
                    println!("event: {}", String::from_utf8_lossy(&frame.payload));
                    saw = true;
                }
                _ => {}
            }
        }
    }
    if saw {
        Ok(())
    } else {
        Err("no status response".to_string())
    }
}

fn monitor(serial: &str, baud: u32, band: Band, seconds: u64) -> Result<(), String> {
    let (mut bridge, _) = fips_lr2021::open_device(
        &fips_lr2021::BridgeConfig {
            port: serial.to_string(),
            baud,
            profile: band.default_profile(),
            relay: false,
        },
        fips_lr2021::LinkConfig::default(),
    )
    .map_err(|e| e.to_string())?;
    let deadline = Instant::now() + Duration::from_secs(seconds);
    let mut packets = 0u64;
    while Instant::now() < deadline {
        for frame in bridge.read_frames(200).map_err(|e| e.to_string())? {
            if frame.kind == BridgeKind::Rx {
                if let Some(rx) = RxPayload::decode(&frame.payload) {
                    packets += 1;
                    println!(
                        "rx len={} rssi={} snr={} crc={} data={}",
                        rx.data.len(),
                        rx.rssi_dbm,
                        rx.snr_db,
                        rx.crc_ok,
                        hex(&rx.data)
                    );
                }
            }
        }
    }
    println!("monitor: {packets} packets");
    Ok(())
}

fn per(
    serial: &str,
    baud: u32,
    band: Band,
    count: u32,
    size: usize,
    peer_id: u32,
    timeout_ms: u64,
) -> Result<(), String> {
    let cfg = Lr2021Config {
        port: Some(serial.to_string()),
        baud: Some(baud),
        band: Some(band),
        ..Lr2021Config::default()
    };
    let bridge_cfg = cfg.bridge_config()?;
    let link_cfg = cfg.link_config()?;
    let (mut bridge, mut link) =
        fips_lr2021::open_device(&bridge_cfg, link_cfg).map_err(|e| e.to_string())?;

    let mut delivered = 0u32;
    let mut rssi_sum = 0i64;
    let mut rssi_count = 0u32;
    let start = Instant::now();
    for seq in 0..count {
        let mut payload = vec![0u8; size.max(8)];
        payload[0..4].copy_from_slice(&seq.to_be_bytes());
        for (i, byte) in payload.iter_mut().enumerate().skip(4) {
            *byte = (i as u8).wrapping_mul(31).wrapping_add(seq as u8);
        }
        let packets = link
            .submit(&payload, peer_id, now_ms())
            .map_err(|e| e.to_string())?;
        send_packets(&mut bridge, &packets).map_err(|e| e.to_string())?;

        let before = delivered;
        let deadline = Instant::now() + Duration::from_millis(timeout_ms);
        while Instant::now() < deadline {
            for frame in bridge.read_frames(20).map_err(|e| e.to_string())? {
                match frame.kind {
                    BridgeKind::Rx => {
                        if let Some(rx) = RxPayload::decode(&frame.payload) {
                            if rx.crc_ok {
                                rssi_sum += rx.rssi_dbm as i64;
                                rssi_count += 1;
                            }
                            if let Ok(outcome) = link.process(&rx.data, now_ms()) {
                                if !outcome.outbound.is_empty() {
                                    send_packets(&mut bridge, &outcome.outbound)
                                        .map_err(|e| e.to_string())?;
                                }
                                delivered += outcome.delivered.len() as u32;
                            }
                        }
                    }
                    _ => {}
                }
            }
            let retransmit = link.poll(now_ms());
            if !retransmit.is_empty() {
                send_packets(&mut bridge, &retransmit).map_err(|e| e.to_string())?;
            }
            if delivered > before || delivered >= count {
                break;
            }
        }
    }
    let elapsed = start.elapsed().as_secs_f64();
    let per = if count == 0 {
        0.0
    } else {
        100.0 * (count.saturating_sub(delivered)) as f64 / count as f64
    };
    let avg_rssi = if rssi_count == 0 {
        f64::NAN
    } else {
        rssi_sum as f64 / rssi_count as f64
    };
    println!(
        "per: sent={count} delivered={delivered} loss={per:.1}% elapsed={elapsed:.2}s avg_rssi={avg_rssi:.1} dBm"
    );
    Ok(())
}

fn echo(serial: &str, baud: u32, band: Band, seconds: u64) -> Result<(), String> {
    let cfg = Lr2021Config {
        port: Some(serial.to_string()),
        baud: Some(baud),
        band: Some(band),
        local_id: Some(2),
        ..Lr2021Config::default()
    };
    let bridge_cfg = cfg.bridge_config()?;
    let link_cfg = cfg.link_config()?;
    let (mut bridge, mut link) =
        fips_lr2021::open_device(&bridge_cfg, link_cfg).map_err(|e| e.to_string())?;
    let deadline = Instant::now() + Duration::from_secs(seconds);
    let mut frames = 0u64;
    while Instant::now() < deadline {
        for frame in bridge.read_frames(100).map_err(|e| e.to_string())? {
            println!(
                "echo frame kind={:?} len={}",
                frame.kind,
                frame.payload.len()
            );
            if frame.kind != BridgeKind::Rx {
                continue;
            }
            let Some(rx) = RxPayload::decode(&frame.payload) else {
                println!("echo: bad RX payload");
                continue;
            };
            match link.process(&rx.data, now_ms()) {
                Ok(outcome) => {
                    if !outcome.outbound.is_empty() {
                        send_packets(&mut bridge, &outcome.outbound).map_err(|e| e.to_string())?;
                    }
                    for (src, data) in outcome.delivered {
                        frames += 1;
                        println!("echo rx len={} data={}", data.len(), hex(&data));
                        // Reply so a PER sender can count completed round trips.
                        if let Ok(reply) = link.submit(&data, src, now_ms()) {
                            send_packets(&mut bridge, &reply).map_err(|e| e.to_string())?;
                        }
                    }
                }
                Err(e) => println!("echo: link process error: {e}"),
            }
        }
        let retransmit = link.poll(now_ms());
        if !retransmit.is_empty() {
            send_packets(&mut bridge, &retransmit).map_err(|e| e.to_string())?;
        }
    }
    println!("echo: {frames} frames");
    Ok(())
}

fn hex(data: &[u8]) -> String {
    let mut out = String::with_capacity(data.len() * 2);
    for byte in data {
        out.push_str(&format!("{byte:02x}"));
    }
    out
}
