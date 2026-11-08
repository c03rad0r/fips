//! Band profiles and configuration types.

use serde::{Deserialize, Serialize};

/// Supported regulatory / band profiles.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Band {
    Eu868,
    Eu868High,
    Us915,
    Us915Robust,
    Eu2400,
    Us2400,
    Custom,
}

impl Band {
    pub fn parse(s: &str) -> Option<Self> {
        match s.trim().to_ascii_lowercase().as_str() {
            "eu868" | "eu-868" => Some(Self::Eu868),
            "eu868-high" | "eu868_high" | "eu-868-high" => Some(Self::Eu868High),
            "us915" | "us-915" => Some(Self::Us915),
            "us915-robust" | "us915_robust" | "us-915-robust" => Some(Self::Us915Robust),
            "eu2400" | "eu-2400" | "2g4" | "2400" => Some(Self::Eu2400),
            "us2400" | "us-2400" => Some(Self::Us2400),
            "custom" => Some(Self::Custom),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Eu868 => "eu868",
            Self::Eu868High => "eu868-high",
            Self::Us915 => "us915",
            Self::Us915Robust => "us915-robust",
            Self::Eu2400 => "eu2400",
            Self::Us2400 => "us2400",
            Self::Custom => "custom",
        }
    }

    pub fn default_profile(self) -> RadioProfile {
        match self {
            Self::Eu868 => RadioProfile {
                band: self,
                frequency_hz: 868_100_000,
                bitrate_kbps: 650,
                tx_power_dbm: 14,
                sync_word: DEFAULT_SYNC_WORD,
                radio_payload: 255,
                sub_ghz: true,
            },
            Self::Eu868High => RadioProfile {
                band: self,
                frequency_hz: 869_525_000,
                bitrate_kbps: 650,
                tx_power_dbm: 14,
                sync_word: DEFAULT_SYNC_WORD,
                radio_payload: 255,
                sub_ghz: true,
            },
            Self::Us915 => RadioProfile {
                band: self,
                frequency_hz: 915_000_000,
                bitrate_kbps: 1300,
                tx_power_dbm: 22,
                sync_word: DEFAULT_SYNC_WORD,
                radio_payload: 255,
                sub_ghz: true,
            },
            Self::Us915Robust => RadioProfile {
                band: self,
                frequency_hz: 915_000_000,
                bitrate_kbps: 650,
                tx_power_dbm: 22,
                sync_word: DEFAULT_SYNC_WORD,
                radio_payload: 255,
                sub_ghz: true,
            },
            Self::Eu2400 | Self::Us2400 => RadioProfile {
                band: self,
                frequency_hz: 2_440_000_000,
                bitrate_kbps: 2600,
                tx_power_dbm: 12,
                sync_word: DEFAULT_SYNC_WORD,
                radio_payload: 255,
                sub_ghz: false,
            },
            Self::Custom => RadioProfile {
                band: self,
                frequency_hz: 2_440_000_000,
                bitrate_kbps: 2600,
                tx_power_dbm: 12,
                sync_word: DEFAULT_SYNC_WORD,
                radio_payload: 255,
                sub_ghz: false,
            },
        }
    }
}

pub const DEFAULT_SYNC_WORD: [u8; 4] = [0x12, 0xAD, 0x10, 0x1B];

/// Radio parameters sent to the bridge.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct RadioProfile {
    pub band: Band,
    pub frequency_hz: u32,
    /// FLRC bitrate in kbps (2600, 1300, 650, 325).
    pub bitrate_kbps: u16,
    pub tx_power_dbm: i8,
    pub sync_word: [u8; 4],
    /// Raw FLRC payload bytes per radio packet (255 or 511).
    pub radio_payload: u16,
    /// True for the sub-GHz (LF) front end, false for 2.4 GHz (HF).
    pub sub_ghz: bool,
}

impl RadioProfile {
    pub fn validate(&self) -> Result<(), String> {
        if self.frequency_hz < 150_000_000 || self.frequency_hz > 2_500_000_000 {
            return Err(format!(
                "frequency {} Hz is outside the LR2021 range",
                self.frequency_hz
            ));
        }
        match self.bitrate_kbps {
            2600 | 2080 | 1300 | 650 | 325 => {}
            other => return Err(format!("unsupported FLRC bitrate {other} kbps")),
        }
        if self.radio_payload < 32 || self.radio_payload > 511 {
            return Err(format!(
                "radio_payload {} is outside 32..=511",
                self.radio_payload
            ));
        }
        if self.tx_power_dbm < -18 || self.tx_power_dbm > 22 {
            return Err(format!(
                "tx_power {} dBm is outside -18..=22",
                self.tx_power_dbm
            ));
        }
        if self.sub_ghz != (self.frequency_hz < 1_000_000_000) {
            return Err("sub_ghz does not match frequency".to_string());
        }
        Ok(())
    }
}

/// Host-side link configuration.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LinkConfig {
    pub local_id: u32,
    /// Largest FIPS frame accepted from / delivered to FIPS.
    pub mtu: u16,
    /// Raw radio packet size; must match the bridge configuration.
    pub radio_payload: u16,
    pub max_retries: u8,
    pub ack_timeout_ms: u64,
    pub reassembly_timeout_ms: u64,
    pub dedup_window_ms: u64,
}

impl Default for LinkConfig {
    fn default() -> Self {
        Self {
            local_id: 1,
            mtu: 1400,
            radio_payload: 255,
            max_retries: 3,
            ack_timeout_ms: 40,
            reassembly_timeout_ms: 500,
            dedup_window_ms: 5_000,
        }
    }
}

impl LinkConfig {
    /// Maximum DATA payload per radio packet after the link header and CRC.
    pub fn max_fragment_payload(&self) -> usize {
        let overhead = crate::frame::HEADER_SIZE + crate::frame::CRC_SIZE;
        (self.radio_payload as usize).saturating_sub(overhead)
    }
}

/// Bridge/serial configuration.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BridgeConfig {
    pub port: String,
    pub baud: u32,
    pub profile: RadioProfile,
    pub relay: bool,
    /// FIPS link ID, also used by TDMA-capable ESP bridges as the slot role
    /// (`local_id & 1`). Peers must use opposite parity (normally 1 and 2).
    pub local_id: u32,
}

impl Default for BridgeConfig {
    fn default() -> Self {
        Self {
            port: "/dev/ttyACM0".to_string(),
            baud: 2_000_000,
            profile: Band::Eu2400.default_profile(),
            relay: false,
            local_id: 1,
        }
    }
}

/// User-facing FIPS configuration for the `lr2021` transport.
///
/// Every field has a default so a minimal config is:
/// ```yaml
/// transports:
///   lr2021:
///     port: /dev/ttyACM0
///     band: us915
/// ```
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Lr2021Config {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub port: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub baud: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub band: Option<Band>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub frequency_hz: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bitrate_kbps: Option<u16>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tx_power_dbm: Option<i8>,
    /// Four sync-word bytes as hex, e.g. "12ad101b".
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sync_word: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mtu: Option<u16>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub radio_payload: Option<u16>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub local_id: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub relay: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub auto_connect: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ack_timeout_ms: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_retries: Option<u8>,
}

impl Lr2021Config {
    pub fn band(&self) -> Band {
        self.band.unwrap_or(Band::Us2400)
    }

    pub fn profile(&self) -> Result<RadioProfile, String> {
        let mut profile = self.band().default_profile();
        if let Some(frequency_hz) = self.frequency_hz {
            profile.frequency_hz = frequency_hz;
            profile.sub_ghz = frequency_hz < 1_000_000_000;
        }
        if let Some(bitrate_kbps) = self.bitrate_kbps {
            profile.bitrate_kbps = bitrate_kbps;
        }
        if let Some(tx_power_dbm) = self.tx_power_dbm {
            profile.tx_power_dbm = tx_power_dbm;
        }
        if let Some(radio_payload) = self.radio_payload {
            profile.radio_payload = radio_payload;
        }
        if let Some(sync_word) = &self.sync_word {
            profile.sync_word = parse_sync_word(sync_word)?;
        }
        profile.validate()?;
        Ok(profile)
    }

    pub fn port(&self) -> &str {
        self.port.as_deref().unwrap_or("/dev/ttyACM0")
    }

    pub fn baud(&self) -> u32 {
        self.baud.unwrap_or(2_000_000)
    }

    pub fn relay(&self) -> bool {
        self.relay.unwrap_or(false)
    }

    pub fn auto_connect(&self) -> bool {
        self.auto_connect.unwrap_or(false)
    }

    pub fn local_id(&self) -> u32 {
        self.local_id.unwrap_or(1)
    }

    pub fn bridge_config(&self) -> Result<BridgeConfig, String> {
        Ok(BridgeConfig {
            port: self.port().to_string(),
            baud: self.baud(),
            profile: self.profile()?,
            relay: self.relay(),
            local_id: self.local_id(),
        })
    }

    pub fn link_config(&self) -> Result<LinkConfig, String> {
        let profile = self.profile()?;
        let mut cfg = LinkConfig {
            local_id: self.local_id(),
            radio_payload: profile.radio_payload,
            ..LinkConfig::default()
        };
        if let Some(mtu) = self.mtu {
            cfg.mtu = mtu;
        }
        if let Some(ack_timeout_ms) = self.ack_timeout_ms {
            cfg.ack_timeout_ms = ack_timeout_ms;
        }
        if let Some(max_retries) = self.max_retries {
            cfg.max_retries = max_retries;
        }
        if cfg.mtu < 64 {
            return Err("mtu must be at least 64".to_string());
        }
        if cfg.max_fragment_payload() < 32 {
            return Err("radio_payload is too small for the link header".to_string());
        }
        Ok(cfg)
    }
}

pub fn parse_sync_word(s: &str) -> Result<[u8; 4], String> {
    let clean: String = s.chars().filter(|c| c.is_ascii_hexdigit()).collect();
    if clean.len() != 8 {
        return Err("sync_word must contain exactly 8 hex digits".to_string());
    }
    let mut out = [0u8; 4];
    for (i, byte) in out.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&clean[i * 2..i * 2 + 2], 16)
            .map_err(|_| "invalid sync_word".to_string())?;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn band_profiles_validate() {
        for band in [
            Band::Eu868,
            Band::Eu868High,
            Band::Us915,
            Band::Us915Robust,
            Band::Eu2400,
            Band::Us2400,
            Band::Custom,
        ] {
            band.default_profile().validate().unwrap();
        }
    }

    #[test]
    fn sync_word_parses() {
        assert_eq!(parse_sync_word("12ad101b").unwrap(), DEFAULT_SYNC_WORD);
        assert_eq!(parse_sync_word("12:AD:10:1B").unwrap(), DEFAULT_SYNC_WORD);
        assert!(parse_sync_word("12ad10").is_err());
    }

    #[test]
    fn custom_frequency_selects_band_path() {
        let cfg = Lr2021Config {
            band: Some(Band::Custom),
            frequency_hz: Some(868_300_000),
            ..Lr2021Config::default()
        };
        let profile = cfg.profile().unwrap();
        assert!(profile.sub_ghz);
        assert_eq!(profile.frequency_hz, 868_300_000);
    }
}
