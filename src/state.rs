// src/state.rs

use crate::config::SENSOR_COUNT;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct TelemetryPacket {
    pub timestamp: u64,
    pub temperature: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Control {
    pub set_led: bool,
    pub calibrated: bool,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct LFConfig {
    pub telemetry_version: usize,
}

pub static SENSOR_WATCH: Watch<CriticalSectionRawMutex, [u16; SENSOR_COUNT], 2> = Watch::new();

pub static CONTROL_WATCH: Watch<CriticalSectionRawMutex, Control, 2> = Watch::new();

pub static TELEMETRY_CHANNEL: Channel<CriticalSectionRawMutex, TelemetryPacket, 1> =
    Channel::<CriticalSectionRawMutex, TelemetryPacket, 1>::new();
