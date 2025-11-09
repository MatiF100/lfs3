// src/state.rs

use crate::config::SENSOR_COUNT;
use alloc::string::String;
use alloc::vec::Vec;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct TelemetryPacket {
    pub sensor_values: [u16; SENSOR_COUNT],
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

pub static FS_CHANNEL: Channel<CriticalSectionRawMutex, FilesystemRequest, 1> =
    Channel::<CriticalSectionRawMutex, FilesystemRequest, 1>::new();

pub enum FilesystemRequest {
    GetFile(String, oneshot::Sender<Vec<u8>>),
    WriteFile(String, Vec<u8>),
}
