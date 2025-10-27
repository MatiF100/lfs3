// src/state.rs

use crate::config::SENSOR_COUNT;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;
use serde::{Deserialize, Serialize};

// ##################################################
// #      DEFINICJE STRUKTUR STANU
// ##################################################

/// Pakiet danych telemetrycznych wysyłany do zapisu na flash.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct TelemetryPacket {
    pub timestamp: u64,
    pub temperature: f32, // W oryginalnym kodzie to było
                          // TODO: Rozważ dodanie tu odczytów z czujników lub pozycji
}

/// Struktura do wysyłania poleceń sterujących DO robota (np. z web).
#[derive(Clone, Copy, Debug, Default)]
pub struct Control {
    pub set_led: bool,
    pub calibrated: bool,
    // TODO: Rozważ dodanie tu np. zmiany prędkości, nastaw PID
}

/// Struktura konfiguracji zapisywana/odczytywana z LittleFS.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct LFConfig {
    pub telemetry_version: usize,
    // TODO: Rozważ zapisywanie tu danych kalibracyjnych
}

// ##################################################
// #      GLOBALNE OBIEKTY SYNCHRONIZUJĄCE
// ##################################################

/// Kanał `Watch` do publikowania najnowszych (znormalizowanych) odczytów czujników.
///
/// * **Nadawca (Sender):** `control::run_control_loop`
/// * **Odbiorcy (Receiver):** `control::read_sensors`, `http_server` (dla WebSocket)
pub static SENSOR_WATCH: Watch<CriticalSectionRawMutex, [u16; SENSOR_COUNT], 2> = Watch::new();

/// Kanał `Watch` do publikowania poleceń sterujących dla robota.
///
/// * **Nadawca (Sender):** `control::read_sensors` (lub `http_server`, gdy WebSocket to zaimplementuje)
/// * **Odbiorca (Receiver):** `control::run_control_loop`
pub static CONTROL_WATCH: Watch<CriticalSectionRawMutex, Control, 2> = Watch::new();

/// Asynchroniczny kanał `Channel` do kolejkowania pakietów telemetrycznych do zapisu.
///
/// * **Nadawcy (Sender):** Głównie `main` (w przykładzie) lub `control_loop`
/// * **Odbiorca (Receiver):** `storage::telemetry`
pub static TELEMETRY_CHANNEL: Channel<CriticalSectionRawMutex, TelemetryPacket, 1> =
    Channel::<CriticalSectionRawMutex, TelemetryPacket, 1>::new();
