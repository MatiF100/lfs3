// src/bin/main.rs

#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
     holding buffers for the duration of a data transfer."
)]
#![allow(static_mut_refs)] // Pozostawione dla zgodności
#![feature(impl_trait_in_assoc_type)] // Pozostawione dla zgodności

extern crate alloc;

use defmt::error;
use embassy_executor::Spawner;
use esp_hal::clock::CpuClock;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::timer::systimer::SystemTimer;
use esp_println as _; // Dla panic handlera

// Importujemy główną funkcję 'run' z naszej biblioteki
use lfs3::run; // <-- ZMIEŃ 'linefollower' jeśli nazwa pakietu w Cargo.toml jest inna

// --- Panic Handler ---
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    error!("{}", info);
    loop {}
}

// --- Deskryptor aplikacji ---
// (Wymagany przez bootloader esp-idf)
esp_bootloader_esp_idf::esp_app_desc!();

// --- Główna funkcja wejściowa ---
#[esp_rtos::main(entry = "core0")]
async fn main(spawner: Spawner) {
    run(spawner).await;
}
