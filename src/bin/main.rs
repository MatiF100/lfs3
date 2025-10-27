// src/bin/main.rs

#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
     holding buffers for the duration of a data transfer."
)]

extern crate alloc;

use defmt::error;
use embassy_executor::Spawner;
use esp_println as _;

use lfs3::run;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    error!("{}", info);
    loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main(entry = "core0")]
async fn main(spawner: Spawner) {
    run(spawner).await;
}
