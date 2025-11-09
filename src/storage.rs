// src/storage.rs

use crate::config::BASE_OFFSET;
use crate::state::{FS_CHANNEL, LFConfig, TELEMETRY_CHANNEL};
use alloc::format;
use alloc::string::ToString;
use alloc::vec::Vec;
use defmt::{error, info, println};
use embassy_executor::task;
use embassy_futures::select::{Either, select};
use embedded_storage::ReadStorage;
use embedded_storage::nor_flash::NorFlash;
use esp_hal::peripherals;
use esp_storage::FlashStorage;
use littlefs2::consts::{U64, U256};
use littlefs2::fs::Filesystem;
use littlefs2::io::{Result as LfsResult, SeekFrom};
use littlefs2::path;
use littlefs2::path::Path;

pub struct StorageWrapper<'a> {
    pub storage: FlashStorage<'a>,
    pub offset: usize,
}

impl littlefs2::driver::Storage for StorageWrapper<'_> {
    const READ_SIZE: usize = 4;
    const WRITE_SIZE: usize = 4;
    const BLOCK_SIZE: usize = FlashStorage::SECTOR_SIZE as usize;
    const BLOCK_COUNT: usize = 0x550;
    const BLOCK_CYCLES: isize = 100;

    type CACHE_SIZE = U64;
    type LOOKAHEAD_SIZE = U256;

    fn read(&mut self, off: usize, buf: &mut [u8]) -> LfsResult<usize> {
        match self.storage.read((self.offset + off) as u32, buf) {
            Ok(_) => Ok(buf.iter().position(|v| *v == 0xff).unwrap_or(buf.len())),
            Err(e) => {
                error!("LFS read error: {}", e);
                Err(littlefs2::io::Error::INVALID)
            }
        }
    }

    fn write(&mut self, off: usize, data: &[u8]) -> LfsResult<usize> {
        match self.storage.write((self.offset + off) as u32, data) {
            Ok(_) => Ok(data.len()),
            Err(e) => {
                error!("LFS write error: {}", e);
                Err(littlefs2::io::Error::INVALID)
            }
        }
    }

    fn erase(&mut self, off: usize, len: usize) -> LfsResult<usize> {
        match self
            .storage
            .erase((self.offset + off) as u32, (BASE_OFFSET + off + len) as u32) // Używa globalnego BASE_OFFSET
        {
            Ok(_) => Ok(0),
            Err(e) => {
                error!("LFS erase error: {}", e);
                Err(littlefs2::io::Error::INVALID)
            }
        }
    }
}

#[task]
pub async fn telemetry(flash: peripherals::FLASH<'static>, _session: [u8; 16]) {
    info!("Starting telemetry task");
    let rcv = TELEMETRY_CHANNEL.receiver();
    let mut storage = StorageWrapper {
        storage: FlashStorage::new(flash).multicore_auto_park(),
        offset: BASE_OFFSET, // Używamy offsetu z config.rs
    };

    if !Filesystem::is_mountable(&mut storage) {
        info!("Formatting filesystem...");
        let format_err = Filesystem::format(&mut storage);
        if let Err(e) = format_err {
            error!("Failed to format filesystem: {:?}", e.code());
        }
    }

    let mut alloc = Filesystem::allocate();
    let fs = Filesystem::mount(&mut alloc, &mut storage).unwrap();

    let mut telemetry_file = Default::default();

    let meta = fs.open_file_with_options_and_then(
        |o| o.read(true).write(true).create(true),
        path!("config.txt"),
        |file| {
            let mut cfg = LFConfig {
                telemetry_version: 0,
            };
            let mut buf = [0; 128];

            if file.is_empty().is_ok_and(|v| !v) {
                let cnt = file.read(&mut buf)?;
                if cnt > 0 {
                    cfg = serde_json::from_slice(&buf[..cnt]).unwrap_or_default();
                }
            }

            info!("Current telemetry version: {}", cfg.telemetry_version);
            cfg.telemetry_version += 1;

            file.seek(SeekFrom::Start(0))?;
            let serialized = serde_json::to_vec(&cfg).unwrap();
            file.write(&serialized)?;

            telemetry_file = format!("telemetry_{}.csv\0", cfg.telemetry_version);
            Ok(())
        },
    );

    if meta.is_err() {
        error!("Failed to update config, using default telemetry file");
        telemetry_file = "telemetry.csv\0".to_string();
    }

    info!(
        "Will write telemetry to: {}",
        telemetry_file.trim_end_matches('\0')
    );

    loop {
        let event = select(rcv.receive(), FS_CHANNEL.receive()).await;
        match event {
            Either::First(telemetry) => {
                let mut wrt = serde_csv_core::Writer::new();
                let mut buf = [0; 128];

                wrt.serialize(&telemetry, &mut buf).unwrap();

                let e = fs.open_file_with_options_and_then(
                    |o| o.read(true).write(true).append(true).create(true),
                    Path::from_str_with_nul(&telemetry_file).unwrap(),
                    |file| {
                        file.write(&buf)?;
                        Ok(())
                    },
                );

                if let Err(e) = e {
                    error!("Failed to write telemetry: {}", e.code());
                }
            }

            Either::Second(fs_request) => match fs_request {
                crate::state::FilesystemRequest::GetFile(path, sender) => {
                    let e = fs.open_file_with_options_and_then(
                        |o| o.read(true),
                        Path::from_str_with_nul(&path).unwrap(),
                        |file| {
                            let mut result: Vec<u8> = Vec::with_capacity(file.len()?);
                            let mut buf = [0; 100];

                            while let Ok(b) = file.read(&mut buf) {
                                if b == 0 {
                                    break;
                                }
                                result.extend_from_slice(&buf[..b]);
                            }

                            sender.send(result).unwrap();
                            Ok(())
                        },
                    );

                    if let Err(e) = e {
                        println!("{=str}", format!("{:?}", e));
                        error!("Failed to read file: {}", e.code());
                    }
                }
                crate::state::FilesystemRequest::WriteFile(path, items) => {
                    let e = fs.open_file_with_options_and_then(
                        |o| o.write(true).create(true).truncate(true),
                        Path::from_str_with_nul(&path).unwrap(),
                        |file| {
                            let written = file.write(&items)?;
                            info!("{} bytes written of {}", written, items.len());
                            file.sync()?;
                            Ok(())
                        },
                    );

                    if let Err(e) = e {
                        error!("Failed to write telemetry: {}", e.code());
                    }
                }
            },
        }
    }
}
