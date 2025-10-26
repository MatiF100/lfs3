#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![allow(static_mut_refs)]

// MEMORY LAYOUT
/*
    0x10000 OFFSET, 0xAA0000 BYTES -> PROGRAM
    0xAB0000 OFFSET, 0x550000 BYTES -> LittleFS

*/
//
use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, Timer};
use embedded_storage::ReadStorage;
use embedded_storage::nor_flash::NorFlash;
use esp_hal::analog::adc::{self, Adc, AdcConfig};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{LSGlobalClkSource, Ledc, LowSpeed, channel, timer};
use esp_hal::peripherals;
use esp_hal::rmt::Rmt;
use esp_hal::system::Stack;
use esp_hal::time::Rate;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal_smartled::{SmartLedsAdapter, smart_led_buffer};
use esp_println as _;
use esp_radio::wifi::Config;
use esp_storage::FlashStorage;

use littlefs2::consts::{U8, U64, U256};
use littlefs2::io::SeekFrom;
use littlefs2::{driver::Storage, fs::Filesystem};
use littlefs2::{path, ram_storage};

use serde::{Deserialize, Serialize};
use smart_leds::SmartLedsWrite;
use smart_leds::brightness;
use smart_leds::colors::RED;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    error!("{}", info);
    loop {}
}

extern crate alloc;
const LED_LEVEL: u8 = 10;
const SENSOR_COUNT: usize = 8;
static SENSOR_CHANNEL: Watch<CriticalSectionRawMutex, [u16; SENSOR_COUNT], 2> = Watch::new();
static CONTROL_CHANNEL: Watch<CriticalSectionRawMutex, Control, 2> = Watch::new();
const MAX_POSITION: u16 = SENSOR_COUNT as u16 * 1000;
const OTHER_STACK_SIZE: usize = 4 * 1024;
static mut OTHER_STACK: Stack<OTHER_STACK_SIZE> = Stack::new();

const BASE_OFFSET: usize = 0xab0000;

struct StorageWrapper<'a>{
    storage: FlashStorage<'a>,
    offset: usize
}
impl littlefs2::driver::Storage for StorageWrapper<'_> {
    const READ_SIZE: usize = 4;

    const WRITE_SIZE: usize = 4;

    const BLOCK_SIZE: usize = FlashStorage::SECTOR_SIZE as usize;

    const BLOCK_COUNT: usize = 0x550;

    const BLOCK_CYCLES: isize = 100;

    type CACHE_SIZE = U64;

    type LOOKAHEAD_SIZE = U256;

    fn read(&mut self, off: usize, buf: &mut [u8]) -> littlefs2::io::Result<usize> {
        match self.storage.read((self.offset + off) as u32, buf) {
            //Ok(_) => {info!("{}", buf); Ok(buf.iter().position(|v| *v == 0xff).unwrap_or(buf.len()))},
            Ok(_) => Ok(buf.iter().position(|v| *v == 0xff).unwrap_or(buf.len())),
            Err(e) => Err(littlefs2::io::Error::INVALID),
        }
    }

    fn write(&mut self, off: usize, data: &[u8]) -> littlefs2::io::Result<usize> {
        match self.storage.write((self.offset + off) as u32, data) {
            Ok(_) => Ok(data.len()),
            Err(e) => Err(littlefs2::io::Error::INVALID),
        }
    }

    fn erase(&mut self, off: usize, len: usize) -> littlefs2::io::Result<usize> {
        //return Ok(len);
        match self
            .storage
            .erase((self.offset + off) as u32, (BASE_OFFSET + off + len) as u32)
        {
            Ok(_) => Ok(0),
            Err(e) => Err(littlefs2::io::Error::INVALID),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct TelemetryPacket {
    timestamp: u64,
    temperature: f32,
}

#[derive(Clone, Copy, Debug, Default)]
struct Control {
    set_led: bool,
    calibrated: bool,
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main(entry = "core0")]
async fn main(spawner: Spawner) {
    // generator version: 0.5.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    //esp_rtos::init(timer0.alarm0);
    esp_rtos::start(timer0.alarm0);

    let software_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    info!("Embassy initialized!");

    let mut storage = StorageWrapper{
        storage: FlashStorage::new(peripherals.FLASH),
        offset: BASE_OFFSET,
    };

    /*let format_err = Filesystem::format(&mut storage);
    if let Err(e) = format_err{
        info!("{:?}", e.code());
    }
    */
    // must allocate state statically before use
    let mut alloc = Filesystem::allocate();
    let mut fs = Filesystem::mount(&mut alloc, &mut storage).unwrap();

    let example_exist = fs.exists(path!("example.txt"));

    let mut buf = [0u8; 11];
    let o = fs.open_file_with_options_and_then(
        |options| options.read(true).write(true).create(true).append(true),
        path!("exampl.txt"),
        |file| {
            if !example_exist{

            let err = file.write(b"Why is black smoke coming out?!");
            match err {
                Ok(v) => info!("{}", v),
                Err(e) => error!("{}", e.code()),
            }
            }
            file.seek(SeekFrom::End(-24)).unwrap();
            file.read(&mut buf).unwrap();
            //assert_eq!(file.read(&mut buf)?, 11);
            Ok(())
        },
    );
    if let Err(o) = o {
        info!("{:?}", o.code());
    }
    info!("{}", buf);
    info!("example: {}", fs.exists(path!("example.txt")));

    let rng = esp_hal::rng::Rng::new();
    let timer1 = TimerGroup::new(peripherals.TIMG0);
    let wifi_init = esp_radio::init().expect("Failed to initialize WIFI/BLE controller");

    let mut led = {
        let freq = Rate::from_khz(80000);
        let rmt = Rmt::new(peripherals.RMT, freq).expect("Failed to init RMT");
        SmartLedsAdapter::new(rmt.channel0, peripherals.GPIO48, smart_led_buffer!(1))
    };
    led.write(brightness([RED].into_iter(), LED_LEVEL)).unwrap();

    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        software_interrupt.software_interrupt0,
        software_interrupt.software_interrupt1,
        unsafe { &mut OTHER_STACK },
        move || {
            let (mut _wifi_controller, _interfaces) =
                esp_radio::wifi::new(&wifi_init, peripherals.WIFI, Config::default())
                    .expect("Failed to initialize WIFI controller");

            info!("Info from 2nd core!");
            loop {}
        },
    );

    //let fs = fatfs::FileSystem::new(disk, options);

    let sensor_comm = SENSOR_CHANNEL.sender();
    let mut sensor_rcv = SENSOR_CHANNEL.receiver().unwrap();

    let mut control_rcv = CONTROL_CHANNEL.receiver().unwrap();
    // TODO: Spawn some tasks
    let _ = spawner;

    spawner.spawn(read_sensors()).unwrap();

    let mut pwm = Ledc::new(peripherals.LEDC);
    pwm.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lsttiemr0 = pwm.timer::<LowSpeed>(timer::Number::Timer0);
    lsttiemr0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty7Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(24),
        })
        .expect("Failed to init timer");

    let mut left_motor_forward = pwm.channel(channel::Number::Channel0, peripherals.GPIO39);
    left_motor_forward
        .configure(channel::config::Config {
            timer: &lsttiemr0,
            duty_pct: 10,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .expect("Failed to init PWM");
    let mut left_motor_backward = pwm.channel(channel::Number::Channel1, peripherals.GPIO38);
    left_motor_backward
        .configure(channel::config::Config {
            timer: &lsttiemr0,
            duty_pct: 10,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .expect("Failed to init PWM");

    let mut right_motor_forward = pwm.channel(channel::Number::Channel2, peripherals.GPIO37);
    right_motor_forward
        .configure(channel::config::Config {
            timer: &lsttiemr0,
            duty_pct: 10,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .expect("Failed to init PWM");
    let mut right_motor_backward = pwm.channel(channel::Number::Channel3, peripherals.GPIO36);
    right_motor_backward
        .configure(channel::config::Config {
            timer: &lsttiemr0,
            duty_pct: 10,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .expect("Failed to init PWM");

    left_motor_forward.set_duty(0).expect("Failed setting PWM");
    left_motor_backward.set_duty(0).expect("Failed setting PWM");
    right_motor_forward.set_duty(0).expect("Failed setting PWM");
    right_motor_backward
        .set_duty(0)
        .expect("Failed setting PWM");
    /*
    let mut left_motor_forward = mcpwm
        .operator0
        .with_pins(peripherals.GPIO11, PwmPinConfig::UP_ACTIVE_HIGH);
    let mut left_motor_backward = mcpwm
        .operator1
        .with_pin_a(peripherals.GPIO12, PwmPinConfig::UP_ACTIVE_HIGH);
    let mut right_motor_forward = mcpwm
        .operator2
        .with_pin_a(peripherals.GPIO13, PwmPinConfig::UP_ACTIVE_HIGH);
    let mut right_motor_backward = mcpwm
        .operator
        .with_pin_a(peripherals.GPIO14, PwmPinConfig::UP_ACTIVE_HIGH);
    */
    // Pins
    //let mut led = Output::new(peripherals.GPIO38, Level::Low, OutputConfig::default());
    let mut emiter = Output::new(peripherals.GPIO16, Level::High, OutputConfig::default());

    // ADC

    let mut adc_config = AdcConfig::new();
    let mut sensor1 = adc_config.enable_pin(peripherals.GPIO9, adc::Attenuation::_11dB);
    let mut sensor2 = adc_config.enable_pin(peripherals.GPIO3, adc::Attenuation::_11dB);
    let mut sensor3 = adc_config.enable_pin(peripherals.GPIO8, adc::Attenuation::_11dB);
    let mut sensor4 = adc_config.enable_pin(peripherals.GPIO6, adc::Attenuation::_11dB);
    let mut sensor5 = adc_config.enable_pin(peripherals.GPIO10, adc::Attenuation::_11dB);
    let mut sensor6 = adc_config.enable_pin(peripherals.GPIO5, adc::Attenuation::_11dB);
    let mut sensor7 = adc_config.enable_pin(peripherals.GPIO4, adc::Attenuation::_11dB);
    let mut sensor8 = adc_config.enable_pin(peripherals.GPIO7, adc::Attenuation::_11dB);
    let mut adc = Adc::new(peripherals.ADC1, adc_config);

    /*
    let mut left_motor_forward =
        Output::new(peripherals.GPIO11, Level::Low, OutputConfig::default());
    let mut left_motor_backward =
        Output::new(peripherals.GPIO12, Level::Low, OutputConfig::default());
    let mut right_motor_forward =
        Output::new(peripherals.GPIO13, Level::Low, OutputConfig::default());
    let mut right_motor_backward =
        Output::new(peripherals.GPIO14, Level::Low, OutputConfig::default());*/

    let mut calibration = [(0, 4096); SENSOR_COUNT];
    let mut mins = [0; SENSOR_COUNT];
    let mut maxs = [4096; SENSOR_COUNT];

    let mut kp = 0.005;
    let mut ki = 0.;
    let mut kd = 0.025;
    let mut last_error = 0.;
    let mut base_speed = 50;
    let mut max_speed = 80;
    let mut turn_speed = 70;

    let mut last_seen = 0;
    let mut lost = false;
    let mut lost_sen = 0;
    let mut last_detection_time = Instant::now();
    let mut line_th = 3000;

    // Calibrate sensors
    for j in 0..10 {
        emiter.set_low();
        Timer::after(Duration::from_millis(50)).await;
        let pin_val1 = adc.read_blocking(&mut sensor1);
        let pin_val2 = adc.read_blocking(&mut sensor2);
        let pin_val3 = adc.read_blocking(&mut sensor3);
        let pin_val4 = adc.read_blocking(&mut sensor4);
        let pin_val5 = adc.read_blocking(&mut sensor5);
        let pin_val6 = adc.read_blocking(&mut sensor6);
        let pin_val7 = adc.read_blocking(&mut sensor7);
        let pin_val8 = adc.read_blocking(&mut sensor8);
        let pin_val = [
            pin_val1, pin_val2, pin_val3, pin_val4, pin_val5, pin_val6, pin_val7, pin_val8,
        ];

        for (i, p) in pin_val.iter().enumerate() {
            if j == 0 || *p > maxs[i] {
                maxs[i] = *p;
            }
        }

        emiter.set_high();
        Timer::after(Duration::from_millis(50)).await;
        let pin_val1 = adc.read_blocking(&mut sensor1);
        let pin_val2 = adc.read_blocking(&mut sensor2);
        let pin_val3 = adc.read_blocking(&mut sensor3);
        let pin_val4 = adc.read_blocking(&mut sensor4);
        let pin_val5 = adc.read_blocking(&mut sensor5);
        let pin_val6 = adc.read_blocking(&mut sensor6);
        let pin_val7 = adc.read_blocking(&mut sensor7);
        let pin_val8 = adc.read_blocking(&mut sensor8);
        let pin_val = [
            pin_val1, pin_val2, pin_val3, pin_val4, pin_val5, pin_val6, pin_val7, pin_val8,
        ];

        for (i, p) in pin_val.iter().enumerate() {
            if j == 0 || *p < mins[i] {
                mins[i] = *p;
            }
        }
    }
    for (i, p) in calibration.iter_mut().enumerate() {
        *p = (mins[i], maxs[i]);
    }
    info!("Calibration: {:?}", calibration);
    let mut position = 0;

    loop {
        // Check control signals
        let control = control_rcv.try_changed();

        if let Some(control) = control {
            if control.set_led {
                //led.set_high();
            } else {
                //led.set_low();
            }

            // Apply control signals
        }
        // Trigger sensors
        let s_time = Instant::now();
        let pin_val1 = adc.read_blocking(&mut sensor1);
        let pin_val2 = adc.read_blocking(&mut sensor2);
        let pin_val3 = adc.read_blocking(&mut sensor3);
        let pin_val4 = adc.read_blocking(&mut sensor4);
        let pin_val5 = adc.read_blocking(&mut sensor5);
        let pin_val6 = adc.read_blocking(&mut sensor6);
        let pin_val7 = adc.read_blocking(&mut sensor7);
        let pin_val8 = adc.read_blocking(&mut sensor8);
        let pin_val = [
            pin_val1, pin_val2, pin_val3, pin_val4, pin_val5, pin_val6, pin_val7, pin_val8,
        ];
        emiter.set_low();
        let total_time = Instant::now().checked_duration_since(s_time).unwrap();

        let normalized = normalize(&calibration, &pin_val);
        position = get_position(&normalized, position);

        sensor_comm.send(normalized);

        let error = (position as i32 - (SENSOR_COUNT as i32 - 1) * 1000 / 2) as f32;
        let mut motor_speed = (kp * error + kd * (error - last_error)) as i32;
        if motor_speed > 100 {
            motor_speed = 100;
        }
        last_error = error;

        let mut rms = base_speed as i32 * (100 + motor_speed) / 100;
        let mut lms = base_speed as i32 * (100 - motor_speed) / 100;

        if rms > max_speed {
            rms = max_speed;
        }
        if lms > max_speed {
            lms = max_speed;
        }
        if rms < 0 {
            rms = 0;
        }
        if lms < 0 {
            lms = 0;
        }

        let c_time = Instant::now();
        let delta_time = c_time.checked_duration_since(last_detection_time).unwrap();
        if pin_val[0] > 3000 && *pin_val.last().unwrap() < 3000 {
            if last_seen != 1 && delta_time.as_millis() >= 100 {
                last_seen = 1;
                last_detection_time = c_time;
                // On = left
                //led.set_high();
            }
        }
        if pin_val[0] < 3000 && *pin_val.last().unwrap() > 3000 {
            if last_seen != 2 && delta_time.as_millis() >= 100 {
                last_seen = 2;
                last_detection_time = c_time;
                // Off = right
                //led.set_low();
            }
        }

        //lost_sen = 0;

        if pin_val.iter().all(|pin| *pin < line_th) {
            lost = true;
        }

        if lost && last_seen == 1 {
            left_motor_forward
                .set_duty(turn_speed)
                .expect("Failed to set PWM");
            left_motor_backward
                .set_duty(turn_speed)
                .expect("Failed to set PWM");
            right_motor_forward.set_duty(0).expect("Failed to set PWM");
            right_motor_backward.set_duty(0).expect("Failed to set PWM");
        } else if lost && last_seen == 2 {
            left_motor_forward.set_duty(0).expect("Failed to set PWM");
            left_motor_backward.set_duty(0).expect("Failed to set PWM");
            right_motor_forward
                .set_duty(turn_speed)
                .expect("Failed to set PWM");
            right_motor_backward
                .set_duty(turn_speed)
                .expect("Failed to set PWM");
        } else {
            left_motor_forward
                .set_duty(lms as u8)
                .expect("Failed to set PWM");
            left_motor_backward.set_duty(0).expect("Failed to set PWM");
            right_motor_forward
                .set_duty(rms as u8)
                .expect("Failed to set PWM");
            right_motor_backward.set_duty(0).expect("Failed to set PWM");
        }

        // Debug info
        for (i, v) in sensor_rcv.get().await.iter().enumerate() {
            info!("Read value S{}: {}", i, v);
        }
        info!("Calculated position: {}", position);
        info!("Calculated speed: {}", motor_speed);
        info!("Pin status: {}", emiter.is_set_high());
        info!("Time passed: {}us", total_time.as_micros());
        info!("{:?}", control_rcv.get().await.set_led);
        info!("==============");

        // Apply settings

        // Notify outputs

        //led.toggle();

        emiter.set_high();
        Timer::after(Duration::from_millis(500)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-rc.0/examples/src/bin
}

fn normalize(
    calibration: &[(u16, u16); SENSOR_COUNT],
    values: &[u16; SENSOR_COUNT],
) -> [u16; SENSOR_COUNT] {
    let mut data = [0; SENSOR_COUNT];
    calibration
        .iter()
        .zip(values)
        .enumerate()
        .for_each(|(i, ((min, max), v))| {
            info!("{},{}", v, min);
            data[i] = if min == max {
                1000
            } else if v <= min {
                0
            } else if v >= max {
                1000
            } else {
                (*v as u32 - *min as u32) * 1000 / (*max as u32 - *min as u32)
            } as u16;
        });
    data
}

fn get_position(values: &[u16; SENSOR_COUNT], last_position: u16) -> u16 {
    let mut line_seen = false;
    let (avg, sum) = values
        .iter()
        .enumerate()
        .fold((0, 0), |(mut avg, mut sum), (i, el)| {
            if *el > 200 {
                line_seen = true;
            }
            if *el > 50 {
                avg += *el as u32 * i as u32 * 1000;
                sum += *el;
            }
            (avg, sum)
        });

    if !line_seen {
        if last_position < (SENSOR_COUNT as u16 - 1) * 1000 / 2 {
            return 0;
        } else {
            return (SENSOR_COUNT as u16 - 1) * 1000;
        }
    }

    (avg / sum as u32) as u16
}

#[embassy_executor::task]
async fn read_sensors() {
    // Variables
    let mut sensor_rcv = SENSOR_CHANNEL.receiver().unwrap();
    let control_comm = CONTROL_CHANNEL.sender();
    let mut control = Control::default();

    loop {
        let pin_val = sensor_rcv.changed().await;

        // Calculate controls

        control_comm.send(control);
    }
}

#[embassy_executor::task]
async fn telemetry(
    flash: peripherals::FLASH<'static>
) {
    // Variables
    let mut sensor_rcv = SENSOR_CHANNEL.receiver().unwrap();
    let control_comm = CONTROL_CHANNEL.sender();
    let mut control = Control::default();

    loop {
        let pin_val = sensor_rcv.changed().await;

        // Calculate controls

        control_comm.send(control);
    }
}