#![no_std]
#![allow(static_mut_refs)] // Dopuszczone w oryginalnym kodzie
#![feature(impl_trait_in_assoc_type)]

extern crate alloc;

use defmt::info; // 'error' jest w panic_handler, ale 'warn' może się przydać
use embassy_executor::Spawner;
use embassy_time::Duration;
use esp_hal::analog::adc::{Adc, AdcConfig};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{LSGlobalClkSource, Ledc, LowSpeed, channel, timer};
use esp_hal::rmt::Rmt;
use esp_hal::system::Stack;
use esp_hal::time::Rate;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal_smartled::{SmartLedsAdapter, smart_led_buffer};
use picoserve::AppBuilder;
use smart_leds::SmartLedsWrite;
use smart_leds::brightness;
use smart_leds::colors::RED;

pub mod commands;
pub mod config;
pub mod control;
pub mod http_server;
pub mod network;
pub mod state;
pub mod storage;

use crate::config::SENSOR_COUNT;
use crate::http_server::AppProps;

#[macro_export]
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

static mut OTHER_STACK: Stack<{ config::OTHER_STACK_SIZE }> = Stack::new();

pub async fn run(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 128 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_rtos::start(timer0.alarm0);

    let software_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    defmt::info!("Embassy initialized!");

    let rng = esp_hal::rng::Rng::new();
    let _timer1 = TimerGroup::new(peripherals.TIMG0);

    let mut session: [u8; 16] = [0; 16];
    rng.read(&mut session);

    let mut led = {
        let freq = Rate::from_khz(config::SMARTLED_RMT_FREQ_KHZ);
        let rmt = Rmt::new(peripherals.RMT, freq).expect("Failed to init RMT");
        SmartLedsAdapter::new(rmt.channel0, peripherals.GPIO48, smart_led_buffer!(1))
    };
    led.write(brightness([RED].into_iter(), config::LED_LEVEL))
        .unwrap();

    let mut pwm = Ledc::new(peripherals.LEDC);
    pwm.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let lstimer0 = mk_static!(
        esp_hal::ledc::timer::Timer<LowSpeed>,
        pwm.timer::<LowSpeed>(timer::Number::Timer0)
    );
    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty7Bit, // 7-bitowa precyzja (0-127)
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(config::MOTOR_PWM_FREQ_KHZ),
        })
        .expect("Failed to init timer");

    let mut left_motor_forward = pwm.channel(channel::Number::Channel0, peripherals.GPIO39);
    left_motor_forward
        .configure(channel::config::Config {
            timer: lstimer0,
            duty_pct: 0, // Zaczynamy z 0
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .expect("Failed to init PWM");
    let mut left_motor_backward = pwm.channel(channel::Number::Channel1, peripherals.GPIO38);
    left_motor_backward
        .configure(channel::config::Config {
            timer: lstimer0,
            duty_pct: 0,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .expect("Failed to init PWM");

    let mut right_motor_forward = pwm.channel(channel::Number::Channel2, peripherals.GPIO37);
    right_motor_forward
        .configure(channel::config::Config {
            timer: lstimer0,
            duty_pct: 0,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .expect("Failed to init PWM");
    let mut right_motor_backward = pwm.channel(channel::Number::Channel3, peripherals.GPIO36);
    right_motor_backward
        .configure(channel::config::Config {
            timer: lstimer0,
            duty_pct: 0,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .expect("Failed to init PWM");

    let (ap_stack, sta_stack) = network::init_network(spawner, peripherals.WIFI).await;

    let app = mk_static!(
        picoserve::AppRouter<AppProps>,
        http_server::AppProps.build_app()
    );
    let pico_config = mk_static!(
        picoserve::Config<Duration>,
        picoserve::Config::new(picoserve::Timeouts {
            start_read_request: Some(Duration::from_secs(5)),
            persistent_start_read_request: Some(Duration::from_secs(1)),
            read_request: Some(Duration::from_secs(1)),
            write: Some(Duration::from_secs(1)),
        })
        .keep_connection_alive()
    );

    for id in 0..config::WEB_TASK_POOL_SIZE {
        spawner
            .spawn(http_server::web_task(id, sta_stack, app, pico_config))
            .unwrap();
    }

    for id in 0..config::WEB_TASK_POOL_SIZE {
        spawner
            .spawn(http_server::web_task(id, ap_stack, app, pico_config))
            .unwrap();
    }

    spawner
        .spawn(network::run_webserver(ap_stack, sta_stack))
        .unwrap();

    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        software_interrupt.software_interrupt0,
        software_interrupt.software_interrupt1,
        unsafe { &mut OTHER_STACK },
        move || {
            info!("Info from 2nd core!");
            // loop {}
        },
    );

    let sensor_comm = state::SENSOR_WATCH.sender();
    let control_rcv = state::CONTROL_WATCH.receiver().unwrap();
    let tele_sender = state::TELEMETRY_CHANNEL.sender();

    //spawner.spawn(control::read_sensors()).unwrap();
    spawner
        .spawn(storage::telemetry(peripherals.FLASH, session))
        .unwrap();

    tele_sender
        .send(state::TelemetryPacket {
            sensor_values: [0; SENSOR_COUNT],
        })
        .await;
    tele_sender
        .send(state::TelemetryPacket {
            sensor_values: [255; SENSOR_COUNT],
        })
        .await;

    let emiter = Output::new(peripherals.GPIO16, Level::High, OutputConfig::default());

    let mut adc_config = AdcConfig::new();
    let sensor1 = adc_config.enable_pin(peripherals.GPIO9, config::SENSOR_ADC_ATTENUATION);
    let sensor2 = adc_config.enable_pin(peripherals.GPIO3, config::SENSOR_ADC_ATTENUATION);
    let sensor3 = adc_config.enable_pin(peripherals.GPIO8, config::SENSOR_ADC_ATTENUATION);
    let sensor4 = adc_config.enable_pin(peripherals.GPIO6, config::SENSOR_ADC_ATTENUATION);
    let sensor5 = adc_config.enable_pin(peripherals.GPIO10, config::SENSOR_ADC_ATTENUATION);
    let sensor6 = adc_config.enable_pin(peripherals.GPIO5, config::SENSOR_ADC_ATTENUATION);
    let sensor7 = adc_config.enable_pin(peripherals.GPIO4, config::SENSOR_ADC_ATTENUATION);
    let sensor8 = adc_config.enable_pin(peripherals.GPIO7, config::SENSOR_ADC_ATTENUATION);
    let adc = Adc::new(peripherals.ADC1, adc_config);

    info!("All tasks spawned. Starting main control loop...");

    control::run_control_loop(
        adc,
        sensor1,
        sensor2,
        sensor3,
        sensor4,
        sensor5,
        sensor6,
        sensor7,
        sensor8,
        emiter,
        left_motor_forward,
        left_motor_backward,
        right_motor_forward,
        right_motor_backward,
        led,
        control_rcv,
        sensor_comm,
    )
    .await;
}
