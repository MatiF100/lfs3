use core::net::Ipv4Addr;
use esp_hal::analog::adc;

// ##################################################
// #      SYSTEM & HARDWARE CONFIG
// ##################################################

pub const OTHER_STACK_SIZE: usize = 4 * 1024;
pub const MAIN_LOOP_DELAY_MS: u64 = 2;
pub const LED_LEVEL: u8 = 10;
pub const SMARTLED_RMT_FREQ_KHZ: u32 = 80_000;
pub const MOTOR_PWM_FREQ_KHZ: u32 = 24;

// ##################################################
// #      NETWORK & WEB SERVER CONFIG
// ##################################################

pub const SSID: &str = "Xiaomi_AAD9";
pub const PASSWORD: &str = "fesz987654321";
pub const AP_SSID: &str = "esp-radio";

pub const AP_IP_ADDR: Ipv4Addr = Ipv4Addr::new(192, 168, 99, 1);
pub const AP_IP_CIDR: u8 = 24;
pub const NET_SEED: u64 = 1234;

pub const WEB_PORT: u16 = 80;
pub const WEB_TASK_POOL_SIZE: usize = 3;
pub const PICOSERVE_TCP_RX_BUFFER: usize = 1024;
pub const PICOSERVE_TCP_TX_BUFFER: usize = 1024;
pub const PICOSERVE_HTTP_BUFFER: usize = 2048;

pub const LEGACY_WEB_PORT: u16 = 8080;
pub const TCP_BUFFER_SIZE: usize = 1536;

// ##################################################
// #      ROBOT LOGIC & SENSOR CONFIG
// ##################################################

pub const SENSOR_COUNT: usize = 8;
pub const MAX_POSITION: u16 = SENSOR_COUNT as u16 * 1000;
pub const SENSOR_ADC_ATTENUATION: adc::Attenuation = adc::Attenuation::_11dB;
pub const CALIBRATION_STEPS: usize = 10;

// Longer delay allows sensor LED to settle state before taking reads
pub const CALIBRATION_DELAY_MS: u64 = 1;

pub const LINE_THRESHOLD: u16 = 800;
pub const SENSOR_WEIGHT_THRESHOLD: u16 = 200;
pub const SENSOR_LINE_SEEN_THRESHOLD: u16 = 500;

pub const PID_KP: f32 = 0.005;
pub const PID_KI: f32 = 0.0;
pub const PID_KD: f32 = 0.035;

pub const BASE_SPEED: u8 = 70;
pub const MAX_SPEED: i32 = 90;
pub const TURN_SPEED: u8 = 90;

// ##################################################
// #      STORAGE CONFIG
// ##################################################

pub const BASE_OFFSET: usize = 0xab0000;
