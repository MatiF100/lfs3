// src/config.rs

use core::net::Ipv4Addr;
use esp_hal::analog::adc;
use esp_hal::time::Rate;

// ##################################################
// #      SYSTEM & HARDWARE CONFIG
// ##################################################

/// Rozmiar stosu dla drugiego rdzenia.
pub const OTHER_STACK_SIZE: usize = 4 * 1024;
/// Główna pętla sterowania (PID) będzie czekać tyle milisekund.
pub const MAIN_LOOP_DELAY_MS: u64 = 500;
/// Poziom jasności dla SmartLED (0-255).
pub const LED_LEVEL: u8 = 10;
/// Częstotliwość RMT dla SmartLED w kHz.
pub const SMARTLED_RMT_FREQ_KHZ: u32 = 80_000;
/// Częstotliwość PWM dla sterowników silników w kHz.
pub const MOTOR_PWM_FREQ_KHZ: u32 = 24;

// ##################################################
// #      NETWORK & WEB SERVER CONFIG
// ##################################################

// --- Konfiguracja Wi-Fi ---
/// SSID sieci Wi-Fi, do której robot ma się podłączyć (tryb STA).
pub const SSID: &str = "Xiaomi_AAD9";
/// Hasło do sieci Wi-Fi (tryb STA).
pub const PASSWORD: &str = "fesz987654321";
/// Nazwa sieci, którą robot będzie rozgłaszał (tryb AP).
pub const AP_SSID: &str = "esp-radio";

// --- Konfiguracja adresów IP ---
/// Statyczny adres IP dla robota w trybie AP.
pub const AP_IP_ADDR: Ipv4Addr = Ipv4Addr::new(192, 168, 99, 1);
/// Maska podsieci dla trybu AP (np. 24 dla /24).
pub const AP_IP_CIDR: u8 = 24;
/// Ziarno (seed) dla stosów sieciowych embassy-net.
pub const NET_SEED: u64 = 1234;

// --- Konfiguracja Picoserve ---
/// Port, na którym działa serwer Picoserve.
pub const WEB_PORT: u16 = 80;
/// Liczba tasków obsługujących żądania HTTP.
pub const WEB_TASK_POOL_SIZE: usize = 3;
/// Rozmiar bufora RX dla gniazda TCP Picoserve.
pub const PICOSERVE_TCP_RX_BUFFER: usize = 1024;
/// Rozmiar bufora TX dla gniazda TCP Picoserve.
pub const PICOSERVE_TCP_TX_BUFFER: usize = 1024;
/// Rozmiar bufora roboczego HTTP Picoserve.
pub const PICOSERVE_HTTP_BUFFER: usize = 2048;

// --- Konfiguracja starego serwera (można usunąć, jeśli nie jest używany) ---
#[deprecated = "Używany przez stary, ręczny serwer TCP"]
pub const LEGACY_WEB_PORT: u16 = 8080;
#[deprecated = "Używany przez stary, ręczny serwer TCP"]
pub const TCP_BUFFER_SIZE: usize = 1536;

// ##################################################
// #      ROBOT LOGIC & SENSOR CONFIG
// ##################################################

// --- Konfiguracja czujników ---
/// Liczba czujników linii.
pub const SENSOR_COUNT: usize = 8;
/// Maksymalna wartość pozycji (SENSOR_COUNT * 1000).
pub const MAX_POSITION: u16 = SENSOR_COUNT as u16 * 1000;
/// Poziom tłumienia dla wejść ADC czujników.
pub const SENSOR_ADC_ATTENUATION: adc::Attenuation = adc::Attenuation::_11dB;
/// Liczba kroków (pomiarów) podczas kalibracji.
pub const CALIBRATION_STEPS: usize = 10;
/// Czas oczekiwania między krokami kalibracji w ms.
pub const CALIBRATION_DELAY_MS: u64 = 50;

// --- Konfiguracja logiki linii ---
/// Próg ADC, powyżej którego czujnik uznaje, że "coś widzi" (używane do wykrywania zgubienia linii).
pub const LINE_THRESHOLD: u16 = 3000;
/// Próg normalizowany (0-1000), powyżej którego czujnik jest brany pod uwagę przy liczeniu pozycji.
pub const SENSOR_WEIGHT_THRESHOLD: u16 = 50;
/// Próg normalizowany (0-1000), powyżej którego uznajemy, że linia jest na pewno widziana.
pub const SENSOR_LINE_SEEN_THRESHOLD: u16 = 200;

// --- Konfiguracja PID i silników ---
/// Wzmocnienie proporcjonalne (Kp) regulatora PID.
pub const PID_KP: f32 = 0.005;
/// Wzmocnienie całkujące (Ki) regulatora PID.
pub const PID_KI: f32 = 0.0;
/// Wzmocnienie różniczkujące (Kd) regulatora PID.
pub const PID_KD: f32 = 0.025;

/// Bazowa prędkość silników (0-100).
pub const BASE_SPEED: u8 = 50;
/// Maksymalna prędkość silników (0-100).
pub const MAX_SPEED: i32 = 80;
/// Prędkość używana podczas obrotu w miejscu po zgubieniu linii (0-100).
pub const TURN_SPEED: u8 = 70;

// ##################################################
// #      STORAGE CONFIG
// ##################################################

/// Adres offsetu w pamięci FLASH, gdzie zaczyna się system plików LittleFS.
pub const BASE_OFFSET: usize = 0xab0000;
