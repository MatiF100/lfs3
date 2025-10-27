use crate::config::*;
use crate::state::{CONTROL_WATCH, Control, SENSOR_WATCH};
use defmt::info;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::{Receiver, Sender};
use embassy_time::{Duration, Instant, Timer};
use esp_hal::analog::adc::{Adc, AdcPin};
use esp_hal::gpio::{Output};
use esp_hal::ledc::LowSpeed;
use esp_hal::ledc::channel::{self, ChannelIFace};
use esp_hal::{Blocking, peripherals};

#[allow(clippy::too_many_arguments)]
pub async fn run_control_loop(
    mut adc: Adc<'static, peripherals::ADC1<'static>, Blocking>,
    mut sensor1: AdcPin<peripherals::GPIO9<'static>, peripherals::ADC1<'static>>,
    mut sensor2: AdcPin<peripherals::GPIO3<'static>, peripherals::ADC1<'static>>,
    mut sensor3: AdcPin<peripherals::GPIO8<'static>, peripherals::ADC1<'static>>,
    mut sensor4: AdcPin<peripherals::GPIO6<'static>, peripherals::ADC1<'static>>,
    mut sensor5: AdcPin<peripherals::GPIO10<'static>, peripherals::ADC1<'static>>,
    mut sensor6: AdcPin<peripherals::GPIO5<'static>, peripherals::ADC1<'static>>,
    mut sensor7: AdcPin<peripherals::GPIO4<'static>, peripherals::ADC1<'static>>,
    mut sensor8: AdcPin<peripherals::GPIO7<'static>, peripherals::ADC1<'static>>,

    mut emiter: Output<'static>,

     left_motor_forward: channel::Channel<'static, LowSpeed>,
     left_motor_backward: channel::Channel<'static, LowSpeed>,
     right_motor_forward: channel::Channel<'static, LowSpeed>,
     right_motor_backward: channel::Channel<'static, LowSpeed>,

    mut control_rcv: Receiver<'static, CriticalSectionRawMutex, Control, 2>,
    sensor_comm: Sender<'static, CriticalSectionRawMutex, [u16; SENSOR_COUNT], 2>,
) {
    let mut calibration = [(0, 4096); SENSOR_COUNT];
    let mut mins = [0; SENSOR_COUNT];
    let mut maxs = [4096; SENSOR_COUNT];

    let kp = PID_KP;
    let ki = PID_KI;
    let kd = PID_KD;
    let mut last_error = 0.;
    let base_speed = BASE_SPEED;
    let max_speed = MAX_SPEED;
    let turn_speed = TURN_SPEED;

    let mut last_seen = 0;
    let mut lost = false;
    let mut lost_sen = 0;
    let mut last_detection_time = Instant::now();
    let line_th = LINE_THRESHOLD;

    info!("Starting calibration");
    for j in 0..CALIBRATION_STEPS {
        emiter.set_low();
        Timer::after(Duration::from_millis(CALIBRATION_DELAY_MS)).await;
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
        Timer::after(Duration::from_millis(CALIBRATION_DELAY_MS)).await;
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
    info!("Calibrated: {:?}", calibration);

    let mut position = 0;

    loop {
        let control = control_rcv.try_changed();
        if let Some(control) = control {
            if control.set_led {
                //led.set_high();
            } else {
                //led.set_low();
            }
        }

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
        let _total_time = Instant::now().checked_duration_since(s_time).unwrap();

        let normalized = normalize(&calibration, &pin_val);
        position = get_position(&normalized, position);

        sensor_comm.send(normalized);

        let error = (position as i32 - (SENSOR_COUNT as i32 - 1) * 1000 / 2) as f32;
        let mut motor_speed = (kp * error + kd * (error - last_error)) as i32;
        if motor_speed > 100 {
            motor_speed = 100;
        } else if motor_speed < -100 {
            // Dodano zabezpieczenie dla ujemnych wartości
            motor_speed = -100;
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

        if pin_val[0] > line_th && *pin_val.last().unwrap() < line_th {
            if last_seen != 1 && delta_time.as_millis() >= 100 {
                last_seen = 1; // Widziano linię po lewej
                last_detection_time = c_time;
            }
        }
        if pin_val[0] < line_th && *pin_val.last().unwrap() > line_th {
            if last_seen != 2 && delta_time.as_millis() >= 100 {
                last_seen = 2; // Widziano linię po prawej
                last_detection_time = c_time;
            }
        }

        if pin_val.iter().all(|pin| *pin < line_th) {
            lost = true;
        } else {
            lost = false;
        }

        if lost && last_seen == 1 {
            left_motor_forward.set_duty(0).expect("Failed to set PWM");
            left_motor_backward
                .set_duty(turn_speed)
                .expect("Failed to set PWM");
            right_motor_forward
                .set_duty(turn_speed)
                .expect("Failed to set PWM");
            right_motor_backward.set_duty(0).expect("Failed to set PWM");
        } else if lost && last_seen == 2 {
            left_motor_forward
                .set_duty(turn_speed)
                .expect("Failed to set PWM");
            left_motor_backward.set_duty(0).expect("Failed to set PWM");
            right_motor_forward.set_duty(0).expect("Failed to set PWM");
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

        emiter.set_high();
        Timer::after(Duration::from_millis(MAIN_LOOP_DELAY_MS)).await;
    }
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
            if *el > SENSOR_LINE_SEEN_THRESHOLD {
                line_seen = true;
            }
            if *el > SENSOR_WEIGHT_THRESHOLD {
                avg += *el as u32 * i as u32 * 1000;
                sum += *el;
            }
            (avg, sum)
        });

    if !line_seen {
        if last_position < (MAX_POSITION / 2) {
            return 0;
        } else {
            return MAX_POSITION;
        }
    }

    if sum == 0 {
        return last_position;
    }

    (avg / sum as u32) as u16
}

#[embassy_executor::task]
pub async fn read_sensors() {
    // Variables
    let mut sensor_rcv = SENSOR_WATCH.receiver().unwrap();
    let control_comm = CONTROL_WATCH.sender();
    let control = Control::default();

    loop {
        let _pin_val = sensor_rcv.changed().await;

        control_comm.send(control);
    }
}
