#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]
#![feature(asm_experimental_arch)]
#![feature(asm_const)]

mod millis;

use core::fmt::Write;
use core::ops::{Deref, DerefMut};
use embedded_hal::blocking::delay::{DelayMs, DelayUs};

use heapless::String;
use lcd1602_driver::builder::{Builder, BuilderAPI};
use lcd1602_driver::enums::basic_command::{Font, LineMode, MoveDirection, ShiftType, State};
use lcd1602_driver::pins::FourPinsAPI;
use lcd1602_driver::{LCDBasic, LCDExt};
use millis::{disable_interrupts, enable_interrupts, millis, millis_init, nanos};

use panic_halt as _;
use ufmt::{uWrite, uwrite};

struct SignalFilter {
    f: f32,
    factor_millis: f32,
}

impl SignalFilter {
    fn new(factor_millis: f32) -> Self {
        Self {
            f: 0f32,
            factor_millis,
        }
    }

    fn filter(&mut self, value: f32) -> f32 {
        self.f += self.factor_millis * (value - self.f) / 1000f32;
        self.f
    }
}

struct Detector {
    current_burst_start: u64,
    current_burst_end: u64,
    threshold_percent: u32,
    filter: SignalFilter,
}

impl Detector {
    fn new(threshold_percent: u32) -> Self {
        Self {
            current_burst_start: 0,
            current_burst_end: 1,
            threshold_percent,
            filter: SignalFilter::new(1f32),
        }
    }

    fn register_input(&mut self, value: u16) {
        let avg = self.filter.filter(value as f32) as u32;

        if (value as u32) > avg * (100 + self.threshold_percent) / 100
            && self.current_burst_start < self.current_burst_end
        {
            self.current_burst_start = nanos();
        }
        if (value as u32) <= avg && self.current_burst_start > self.current_burst_end {
            self.current_burst_end = nanos();
        }
    }

    fn is_in_active_burst(&self) -> bool {
        self.current_burst_start > self.current_burst_end
    }

    fn get_last_burst_duration_us(&self) -> Option<f32> {
        if self.current_burst_start < self.current_burst_end {
            Some(((self.current_burst_end - self.current_burst_start) as f32) / 1000f32)
        } else {
            None
        }
    }
}

struct Delayer {}
impl DelayMs<u32> for Delayer {
    fn delay_ms(&mut self, ms: u32) {
        arduino_hal::delay_ms(ms as u16);
    }
}
impl DelayUs<u32> for Delayer {
    fn delay_us(&mut self, us: u32) {
        arduino_hal::delay_us(us);
    }
}

struct EString<const L: usize>(String<L>);

impl<const L: usize> Deref for EString<L> {
    type Target = String<L>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<const L: usize> DerefMut for EString<L> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<const L: usize> uWrite for EString<L> {
    type Error = core::fmt::Error;

    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.0.push_str(s).map_err(|_| core::fmt::Error)
    }

    fn write_char(&mut self, c: char) -> Result<(), Self::Error> {
        self.0.push(c).map_err(|_| core::fmt::Error)
    }
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());

    let rs_pin = pins.d3.into_output().downgrade();
    let rw_pin = pins.d4.into_output().downgrade();
    let en_pin = pins.d2.into_output().downgrade();

    let db4_pin = pins.d9.into_opendrain().downgrade();
    let db5_pin = pins.d10.into_opendrain().downgrade();
    let db6_pin = pins.d11.into_opendrain().downgrade();
    let db7_pin = pins.d12.into_opendrain().downgrade();

    // put pins together
    let lcd_pins =
        lcd1602_driver::pins::Pins::new(rs_pin, rw_pin, en_pin, db4_pin, db5_pin, db6_pin, db7_pin);

    // setup a builder
    let lcd_builder = Builder::new(lcd_pins, Delayer {})
        .set_blink(State::Off)
        .set_cursor(State::Off)
        .set_direction(MoveDirection::LeftToRight)
        .set_display(State::On)
        .set_font(Font::Font5x8)
        .set_line(LineMode::TwoLine)
        .set_shift(ShiftType::CursorOnly)
        .set_wait_interval_us(10);

    let mut lcd = lcd_builder.build_and_init();

    let mut s = EString(String::<32>::new());

    millis_init(dp.TC1);
    enable_interrupts();

    let input = pins.a0.into_analog_input(&mut adc);
    let reset_pin = pins.a5.into_pull_up_input();
    let mut wait_reset = |ms| {
        for _ in 0..ms {
            if reset_pin.is_low() {
                return true;
            }
            arduino_hal::delay_ms(1);
        }
        false
    };
    // let mut detector = Detector::new(20);

    'main: loop {
        disable_interrupts();
        lcd.clean_display();
        lcd.set_cursor_pos((1, 0));
        lcd.write_str_to_cur("Calibrating");
        lcd.set_cursor_blink_state(State::On);
        enable_interrupts();

        let mut avg = 0.0;
        let calibration_loops = 1000;
        for _ in 0..calibration_loops {
            let value = input.analog_read(&mut adc);
            avg += value as f32 / calibration_loops as f32;
            arduino_hal::delay_ms(1);
        }

        let threshold = 1.5;
        let expected_high = (avg * threshold).min(1022.0) as u16;

        disable_interrupts();
        lcd.clean_display();
        lcd.set_cursor_pos((1, 0));
        lcd.write_str_to_cur("* Measuring *");
        lcd.set_cursor_blink_state(State::On);
        lcd.set_cursor_pos((1, 1));
        s.clear();
        let _ = uwrite!(s, "A: {} T: {}", avg as u32, expected_high);
        lcd.write_str_to_cur(&s);
        enable_interrupts();

        let mut max: u16 = 0;
        let mut sample_ctr = 0;
        let mut sum: u64 = 0;
        let t_start: u64;
        let t_end;

        loop {
            let value = input.analog_read(&mut adc);
            if value > expected_high {
                t_start = nanos();
                break;
            }
            if reset_pin.is_low() {
                continue 'main;
            }
        }

        loop {
            let value = input.analog_read(&mut adc);
            if value < expected_high {
                t_end = nanos();
                break;
            }
            sum += value as u64;
            max = max.max(value);
            sample_ctr += 1;
            if reset_pin.is_low() {
                continue 'main;
            }
        }

        let integrated_min = avg as u64;
        let integrated = (sum - integrated_min as u64 * sample_ctr as u64) as f32 / sample_ctr as f32;
        let integrated_max = max as u64 - integrated_min;
        let integrated_normalized = integrated / integrated_max as f32;

        let duration_raw = t_end - t_start;
        let duration = (duration_raw as f32 * integrated_normalized) as u64;

        disable_interrupts();
        lcd.clean_display();
        lcd.set_cursor_blink_state(State::Off);
        lcd.set_cursor_pos((0, 0));

        {
            let fraction = 1000000000.0 / duration as f32;
            let int = fraction as u32;
            let fr = (fraction - int as f32) * 10.0;
            s.clear();
            let _ = uwrite!(s, "1/{}.{} | {}ms", int, fr as u32, duration / 1000000);
            lcd.write_str_to_cur(&s);
        }
        enable_interrupts();

        loop {
            let mut show_info = |label, value| {
                disable_interrupts();
                lcd.write_str_to_pos("                 ", (0, 1));
                s.clear();
                let _ = uwrite!(s, "{}: {}", label, value);
                lcd.write_str_to_pos(&s, (0, 1));
                enable_interrupts();
            };

            show_info("Int f", (integrated_normalized * 1000.0) as u32);

            if wait_reset(1000) {
                break;
            }

            show_info("Max", max as u32);

            if wait_reset(1000) {
                break;
            }

            show_info("Avg", avg as u32);

            if wait_reset(1000) {
                break;
            }

            show_info("Samples", sample_ctr);

            if wait_reset(1000) {
                break;
            }
        }
    }

    // {

    // if !detector.is_in_active_burst() {
    //     if let Some(duration) = detector.get_last_burst_duration_us() {
    //         if (millis() / 1000) % 2 == 0 {
    //             // display.display_number((duration / 1000f32) as u16, 0, 0);
    //         } else {
    //             let ratio = 1000000f32 / duration;
    //             // display.display_number(ratio as u16, 0, 99);
    //         }
    //     }
    // } else {
    //     // display.display_progress();
    //     // display.display_number(detector.filter.filter(value as f32) as u16, 1);
    // }

    // display.display_number((millis() / 1000) as u16, 1);

    // let max = (value as u32 * 9999 / 2u32.pow(16)) as u16;
    // let max = micros();
    // display.display_number((max ) as u16, 1);
    // }

    loop {}
}
