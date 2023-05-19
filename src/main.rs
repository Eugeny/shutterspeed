#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]
#![feature(asm_experimental_arch)]
#![feature(asm_const)]

// mod micros;
mod millis;

use embedded_hal::digital::v2::OutputPin;
// use micros::Timepiece;
use millis::{enable_interrupts, micros, millis, millis_init};
use sevensegment::SevenSeg;

use panic_halt as _;

// struct Timer {
//     clock: Chronometer<Atmega, Timepiece>,
// }

// impl Timer {
//     pub fn new(timer: arduino_hal::time::timers::Timer0) -> Self {
//         let clock = Chronometer::new(Timepiece::new(timer));
//         Self { clock }
//     }

//     pub fn micros(&self) -> u32 {
//         self.clock.micros()
//     }
// }

struct Display<P: OutputPin, const L: usize> {
    segment: SevenSeg<P, P, P, P, P, P, P>,
    select_pins: [P; L],
}

impl<P: OutputPin, const L: usize> Display<P, L> {
    fn new(data_pins: [P; 7], select_pins: [P; L]) -> Self {
        let [a, b, c, d, e, f, g] = data_pins;
        Self {
            segment: SevenSeg::new(a, b, c, d, e, f, g),
            select_pins,
        }
    }

    fn display_number(&mut self, number: u16, delay: u16) {
        let mut number = number;
        for position in 0..L {
            let digit = number % 10;
            self.display_in_position(
                L - 1 - position,
                if number > 0 || position == 0 {
                    digit as u8
                } else {
                    99
                },
            );
            number /= 10;
            arduino_hal::delay_ms(delay);
            self.segment.clear().unwrap();
        }
    }

    fn display_in_position(&mut self, position: usize, digit: u8) {
        for (index, pin) in self.select_pins.iter_mut().enumerate() {
            if index == position {
                let _ = pin.set_low();
            } else {
                let _ = pin.set_high();
            }
        }
        let _ = self.select_pins[position].set_low();
        self.segment.display(digit).unwrap();
    }

    fn display_progress(&mut self) {
        for position in 0..L {
            self.display_in_position(position, 99);
        }

        self.display_in_position((millis() / 200u32) as usize % L, 99);
        self.segment.seg_g(true).unwrap();
    }
}

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
        //     &&
        //  (value as u32) > self.latest_max() as u32 * (100 + self.threshold_percent) / 100
        {
            if self.current_burst_start < self.current_burst_end {
                self.current_burst_start = micros();
            }
        }
        if (value as u32) <= avg {
            if self.current_burst_start > self.current_burst_end {
                self.current_burst_end = micros();
            }
        }
    }

    fn is_in_active_burst(&self) -> bool {
        self.current_burst_start > self.current_burst_end
    }

    fn get_last_burst_duration(&self) -> Option<u32> {
        if self.current_burst_start < self.current_burst_end {
            Some((self.current_burst_end - self.current_burst_start) as u32)
        } else {
            None
        }
    }

    // fn get_last_burst_start_time(&self) -> Option<u32> {
    //     if self.current_burst_start <= self.current_burst_end {
    //         Some(self.current_burst_start)
    //     } else {
    //         None
    //     }
    // }
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());

    let mut display = Display::new(
        [
            pins.d2.into_output().downgrade(),
            pins.d3.into_output().downgrade(),
            pins.d4.into_output().downgrade(),
            pins.d5.into_output().downgrade(),
            pins.d6.into_output().downgrade(),
            pins.d7.into_output().downgrade(),
            pins.d8.into_output().downgrade(),
        ],
        [
            pins.d9.into_output().downgrade(),
            pins.d10.into_output().downgrade(),
            pins.d11.into_output().downgrade(),
            pins.d12.into_output().downgrade(),
        ],
    );

    millis_init(dp.TC1);
    enable_interrupts();

    let input = pins.a0.into_analog_input(&mut adc);
    let mut detector = Detector::new(2);

    loop {
        let mut value = input.analog_read(&mut adc);
        if value < 5 {
            value = 0;
        }
        detector.register_input(value);

        if !detector.is_in_active_burst() {
            if let Some(duration) = detector.get_last_burst_duration() {
                let ratio = 1000000f32 / duration as f32;
                display.display_number(ratio as u16, 0);
            }
        } else {
            display.display_progress();
            display.display_number(detector.filter.filter(value as f32) as u16, 1);
        }

        // display.display_number((millis() / 1000) as u16, 1);

        // let max = (value as u32 * 9999 / 2u32.pow(16)) as u16;
        // let max = micros();
        // display.display_number((max ) as u16, 1);
    }
}
