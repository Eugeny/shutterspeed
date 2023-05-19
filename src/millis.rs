use core::arch::asm;
use core::cell;

const PRESCALER: u64 = 1;
const TIMER_TOP: u32 = 65535;

const CPU_FREQ: u64 = 16000000;
const TICK_NS: u64 = 1000000000 * PRESCALER / CPU_FREQ;

const TCNT1H: u8 = 0x85;
const TCNT1L: u8 = 0x84;

static NANOS_COUNTER: avr_device::interrupt::Mutex<cell::Cell<u64>> =
    avr_device::interrupt::Mutex::new(cell::Cell::new(0));

pub fn millis_init(tc1: arduino_hal::pac::TC1) {
    tc1.tccr1a.write(|w| w.wgm1().variant(0));
    tc1.tcnt1.write(|w| w.bits(0));
    tc1.tccr1b.write(|w| match PRESCALER {
        1 => w.cs1().direct(),
        8 => w.cs1().prescale_8(),
        64 => w.cs1().prescale_64(),
        256 => w.cs1().prescale_256(),
        1024 => w.cs1().prescale_1024(),
        _ => panic!(),
    });
    tc1.timsk1.write(|w| w.toie1().set_bit());

    // Reset the global millisecond counter
    avr_device::interrupt::free(|cs| {
        NANOS_COUNTER.borrow(cs).set(0);
    });
}

#[avr_device::interrupt(atmega32u4)]
fn TIMER1_OVF() {
    avr_device::interrupt::free(|cs| {
        let counter_cell = NANOS_COUNTER.borrow(cs);
        let counter = counter_cell.get();
        counter_cell.set(counter + 1);
    })
}

unsafe fn read_tcnt1() -> u16 {
    let vh: u8;
    let vl: u8;
    asm!(
        "
            lds {0}, {TCNT1L}
            lds {1}, {TCNT1H}
        ",
        out(reg) vl,
        out(reg) vh,
        TCNT1L = const TCNT1L,
        TCNT1H = const TCNT1H,
    );
    (vh as u16) * 256 + vl as u16
}

pub fn nanos() -> u64 {
    let ticks = avr_device::interrupt::free(|cs| {
        let counter = unsafe { read_tcnt1() };
        NANOS_COUNTER
            .borrow(cs)
            .get()
            .wrapping_mul(TIMER_TOP as u64)
            .wrapping_add(counter as u64)
    });
    ticks.wrapping_mul(TICK_NS as u64)
}

pub fn micros() -> u64 {
    nanos() / 1000
}

pub fn millis() -> u32 {
    (micros() / 1000) as u32
}

pub fn enable_interrupts() {
    unsafe { avr_device::interrupt::enable() };
}
