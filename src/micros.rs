use arduino_hal::impl_timepiece;

impl_timepiece! {
    pub timepiece Timepiece {
        peripheral: Timer0,
        cpu_clock: arduino_hal::DefaultClock,
        millis: u32,
        micros: u32,
        resolution: arduino_hal::time::Resolution::MS_1,
    }
}
