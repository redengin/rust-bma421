#![no_std]
#![no_main]
use panic_rtt_target as _;
use cortex_m_rt::entry;
use rtt_target::{rtt_init_print, rprintln};

use rust_bma4::{BMA421, I2C_Address};

// hardware target (CHOOSE ONE)
use nrf52832_hal::{self as hal};

use hal::pac::Peripherals;

#[entry]
fn main() -> ! {
    rtt_init_print!();

    // get the peripherals
    let p = Peripherals::take().unwrap();
    
    // Set up GPIO
    let gpio = hal::gpio::p0::Parts::new(p.P0);

    // Set up I2C
    let i2c_pins = hal::twim::Pins {
        scl: gpio.p0_07.into_floating_input().degrade(),
        sda: gpio.p0_06.into_floating_input().degrade(),
    };
    let i2c= hal::twim::Twim::new(p.TWIM0, i2c_pins, hal::twim::Frequency::K400);

    // Set up accelerometer
    let interrupt_pin1 = Some(gpio.p0_08.into_pullup_input());
    let mut bma = BMA421::new(
        i2c,
        I2C_Address::BMA456,
        interrupt_pin1,
        None).unwrap();
    // put the device into low power mode
    bma.low_power(None).unwrap();

    loop {
        rprintln!("accelerations {:?}", bma.accelerations().unwrap());
    }
}