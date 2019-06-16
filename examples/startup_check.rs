//! Test the formated output
//!
//! This example hasn't a special requires
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_semihosting;

use cortex_m::asm;
use stm32f30x_hal::{prelude::*, serial::Serial, spi::Spi};
use serialio::{SerialIO, sprintln};
use cortex_m_rt::entry;

use f3::hal::stm32f30x;

use tle5012::{self, Tle5012};

#[entry]
fn main() -> ! {
    let p = stm32f30x::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioc = p.GPIOC.split(&mut rcc.ahb);
    let mut gpioa = p.GPIOA.split(&mut rcc.ahb);

    // Setup serial i/o
    let tx = gpioc.pc4.into_af7(&mut gpioc.moder, &mut gpioc.afrl);
    let rx = gpioc.pc5.into_af7(&mut gpioc.moder, &mut gpioc.afrl);

    let serial = Serial::usart1(p.USART1, (tx, rx), 115_200.bps(), clocks, &mut rcc.apb2);

    let (tx, rx) = serial.split();

    let mut in_out = SerialIO::new(tx, rx);

    // Setup spi i/o
    let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let mut nss = gpioa.pa4.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    nss.set_high();

    let spi = Spi::spi1(p.SPI1, (sck, miso, mosi), tle5012::MODE, 1.mhz(), clocks, &mut rcc.apb2);

    let mut angle_sensor = Tle5012::new(spi, nss).unwrap();

    let status = angle_sensor.read_status();
    
    sprintln!(in_out, "Angle sensor status is {:x}", status.unwrap());

    asm::bkpt();

    loop {}
}
