//! A platform agnostic driver to iterface with the TLE5012 (GMR angle sensor)
//!
//! This driver wa built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/0.2
//!
//#![deny(missing_docs)]
#![no_std]

use embedded_hal::{
    blocking::spi::{Transfer, Write},
    digital::v2::OutputPin,
    spi::{Mode, Phase, Polarity}
};

/// SPI mode
pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

/// TLE5012 driver
pub struct Tle5012<SPI, CS> {
    spi: SPI,
    cs: CS,
}

impl<SPI, CS, E> Tle5012<SPI, CS>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    CS: OutputPin,
{
    /// Create a new driver from a SPI peripheral and a NCS pin
    pub fn new(spi: SPI, cs: CS) -> Result<Self, E> {
        Ok(Tle5012 { spi, cs })
    }

    /// Status of module
    pub fn read_status(&mut self) -> Result<u16, E> {
        self.read_register(ReadCmd::STA_CMD)
    }

    /// Updated status of module
    pub fn read_upd_status(&mut self) -> Result<u16, E> {
        self.read_register(ReadUpdCmd::STA_CMD)
    }

    fn read_register<T>(&mut self, reg: T) -> Result<u16, E>
    where
        T: Address
    {
        self.cs.set_low().ok();
        
        let cmd = reg.addr().to_be_bytes();
        let mut buffer:[u8; 6] = [cmd[0], cmd[1], 0, 0, 0, 0];

        self.spi.transfer(&mut buffer)?;
        
        self.cs.set_high().ok();

        let data:u16 = (buffer[2] as u16) << 8 + buffer[3] as u16;
        let safety:u16 = (buffer[4] as u16) << 8 + buffer[5] as u16;
        
        self.check_safety(safety, &cmd, data)
    }

    fn check_safety(&mut self, safety: u16, _cmd: &[u8], data: u16) -> Result<u16, E> {
        match safety {
            _ => Ok(data)
        }
    }
}

trait Address {
    fn addr(self) -> u16;
}

/// Commands for read
#[allow(dead_code)]
#[allow(non_camel_case_types)]
enum ReadCmd {
    STA_CMD_NOSAFETY = 0x8000,
    STA_CMD = 0x8001,
    ACTIV_STA_CMD = 0x8011,
    ANGLE_VAL_CMD = 0x8021,
    ANGLE_SPD_CMD = 0x8031,
    ANGLE_REV_CMD = 0x8041,
    TEMP_CMD = 0x8051,
    INTMODE_1 = 0x8061,
    SIL = 0x8071,
    INTMODE_2 = 0x8081,
    INTMODE_3 = 0x8091,
    OFFSET_X = 0x80A1,
    OFFSET_Y = 0x80B1,
    SYNCH = 0x80C1,
    IFAB = 0x80D1,
    INTMODE_4 = 0x80E1,
    TEMP_COEFF = 0x80F1,
    RAW_X_CMD = 0x8101,
    RAW_Y_CMD = 0x8111,
}

impl Address for ReadCmd {
    fn addr(self) -> u16 {
        self as u16
    }
}

/// Commands for update read
#[allow(dead_code)]
#[allow(non_camel_case_types)]
enum ReadUpdCmd {
    STA_CMD = 0x8401,
    ANGLE_VAL_CMD = 0x8421,
    ANGLE_SPD_CMD = 0x8431,
    ANGLE_REV_CMD = 0x8441,
}

impl Address for ReadUpdCmd {
    fn addr(self) -> u16 {
        self as u16
    }
}

/// Commands for write
#[allow(dead_code)]
#[allow(non_camel_case_types)]
enum WriteCmd {
    ACTIV_STA = 0x0011,
    INTMODE_1 = 0x5061,
    SIL = 0x5071,
    INTMODE_2 = 0x5081,
    INTMODE_3 = 0x5091,
    OFFSET_X = 0x50A1,
    OFFSET_Y = 0x50B1,
    SYNCH = 0x50C1,
    IFAB = 0x50D1,
    INTMODE_4 = 0x50E1,
    TEMP_COEFF = 0x50F1,
}

impl Address for WriteCmd {
    fn addr(self) -> u16 {
        self as u16
    }
}
