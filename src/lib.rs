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
    spi::{Mode, Phase, Polarity},
};

/// SPI mode
pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleLow,
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
    pub fn read_status(&mut self) -> Result<u16, Error<E>> {
        self.read_register(ReadCmd::STA_CMD)
    }

    /// Updated status of module
    pub fn read_upd_status(&mut self) -> Result<u16, Error<E>> {
        self.read_register(ReadUpdCmd::STA_CMD)
    }

    /// Activation status of module
    pub fn read_activation_status(&mut self) -> Result<u16, Error<E>> {
        self.read_register(ReadCmd::ACTIV_STA_CMD)
    }

    /// Read angle value. Sensor return 15 bit signed integer
    pub fn read_angle_value(&mut self) -> Result<i16, Error<E>> {
        let tmp = self.read_register(ReadCmd::ANGLE_VAL_CMD)?;

        Ok(self.from_i15_to_i16(tmp))
    }

    /// Read update angle value. Sensor return 15 bit signed integer
    pub fn read_update_angle_value(&mut self) -> Result<i16, Error<E>> {
        let tmp = self.read_register(ReadUpdCmd::ANGLE_VAL_CMD)?;

        Ok(self.from_i15_to_i16(tmp))
    }

    /// Read angle speed. Sensor return 15 bit signed integer
    pub fn read_angle_speed(&mut self) -> Result<i16, Error<E>> {
        let tmp = self.read_register(ReadCmd::ANGLE_SPD_CMD)?;

        Ok(self.from_i15_to_i16(tmp))
    }

    /// Read update angle speed. Sensor return 15 bit signed integer
    pub fn read_update_angle_speed(&mut self) -> Result<i16, Error<E>> {
        let tmp = self.read_register(ReadUpdCmd::ANGLE_SPD_CMD)?;

        Ok(self.from_i15_to_i16(tmp))
    }

    /// Read angle revolution. Sensor return 9 bit signed integer
    pub fn read_angle_revolution(&mut self) -> Result<i16, Error<E>> {
        let tmp = self.read_register(ReadCmd::ANGLE_REV_CMD)?;

        Ok(self.from_i9_to_i16(tmp))
    }

    /// Read update angle revolution. Sensor return 9 bit signed integer
    pub fn read_update_angle_revolution(&mut self) -> Result<i16, Error<E>> {
        let tmp = self.read_register(ReadUpdCmd::ANGLE_REV_CMD)?;

        Ok(self.from_i9_to_i16(tmp))
    }

    /// Read temp value. Sensor return 9 bit signed integer
    pub fn read_temp(&mut self) -> Result<i16, Error<E>> {
        let tmp = self.read_register(ReadCmd::TEMP_CMD)?;

        Ok(self.from_i9_to_i16(tmp))
    }

    /// Read raw X value.
    pub fn read_raw_x(&mut self) -> Result<i16, Error<E>> {
        let tmp = self.read_register(ReadCmd::RAW_X_CMD)?;

        Ok(tmp as i16)
    }

    /// Read raw Y value.
    pub fn read_raw_y(&mut self) -> Result<i16, Error<E>> {
        let tmp = self.read_register(ReadCmd::RAW_Y_CMD)?;

        Ok(tmp as i16)
    }

    pub fn read_int_mode_1(&mut self) -> Result<u16, Error<E>> {
        self.read_register(ReadCmd::INTMODE_1)
    }

    pub fn read_sil(&mut self) -> Result<u16, Error<E>> {
        self.read_register(ReadCmd::SIL)
    }

    pub fn read_int_mode_2(&mut self) -> Result<u16, Error<E>> {
        self.read_register(ReadCmd::INTMODE_2)
    }

    pub fn read_int_mode_3(&mut self) -> Result<u16, Error<E>> {
        self.read_register(ReadCmd::INTMODE_3)
    }

    pub fn read_offset_x(&mut self) -> Result<u16, Error<E>> {
        self.read_register(ReadCmd::OFFSET_X)
    }

    pub fn read_offset_y(&mut self) -> Result<u16, Error<E>> {
        self.read_register(ReadCmd::OFFSET_Y)
    }

    pub fn read_synch(&mut self) -> Result<u16, Error<E>> {
        self.read_register(ReadCmd::SYNCH)
    }

    pub fn read_ifab(&mut self) -> Result<u16, Error<E>> {
        self.read_register(ReadCmd::IFAB)
    }

    pub fn read_int_mode_4(&mut self) -> Result<u16, Error<E>> {
        self.read_register(ReadCmd::INTMODE_4)
    }

    pub fn read_temp_coeff(&mut self) -> Result<u16, Error<E>> {
        self.read_register(ReadCmd::TEMP_COEFF)
    }

    fn read_register<T>(&mut self, reg: T) -> Result<u16, Error<E>>
    where
        T: Address,
    {
        self.cs.set_low().ok();

        let cmd = reg.addr().to_be_bytes();
        let mut buffer: [u8; 2] = cmd;

        self.spi.transfer(&mut buffer).map_err(Error::Spi)?;

        self.spi.transfer(&mut buffer).map_err(Error::Spi)?;

        let data: u16 = u16::from_be_bytes(buffer);

        self.spi.transfer(&mut buffer).map_err(Error::Spi)?;

        let safety: u16 = u16::from_be_bytes(buffer);

        self.cs.set_high().ok();

        self.check_safety(safety, &cmd, data)
    }

    pub fn write_activation_status(&mut self, data: u16) -> Result<bool, Error<E>> {
        self.write_register(WriteCmd::ACTIV_STA, data)
    }

    pub fn write_int_mode_1(&mut self, data: u16) -> Result<bool, Error<E>> {
        self.write_register(WriteCmd::INTMODE_1, data)
    }

    pub fn write_sil(&mut self, data: u16) -> Result<bool, Error<E>> {
        self.write_register(WriteCmd::SIL, data)
    }

    pub fn write_int_mode_2(&mut self, data: u16) -> Result<bool, Error<E>> {
        self.write_register(WriteCmd::INTMODE_2, data)
    }

    pub fn write_int_mode_3(&mut self, data: u16) -> Result<bool, Error<E>> {
        self.write_register(WriteCmd::INTMODE_3, data)
    }

    pub fn write_offset_x(&mut self, data: u16) -> Result<bool, Error<E>> {
        self.write_register(WriteCmd::OFFSET_X, data)
    }

    pub fn write_offset_y(&mut self, data: u16) -> Result<bool, Error<E>> {
        self.write_register(WriteCmd::OFFSET_Y, data)
    }

    pub fn write_synch(&mut self, data: u16) -> Result<bool, Error<E>> {
        self.write_register(WriteCmd::SYNCH, data)
    }

    pub fn write_ifab(&mut self, data: u16) -> Result<bool, Error<E>> {
        self.write_register(WriteCmd::IFAB, data)
    }

    pub fn write_int_mode_4(&mut self, data: u16) -> Result<bool, Error<E>> {
        self.write_register(WriteCmd::INTMODE_4, data)
    }

    pub fn write_temp_coeff(&mut self, data: u16) -> Result<bool, Error<E>> {
        self.write_register(WriteCmd::TEMP_COEFF, data)
    }

    fn write_register<T>(&mut self, reg: T, data: u16) -> Result<bool, Error<E>>
    where
        T: Address,
    {
        self.cs.set_low().ok();

        let cmd = reg.addr().to_be_bytes();
        let mut buffer: [u8; 2] = cmd;

        self.spi.transfer(&mut buffer).map_err(Error::Spi)?;

        buffer = data.to_be_bytes();

        self.spi.transfer(&mut buffer).map_err(Error::Spi)?;

        self.spi.transfer(&mut buffer).map_err(Error::Spi)?;

        let safety: u16 = u16::from_be_bytes(buffer);

        self.cs.set_high().ok();

        match self.check_safety(safety, &cmd, data) {
            Ok(_) => Ok(true),
            Err(e) => Err(e),
        }
    }

    fn calc_crc(&mut self, buf: &[u8], crc_orig: u8) -> Result<(), Error<E>> {
        let crc = {
            let mut crc = CRC_SEED;
            for i in buf {
                crc ^= i;
                for _ in 0..8 {
                    if crc & 0x80 != 0 {
                        crc <<= 1;
                        crc ^= CRC_POLYNOMIAL;
                    } else {
                        crc <<= 1;
                    }
                }
            }

            !crc & CRC_SEED
        };

        if crc != crc_orig {
            Err(Error::Crc)
        } else {
            Ok(())
        }
    }

    fn check_safety(&mut self, safety: u16, cmd: &[u8], data: u16) -> Result<u16, Error<E>> {
        match safety {
            x if x & SYSTEM_ERROR_MASK == 0 => Err(Error::System),
            x if x & INTERFACE_ERROR_MASK == 0 => Err(Error::InterfaceAccess),
            x if x & INV_ANGLE_ERROR_MASK == 0 => Err(Error::InvalidAngle),
            _ => {
                let data_r = data.to_be_bytes();
                let safety = safety.to_be_bytes();
                let buffer = [cmd[0], cmd[1], data_r[0], data_r[1]];
                let crc = safety[1];

                match self.calc_crc(&buffer, crc) {
                    Err(e) => Err(e),
                    _ => Ok(data),
                }
            }
        }
    }

    fn from_i15_to_i16(&mut self, val: u16) -> i16 {
        let tmp = val & DELETE_BIT_15;

        (if tmp & CHECK_BIT_14 != 0 {
            tmp.wrapping_sub(CHANGE_UINT_TO_INT_15)
        } else {
            tmp
        } as i16)
    }

    fn from_i9_to_i16(&mut self, val: u16) -> i16 {
        let tmp = val & DELETE_7BITS;

        (if tmp & CHECK_BIT_9 != 0 {
            tmp.wrapping_sub(CHANGE_UNIT_TO_INT_9)
        } else {
            tmp
        } as i16)
    }
}

/// Crc constants
const CRC_SEED: u8 = 0xff;
const CRC_POLYNOMIAL: u8 = 0x1d;

/// Error masks
const SYSTEM_ERROR_MASK: u16 = 0x4000;
const INTERFACE_ERROR_MASK: u16 = 0x2000;
const INV_ANGLE_ERROR_MASK: u16 = 0x1000;

/// Values used to calculate 15 bit signed int sent by the sensor
const DELETE_BIT_15: u16 = 0x7FFF;
const CHANGE_UINT_TO_INT_15: u16 = 32768;
const CHECK_BIT_14: u16 = 0x4000;

//values used to calculate 9 bit signed int sent by the sensor
const DELETE_7BITS: u16 = 0x01FF;
const CHANGE_UNIT_TO_INT_9: u16 = 512;
const CHECK_BIT_9: u16 = 0x0100;

#[derive(Debug)]
pub enum Error<E> {
    /// General system error
    System,
    /// Interface errors
    InterfaceAccess,
    /// Wrong readed angle value
    InvalidAngle,
    /// Wrong CRC
    Crc,
    /// SPI bus error
    Spi(E),
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
