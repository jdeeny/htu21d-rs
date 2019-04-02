#![doc(html_root_url = "https://docs.rs/bme280")]
#![doc(issue_tracker_base_url = "https://github.com/uber-foo/bme280/issues/")]
#![deny(
    missing_docs, missing_debug_implementations, missing_copy_implementations, trivial_casts,
    trivial_numeric_casts, unsafe_code, unstable_features, unused_import_braces,
    unused_qualifications, unused_variables, unreachable_code, unused_comparisons, unused_imports,
    unused_must_use
)]
#![no_std]

//! A platform agnostic Rust driver for the Measurement Specialties HTU21D, based on the
//! [`embedded-hal`](https://github.com/japaric/embedded-hal) traits.

extern crate embedded_hal;

use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

const HTU21D_I2C_ADDR: u8 = 0x40;

const HTU21D_REG_TEMP_HOLD: u8 = 0xE3;
const HTU21D_REG_RH_HOLD: u8 = 0xE5;
const HTU21D_REG_TEMP_NOHOLD: u8 = 0xF3;
const HTU21D_REG_RH_NOHOLD: u8 = 0xF5;
const HTU21D_REG_WRITE_USER: u8 = 0xE6;
const HTU21D_REG_READ_USER: u8 = 0xE7;
const HTU21D_REG_SOFT_RESET: u8 = 0xFE;

const HTU21D_RH_DATA_LEN: usize = 3;
const HTU21D_T_DATA_LEN: usize = 3;

const HTU21D_SOFT_RESET_CMD: u8 = 0x00;

/*macro_rules! concat_bytes {
    ($msb:expr, $lsb:expr) => {
        (($msb as u16) << 8) | ($lsb as u16)
    };
}

macro_rules! set_bits {
    ($reg_data:expr, $mask:expr, $pos:expr, $data:expr) => {
        ($reg_data & !$mask) | (($data << $pos) & $mask)
    };
}*/

/// HTU21D errors
#[derive(Debug)]
pub enum Error<E> {
    /// I²C bus error
    I2c(E),
    /// Failed to parse sensor data
    InvalidData,
}

/// Measurement data
#[derive(Debug, Copy, Clone)]
pub struct Measurements {
    /// temperature in degrees celsius
    pub temperature: f32,
    /// percent relative humidity
    pub humidity: f32,
}

/// Representation of a HTU21D
#[derive(Debug, Default)]
pub struct HTU21D<I2C> {
    /// concrete I²C device implementation
    i2c: I2C,
    /// I²C device address
    address: u8,
}

impl<I2C, E> HTU21D<I2C>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
{
    /// Create a new HTU21D struct using the default I²C address `0x40`
    pub fn new_primary(i2c: I2C) -> Self {
        Self::new(i2c, HTU21D_I2C_ADDR)
    }

    /// Create a new HTU21D struct using a custom I²C address
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
        }
    }

    /// Initializes the HTU21D
    pub fn init(&mut self) -> Result<(), Error<E>> {
        self.soft_reset()
    }

    fn soft_reset(&mut self) -> Result<(), Error<E>> {
        //self.write_register(HTU21D_REG_SOFT_RESET, HTU21D_SOFT_RESET_CMD)?;
        Ok(())
    }

    /// Captures and processes sensor data for temperature and relative humidity.
    /// Temperature data is used to correct the humidity data.
    pub fn measure(&mut self) -> Result<Measurements, Error<E>> {
        let t = self.read_temperature_reg()?;
        let h = self.read_humidity_reg()?;
        let result = Self::parse(t, h)?;
        Ok(result)
    }

    fn read_temperature_reg(&mut self) -> Result<u16, Error<E>> {
        let mut data: [u8; HTU21D_T_DATA_LEN] = [0; HTU21D_T_DATA_LEN];
        self.i2c
            .write_read(self.address, &[HTU21D_REG_TEMP_HOLD], &mut data)
            .map_err(Error::I2c)?;
        Ok(0)
    }

    fn read_humidity_reg(&mut self) -> Result<u16, Error<E>> {
        Ok(0)
    }

    fn parse(_humidity: u16, _temperature: u16) -> Result<Measurements, Error<E>> {
        Ok( Measurements { temperature: 0.0, humidity: 0.0 })
    }

/*    fn read_register(&mut self, register: u8) -> Result<u8, Error> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[register], &mut data)
            .map_err(Error::I2c)?;
        Ok(data[0])
    }

    fn read_data(&mut self, register: u8) -> Result<u16, Error> {
        let mut data: [u8; BME280_P_T_H_DATA_LEN] = [0; BME280_P_T_H_DATA_LEN];
        self.i2c
            .write_read(self.address, &[register], &mut data)
            .map_err(Error::I2c)?;
        Ok(data)
    }

    fn write_register(&mut self, register: u8, payload: u8) -> Result<(), Error> {
        self.i2c
            .write(self.address, &[register, payload])
            .map_err(Error::I2c)
    }*/
}
