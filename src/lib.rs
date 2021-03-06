//#![doc(html_root_url = "https://docs.rs/bme280")]
//#![doc(issue_tracker_base_url = "https://github.com/uber-foo/bme280/issues/")]
#![deny(
    missing_docs, missing_debug_implementations, missing_copy_implementations, trivial_casts,
    trivial_numeric_casts, unsafe_code, unstable_features, unused_import_braces,
    unused_qualifications, unused_variables, unreachable_code, unused_comparisons, unused_imports,
    unused_must_use
)]
#![no_std]

//! A platform agnostic Rust driver for TE's HTU21D temperature and humidity sensor, based on the
//! [`embedded-hal`](https://github.com/japaric/embedded-hal) traits.

extern crate embedded_hal;

use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

const HTU21D_I2C_ADDR: u8 = 0x40;

const HTU21D_REG_TEMP_HOLD: u8 = 0xE3;
const HTU21D_REG_RH_HOLD: u8 = 0xE5;
#[allow(dead_code)] const HTU21D_REG_TEMP_NOHOLD: u8 = 0xF3;
#[allow(dead_code)] const HTU21D_REG_RH_NOHOLD: u8 = 0xF5;
#[allow(dead_code)] const HTU21D_REG_WRITE_USER: u8 = 0xE6;
#[allow(dead_code)] const HTU21D_REG_READ_USER: u8 = 0xE7;
#[allow(dead_code)] const HTU21D_REG_SOFT_RESET: u8 = 0xFE;

const HTU21D_RH_DATA_LEN: usize = 3;
const HTU21D_T_DATA_LEN: usize = 3;

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
        self.soft_reset()?;
        self.heat()
    }

    fn soft_reset(&mut self) -> Result<(), Error<E>> {
        self.i2c
            .write(self.address, &[HTU21D_REG_SOFT_RESET])
            .map_err(Error::I2c)?;
        Ok(())
    }

    fn heat(&mut self) -> Result<(), Error<E>> {
        self.i2c
            .write(self.address, &[HTU21D_REG_WRITE_USER, 0x03])
            .map_err(Error::I2c)?;
            Ok(())
    }
    /// Captures and processes sensor data for temperature and relative humidity.
    /// Temperature data is used to correct the humidity data.
    pub fn measure(&mut self) -> Result<Measurements, Error<E>> {
        let h = self.read_humidity_reg()?;
        let t = self.read_temperature_reg()?;
        let result = parse(h, t)?;
        Ok(result)
    }

    fn read_temperature_reg(&mut self) -> Result<u16, Error<E>> {
        let mut data: [u8; HTU21D_T_DATA_LEN] = [0; HTU21D_T_DATA_LEN];
        self.i2c
            .write_read(self.address, &[HTU21D_REG_TEMP_HOLD], &mut data)
            .map_err(Error::I2c)?;
        Ok((data[0] as u16) << 8 | data[1] as u16)
    }

    fn read_humidity_reg(&mut self) -> Result<u16, Error<E>> {
        let mut data: [u8; HTU21D_RH_DATA_LEN] = [0; HTU21D_RH_DATA_LEN];
        self.i2c
            .write_read(self.address, &[HTU21D_REG_RH_HOLD], &mut data)
            .map_err(Error::I2c)?;
        Ok((data[0] as u16) << 8 | (data[1] as u16))
    }


}

fn parse<E>(humidity: u16, temperature: u16) -> Result<Measurements, Error<E>> {
    let h = -6.0 + 125.0 * ((humidity & 0xFFFC) as f32 / 65536.0);
    let t = -46.85 + 175.72 * ((temperature & 0xFFFC) as f32 / 65536.0);

    Ok( Measurements { temperature: t, humidity: h })
}

#[test]
/// Test parsing of temperature using examples given in the datasheet
fn test_parse_temp() {
    use assert_float_eq::*;
    let humidity: u16 = 0x683A;
    let temp: u16 = 0x4E85;
    let meas = parse::<()>(humidity, temp).unwrap();
    assert_float_absolute_eq!(meas.temperature, 7.04, 0.01);
}

#[test]
/// Test parsing of humidity using examples given in the datasheet
fn test_parse_rh() {
    use assert_float_eq::*;
    let humidity: u16 = 0x683A;
    let temp: u16 = 0x4E85;
    let meas = parse::<()>(humidity, temp).unwrap();
    assert_float_absolute_eq!(meas.humidity, 44.8, 0.1);
}
