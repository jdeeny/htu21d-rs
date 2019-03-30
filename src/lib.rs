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
//!
//! ## The Device
//!
//! The [Bosch BME280](https://www.bosch-sensortec.com/bst/products/all_products/bme280)
//! is a highly accurate sensor for atmospheric temperature and
//! relative humidity. The device has an I²C interface.
//!
//! ## Usage
//!
//! ```no_run
//! extern crate linux_embedded_hal as hal;
//! extern crate bme280;
//!
//! use hal::{Delay, I2cdev};
//! use bme280::BME280;
//!
//! // using Linux I2C Bus #1 in this example
//! let i2c_bus = I2cdev::new("/dev/i2c-1").unwrap();
//!
//! // initialize the BME280 using the primary I2C address 0x77
//! let mut bme280 = BME280::new_primary(i2c_bus, Delay);
//!
//! // or, initialize the BME280 using the secondary I2C address 0x78
//! // let mut bme280 = BME280::new_secondary(i2c_bus, Delay);
//!
//! // or, initialize the BME280 using a custom I2C address
//! // let bme280_i2c_addr = 0x88;
//! // let mut bme280 = BME280::new(i2c_bus, bme280_i2c_addr, Delay);
//!
//! // initialize the sensor
//! bme280.init().unwrap();
//!
//! // measure temperature, pressure, and humidity
//! let measurements = bme280.measure().unwrap();
//!
//! println!("Relative Humidity = {}%", measurements.humidity);
//! println!("Temperature = {} deg C", measurements.temperature);
//! println!("Pressure = {} pascals", measurements.pressure);
//! ```

extern crate embedded_hal;

use core::marker::PhantomData;
use embedded_hal::blocking::delay::DelayMs;
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

/// BME280 errors
#[derive(Debug)]
pub enum Error<E> {
    /// Failed to compensate a raw measurement
    CompensationFailed,
    /// I²C bus error
    I2c(E),
    /// Failed to parse sensor data
    InvalidData,
}

/// BME280 operating mode
#[derive(Debug, Copy, Clone)]
pub enum SensorMode {
    /// Sleep mode
    Sleep,
    /// Forced mode
    Forced,
    /// Normal mode
    Normal,
}

/// Measurement data
#[derive(Debug)]
pub struct Measurements<E> {
    /// temperature in degrees celsius
    pub temperature: f32,
    /// pressure in pascals
    pub pressure: f32,
    /// percent relative humidity (`0` with BMP280)
    pub humidity: f32,
    _e: PhantomData<E>,
}

impl<E> Measurements<E> {
    fn parse(
        data: [u8; BME280_P_T_H_DATA_LEN],
    ) -> Result<Self, Error<E>> {
        let data_msb: u32 = (data[0] as u32) << 12;
        let data_lsb: u32 = (data[1] as u32) << 4;
        let data_xlsb: u32 = (data[2] as u32) >> 4;
        let pressure = data_msb | data_lsb | data_xlsb;

        let data_msb: u32 = (data[3] as u32) << 12;
        let data_lsb: u32 = (data[4] as u32) << 4;
        let data_xlsb: u32 = (data[5] as u32) >> 4;
        let temperature = data_msb | data_lsb | data_xlsb;

        let data_msb: u32 = (data[6] as u32) << 8;
        let data_lsb: u32 = data[7] as u32;
        let humidity = data_msb | data_lsb;

        Ok(Measurements {
            temperature,
            humidity,
            _e: PhantomData,
        })
    }
}

/// Representation of a HTU21D
#[derive(Debug, Default)]
pub struct HTU21D<I2C, D> {
    /// concrete I²C device implementation
    i2c: I2C,
    /// I²C device address
    address: u8,
    /// concrete Delay implementation
    delay: D,
}

impl<I2C, D, E> HTU21D<I2C, D>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayMs<u8>,
{
    /// Create a new HTU21D struct using the default I²C address `0x40`
    pub fn new_primary(i2c: I2C, delay: D) -> Self {
        Self::new(i2c, HTU21D_I2C_ADDR, delay)
    }

    /// Create a new HTU21D struct using a custom I²C address
    pub fn new(i2c: I2C, address: u8, delay: D) -> Self {
        Self {
            i2c,
            address,
            delay,
        }
    }

    /// Initializes the HTU21D
    pub fn init(&mut self) -> Result<(), Error<E>> {
        self.soft_reset()
    }

    fn soft_reset(&mut self) -> Result<(), Error<E>> {
        self.write_register(HTU21D_RESET_ADDR, HTU21D_SOFT_RESET_CMD)?;
        self.delay.delay_ms(2); // startup time is 2ms
        Ok(())
    }

    /// Captures and processes sensor data for temperature and relative humidity.
    /// Temperature data is used to correct the humidity data.
    pub fn measure(&mut self) -> Result<Measurements<E>, Error<E>> {
        self.delay.delay_ms(40); // await measurement
        let measurements = self.read_data(BME280_DATA_ADDR)?;
        Ok(measurements)
    }

    fn read_register(&mut self, register: u8) -> Result<u8, Error<E>> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[register], &mut data)
            .map_err(Error::I2c)?;
        Ok(data[0])
    }

    fn read_data(&mut self, register: u8) -> Result<[u8; BME280_P_T_H_DATA_LEN], Error<E>> {
        let mut data: [u8; BME280_P_T_H_DATA_LEN] = [0; BME280_P_T_H_DATA_LEN];
        self.i2c
            .write_read(self.address, &[register], &mut data)
            .map_err(Error::I2c)?;
        Ok(data)
    }

    fn write_register(&mut self, register: u8, payload: u8) -> Result<(), Error<E>> {
        self.i2c
            .write(self.address, &[register, payload])
            .map_err(Error::I2c)
    }
}
