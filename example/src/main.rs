extern crate linux_embedded_hal as hal;
extern crate htu21d;

use hal::{Delay, I2cdev};
use htu21d::HTU21D;
use std::thread;
use std::time::Duration;

fn main() {
    let i2c_bus = I2cdev::new("/dev/i2c-2").unwrap();
    let mut htu21d = HTU21D::new_primary(i2c_bus, Delay);
    htu21d.init().unwrap();
    loop {
        let measurements = htu21d.measure().unwrap();
        println!("Relative Humidity = {}%", measurements.humidity);
        println!("Temperature = {} deg C", measurements.temperature);
        thread::sleep(Duration::from_secs(1));
    }
}
