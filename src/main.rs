#![no_std]
#![no_main]
use arduino_hal as hal;
use ufmt_float::uFmt_f32;
use panic_halt as _;

mod bmp180;
use bmp180::Bmp180;
use bmp180::Mode;

#[arduino_hal::entry]
fn main() -> ! {
    let mode = Mode::Bmp085UltraHighRes;

    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = hal::default_serial!(dp, pins, 57600);

    let i2c = hal::I2c::new(
        dp.TWI,
        pins.d20.into_pull_up_input(),
        pins.d21.into_pull_up_input(),
        100000,
    );

    let mut bmp180 = Bmp180::new(i2c);

    if let Err(_e) = bmp180.begin(mode) {
        ufmt::uwriteln!(&mut serial, "Error communicating with the BMP180 sensor.").unwrap();
    }

    loop {
        match bmp180.read_temperature() {
            Ok(temperature) => {
                ufmt::uwriteln!(&mut serial, "Tempature: {}", uFmt_f32::Two(temperature)).unwrap();
            }
            Err(_e) => {
                ufmt::uwriteln!(&mut serial, "Error communicating with the BMP180 sensor.").unwrap();
            }
        }
        
        match bmp180.read_pressure() {
            Ok(pressure) => {
                ufmt::uwriteln!(&mut serial, "Pressure {}", pressure).unwrap();
            }
            Err(_e) => {
                ufmt::uwriteln!(&mut serial, "Error communicating with the BMP180 sensor.").unwrap();
            }
        }

        arduino_hal::delay_ms(1000);
    }
}
