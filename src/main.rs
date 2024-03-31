#![no_std]
#![no_main]
use arduino_hal as hal;
use ufmt_float::uFmt_f32;
use arduino_hal::prelude::*;
use avr_device;

mod bmp180;
use bmp180::Bmp180;
use bmp180::Mode;

#[panic_handler] 
fn panic(info: &core::panic::PanicInfo) -> ! { 
    // disable interrupts - firmware has panicked so no ISRs should continue running 
    avr_device::interrupt::disable(); 
  
    // get the peripherals so we can access serial and the LED. 
    // 
    // SAFETY: Because main() already has references to the peripherals this is an unsafe 
    // operation - but because no other code can run after the panic handler was called, 
    // we know it is okay. 
    let dp = unsafe { arduino_hal::Peripherals::steal() }; 
    let pins = arduino_hal::pins!(dp); 
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600); 
 
    // Print out panic location 
    ufmt::uwriteln!(&mut serial, "Firmware panic!\r").unwrap_infallible(); 
    if let Some(loc) = info.location() { 
        ufmt::uwriteln!( 
            &mut serial, 
            "  At {}:{}:{}\r", 
            loc.file(), 
            loc.line(), 
            loc.column(), 
        ) 
        .unwrap_infallible(); 
    } 
  
    // Blink LED rapidly 
    let mut led = pins.d13.into_output(); 
    loop { 
        led.toggle(); 
        arduino_hal::delay_ms(100); 
    } 
} 

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
