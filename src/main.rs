#![no_std]
#![no_main]
use arduino_hal as hal;
use hal::prelude::*;
use ufmt_float::uFmt_f32;
use panic_halt as _;

/* Declaration of constants, according to the BMP180 Datasheet
   Digital pressure sensor - Bosch */
const BMP180_ADDR: u8 = 0x77;
const CONTROL_REGISTER: u8 = 0xF4;
const MSB_DATA: u8 = 0xF6;
const LSB_DATA: u8 = 0xF7;
const XLSB_DATA: u8 = 0xF8;
const CMD_READ_TEMPERATURE: u8 = 0x2E;
const CMD_READ_PRESSURE_0: u8 = 0x34;
const CAL_AC1_MSB: u8 = 0xAA;

/* Calibration data that should be read once at the beginning. */
#[allow(dead_code)]
struct CalibrationData {
    ac1: i16,
    ac2: i16,
    ac3: i16,
    ac4: u16,
    ac5: u16,
    ac6: u16,
    b1: i16,
    b2: i16,
    mb: i16,
    mc: i16,
    md: i16,
}

/* Function for reading calibration data. Necessary for temperature and pressure calculation.*/
fn i2c_load_calibration_data(i2c: &mut hal::I2c) -> CalibrationData {
        let mut data = [0; 22];

        let _ = i2c.write_read(BMP180_ADDR, &[CAL_AC1_MSB], &mut data);

        CalibrationData{
            ac1: i16::from_be_bytes([data[0], data[1]]),
            ac2: i16::from_be_bytes([data[2], data[3]]),
            ac3: i16::from_be_bytes([data[4], data[5]]),
            ac4: u16::from_be_bytes([data[6], data[7]]),
            ac5: u16::from_be_bytes([data[8], data[9]]),
            ac6: u16::from_be_bytes([data[10], data[11]]),
            b1: i16::from_be_bytes([data[12], data[13]]),
            b2: i16::from_be_bytes([data[14], data[15]]),
            mb: i16::from_be_bytes([data[16], data[17]]),
            mc: i16::from_be_bytes([data[18], data[19]]),
            md: i16::from_be_bytes([data[20], data[21]]),
        }
}

/* "Calculation of the variable b5 used for temperature and pressure. */
fn calculate_b5(ut: i32, cd: &CalibrationData) -> i32 {
    let x1 = (ut - cd.ac6 as i32) * (cd.ac5 as i32) >> 15;
    let x2 = (cd.mc as i32) * 2048 / (x1 + (cd.md as i32));
    x1 + x2
}

/* ************************************************************ 
        Routines for temperature calculation.
   ************************************************************/

/* Activates the sensor for temperature return and reads it in RAW format */
fn i2c_read_raw_temperature(i2c: &mut hal::I2c) -> i32 {
    let mut data = [0; 2];

    let _ = i2c.write(BMP180_ADDR, &[CONTROL_REGISTER, CMD_READ_TEMPERATURE]);
    arduino_hal::delay_ms(5);
    let _ = i2c.write_read(BMP180_ADDR, &[MSB_DATA, LSB_DATA], &mut data);
    (data[0] as i32) << 8 | data[1] as i32
}

/* Determines the temperature in Celsius" */
fn read_temperature(i2c: &mut hal::I2c, cd: &CalibrationData) -> f32 {
    let ut = i2c_read_raw_temperature(i2c);
    let b5 = calculate_b5(ut, cd);
    let temperature = (b5 + 8) >> 4 ;
    //Divide por 10 para retornar uma casa decimal.
    (temperature as f32) / 10.0 
}


/* ************************************************************ 
        Routines for pressure calculation.
   ************************************************************/

/* Reads the registers with pressure information in RAW format. */
fn i2c_read_raw_pressure(i2c: &mut hal::I2c, oss: u32) -> i32 {
    let mut data = [0; 3];

    let cmd = CMD_READ_PRESSURE_0 | ((oss as u8) << 6);

    let _ = i2c.write(BMP180_ADDR, &[CONTROL_REGISTER, cmd]);

    arduino_hal::delay_ms(
        match oss {
            0 => 5,
            1 => 8,
            2 => 14,
            3 => 26,
            _ => 0,
        }
    );

    /* FIXME:   Read MSB_DATA and LSB_DATA at once. 
                Do not try to read XLSB with MSB and LSB together.
                It suports only 8 or 16 bits i2c transactions.     
    */
    let _ = i2c.write_read(BMP180_ADDR, &[MSB_DATA], &mut data);
    let msb = data[0];

    let _ = i2c.write_read(BMP180_ADDR, &[LSB_DATA], &mut data);
    let lsb = data[0];

    let _ = i2c.write_read(BMP180_ADDR, &[XLSB_DATA], &mut data);
    let xlsb = data[0];

    let raw_pressure: u32 = ((msb as u32) << 16) | ((lsb as u32) << 8) | (xlsb as u32);
    (raw_pressure >> (8 - oss)) as i32
}


/* Read raw data and performs calculations for pressure. */
fn i2c_read_pressure(i2c: &mut hal::I2c, oss: u32, cal: &CalibrationData) -> i32 {

    let up = i2c_read_raw_pressure(i2c, oss);

    let ut = i2c_read_raw_temperature(i2c);

    let b5 = calculate_b5(ut, &cal);
    let b6 = b5 - 4000;
    let x1 = ((cal.b2 as i32) * (b6 * b6 >> 12)) >> 11;
    let x2 = (cal.ac2 as i32) * b6 >> 11;
    let x3 = x1 + x2;
    let b3 = (((cal.ac1 as i32 * 4 + x3) << oss) + 2) >> 2;
    let x1 = (cal.ac3 as i32) * b6 >> 13;
    let x2 = (cal.b1 as i32) * ( b6 * b6 >> 12) >> 16;
    let x3 = ((x1 + x2) + 2) >> 2;
    let b4 = (cal.ac4 as u32) * (x3 + 32768) as u32 >> 15;

    /* Cuidado: Essa operação resulta em um inteiro de 64 bits.  */
    let b7 = (up - b3) as u64 * (50000 as u64 >> oss);
    
    let p;
    if b7 < 0x80000000 {
        p = (b7 * 2) / b4 as u64 ;
    } else {
        p = (b7 / b4 as u64) * 2;
    }

    let x1 = (p >> 8) * (p >> 8);
    let x1 = (x1 * 3038) >> 16;
    let x2 = (-7357 * p as i64) >> 16;
    let p = p as i64 + ((x1 as i64 + x2 + 3791) >> 4);
    p as i32
}


#[arduino_hal::entry]
fn main() -> ! {
    let dp = hal::Peripherals::take().unwrap();
    let pins = hal::pins!(dp);
    let mut serial = hal::default_serial!(dp, pins, 57600);
    let oss = 3;

    ufmt::uwriteln!(&mut serial, "Hello from Arduino I2C BMP180 Driver!\r").unwrap();

    let mut i2c = hal::I2c::new(
        dp.TWI,
        pins.d20.into_pull_up_input(),
        pins.d21.into_pull_up_input(),
        100000,
    );

   let cal = i2c_load_calibration_data(&mut i2c);

  loop { 
    let tmp = read_temperature(&mut i2c, &cal);
    ufmt::uwriteln!( &mut serial, "tmp = {} ", uFmt_f32::Two(tmp)).unwrap();
    
    let pressure = i2c_read_pressure(&mut i2c, oss, &cal);
    ufmt::uwriteln!( &mut serial, "pressure = {} ", pressure).unwrap();

    arduino_hal::delay_ms(1000);
  }
    

}

