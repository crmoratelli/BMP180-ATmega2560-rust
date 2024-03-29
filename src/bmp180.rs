use embedded_hal::blocking::i2c::{Write, WriteRead};

/* Declaration of constants, according to the 
   BMP180 Datasheet Digital pressure sensor - Bosch */
const BMP180_ADDR: u8 = 0x77;
const CONTROL_REGISTER: u8 = 0xF4;
const MSB_DATA: u8 = 0xF6;
const LSB_DATA: u8 = 0xF7;
const XLSB_DATA: u8 = 0xF8;
const CMD_READ_TEMPERATURE: u8 = 0x2E;
const CMD_READ_PRESSURE_0: u8 = 0x34;
const CAL_AC1_MSB: u8 = 0xAA;

/* Operation modes */
pub enum Mode {
    #[allow(dead_code)]
    Bmp085UltraLowPower,
    #[allow(dead_code)]
    Bmp085Standard,
    #[allow(dead_code)]
    Bmp085HighRes,
    #[allow(dead_code)]
    Bmp085UltraHighRes,
}

pub enum Error {
    CommunicationError,
}

pub struct Bmp180<I2C> {
    i2c: I2C,
    oss: i32,
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

impl<I2C> Bmp180<I2C> 
where 
    I2C: Write + WriteRead,
{
    /* New instance of Bmp180 device. */
    pub fn new(i2c: I2C) -> Self {
        Bmp180 {i2c, 
                oss: 0,
                ac1: 0, 
                ac2: 0, 
                ac3: 0, 
                ac4: 0, 
                ac5: 0, 
                ac6: 0, 
                b1: 0, 
                b2: 0, 
                mb: 0, 
                mc: 0, 
                md: 0
        }
    }

    /* Drivers initialization. Must be called on device initialization. 
       Load the calibration data, necessary for temperature and pressure calculation.*/
    pub fn begin(& mut self, mode: Mode) -> Result<(), Error>{
        let mut data = [0; 22];

        self.i2c.
            write_read(BMP180_ADDR, &[CAL_AC1_MSB], &mut data)
            .map_err(|_| Error::CommunicationError)?;

        self.oss = match mode {
            Mode::Bmp085UltraLowPower => 0,
            Mode::Bmp085Standard => 1,
            Mode::Bmp085HighRes => 2,
            Mode::Bmp085UltraHighRes => 3,
        };

        self.ac1 =  i16::from_be_bytes([data[0], data[1]]);
        self.ac2 = i16::from_be_bytes([data[2], data[3]]);
        self.ac3 = i16::from_be_bytes([data[4], data[5]]);
        self.ac4 = u16::from_be_bytes([data[6], data[7]]);
        self.ac5 = u16::from_be_bytes([data[8], data[9]]);
        self.ac6 = u16::from_be_bytes([data[10], data[11]]);
        self.b1 = i16::from_be_bytes([data[12], data[13]]);
        self.b2 = i16::from_be_bytes([data[14], data[15]]);
        self.mb = i16::from_be_bytes([data[16], data[17]]);
        self.mc = i16::from_be_bytes([data[18], data[19]]);
        self.md = i16::from_be_bytes([data[20], data[21]]);

        Ok(())
    }

    /* "Calculation of the variable b5 used for temperature and pressure. */
    fn calculate_b5(&mut self, ut: i32) -> i32 {
        let x1 = (ut - self.ac6 as i32) * (self.ac5 as i32) >> 15;
        let x2 = (self.mc as i32) * 2048 / (x1 + (self.md as i32));
        x1 + x2
    }

    /************************************************************* 
        Routines for temperature calculation.
    **************************************************************/

    /* Activates the sensor for temperature return and reads it in RAW format */
    fn i2c_read_raw_temperature(&mut self) -> Result<i32, Error> {
        let mut data = [0; 2];

        // write the command to sample raw temperature.
        self.i2c.
            write(BMP180_ADDR, &[CONTROL_REGISTER, CMD_READ_TEMPERATURE])
            .map_err(|_| Error::CommunicationError)?;

        arduino_hal::delay_ms(5);

        // read the raw temperature data.
        self.i2c
            .write_read(BMP180_ADDR, &[MSB_DATA, LSB_DATA], &mut data)
            .map_err(|_| Error::CommunicationError)?;

        Ok((data[0] as i32) << 8 | data[1] as i32)
    }

    /* Determines the temperature in Celsius. */
    pub fn read_temperature(& mut self) -> Result<f32, Error> {
        let ut: i32;

        match self.i2c_read_raw_temperature() {
            Ok(value) => ut = value,
            Err(_) => return Err(Error::CommunicationError),
        }

        let b5 = self.calculate_b5(ut);

        let temperature = (b5 + 8) >> 4;

        //float division to obtain the temperature in Celsius with one decimal place.
        Ok((temperature as f32) / 10.0)
    }

    /************************************************************* 
        Routines for pressure calculation.
    **************************************************************/

    /* Reads the registers with pressure information in RAW format. */
    fn i2c_read_raw_pressure(&mut self) -> Result<i32, Error> {
        let mut data = [0; 2];

        let cmd = CMD_READ_PRESSURE_0 | ((self.oss as u8) << 6);

        // write the command to sample raw pressure.
        self.i2c
            .write(BMP180_ADDR, &[CONTROL_REGISTER, cmd])
            .map_err(|_| Error::CommunicationError)?;

        arduino_hal::delay_ms(
            match self.oss {
                0 => 5,
                1 => 8,
                2 => 14,
                3 => 26,
                _ => 0,
            }
        );

        // read the raw pressure data.
        self.i2c
            .write_read(BMP180_ADDR, &[MSB_DATA], &mut data)
            .map_err(|_| Error::CommunicationError)?;    
            
        let raw_pressure = (data[0] as i32) << 8 | data[1] as i32;

        // read additional precision bits for pressure calculation.
        if self.oss > 0{
            self.i2c
                .write_read(BMP180_ADDR, &[XLSB_DATA], &mut data)
                .map_err(|_| Error::CommunicationError)?;
        }

        let raw_pressure = (raw_pressure << 8) | data[0] as i32;
    
        Ok((raw_pressure >> (8 - self.oss)) as i32)
    }


    /* Read raw data and performs calculations for pressure. */
    pub fn read_pressure(&mut self) -> Result<i32, Error> {
        let ut: i32;
        let up: i32;
        
        match self.i2c_read_raw_pressure() {
            Ok(value) => up = value,
            Err(_) => return Err(Error::CommunicationError),
        }

        match self.i2c_read_raw_temperature() {
            Ok(value) => ut = value,
            Err(_) => return Err(Error::CommunicationError),
        }

        let b5 = self.calculate_b5(ut);
        let b6 = b5 - 4000;
        let x1 = ((self.b2 as i32) * (b6 * b6 >> 12)) >> 11;
        let x2 = (self.ac2 as i32) * b6 >> 11;
        let x3 = x1 + x2;
        let b3 = (((self.ac1 as i32 * 4 + x3) << self.oss) + 2) >> 2;
        let x1 = (self.ac3 as i32) * b6 >> 13;
        let x2 = (self.b1 as i32) * ( b6 * b6 >> 12) >> 16;
        let x3 = ((x1 + x2) + 2) >> 2;
        let b4 = (self.ac4 as u32) * (x3 + 32768) as u32 >> 15;

        /* Caution! This operation results in a 64-bit integer. */
        let b7 = (up - b3) as u64 * (50000 as u64 >> self.oss);
    
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
        Ok(p as i32)
    }

}
