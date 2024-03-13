use defmt::*;
use embedded_hal::spi::SpiDevice;

// LIS3DSH accelerometer is connected to SPI1:
//
// Max bux speed is 10 MHz
// SCK on PA5 AF5
// MISO in PA6 AF5
// MOSI on PA7 AF5
// CS (I2C vs. SPI) on PE3
// CPOL = 1 (clock is idle high)
// CPHA = 1 (read on trailing (rising) edge of clock)

pub const MAXIMUM_SPI_FREQUENCY_HZ: u32 = 10_000_000;

pub struct Lis3dsh<SPI> {
    spi: SPI,
}

impl<SPI> Lis3dsh<SPI>
where
    SPI: SpiDevice,
{
    const WHO_AM_I: u8 = 0b0011_1111;

    const fn make_address_byte(register_address: u8, read: bool) -> u8 {
        (register_address & 0b0111_1111) | ((read as u8) << 7)
    }

    pub fn new(spi: SPI) -> Self {
        Lis3dsh { spi }
    }

    pub fn init(&mut self) -> Result<(), Error<SPI::Error>> {
        let id_byte = self.read_register_u8(Register::WHO_AM_I)?;

        if id_byte != Self::WHO_AM_I {
            return Err(Error::WhoAmIMismatch);
        }

        // Enable XYZ axes, set output data rate to 100 Hz.
        self.write_register_u8(Register::CTRL_REG4, 0b0110_1111)?;

        // Set FSCALE to +/- 4g.
        self.write_register_u8(Register::CTRL_REG5, 0b0000_1000)?;

        Ok(())
    }

    pub fn read_raw_accel(&mut self) -> Result<RawAccel, Error<SPI::Error>> {
        let mut buf = [
            Self::make_address_byte(0x28, true),
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        ];

        self.spi.transfer_in_place(&mut buf)?;

        Ok(RawAccel {
            x: (buf[1] as i16) | ((buf[2] as i16) << 8),
            y: (buf[3] as i16) | ((buf[4] as i16) << 8),
            z: (buf[5] as i16) | ((buf[6] as i16) << 8),
        })
    }

    fn read_register_u8(&mut self, register: Register) -> Result<u8, Error<SPI::Error>> {
        let register = Self::make_address_byte(register as u8, true);
        let mut buf = [register, 0];

        self.spi.transfer_in_place(&mut buf)?;

        Ok(buf[1])
    }

    fn write_register_u8(
        &mut self,
        register: Register,
        value: u8,
    ) -> Result<(), Error<SPI::Error>> {
        let register = Self::make_address_byte(register as u8, false);

        self.spi.write(&[register, value])?;

        Ok(())
    }
}

#[allow(non_camel_case_types, clippy::upper_case_acronyms, dead_code)]
#[derive(Copy, Clone, Debug, defmt::Format)]
pub enum Register {
    /// Temperature output - r
    UT_T = 0x0C,
    /// Information register 1 - r
    INFO1 = 0x0D,
    /// Information register 2 - r
    INFO2 = 0x0E,
    /// Who I am ID - r
    WHO_AM_I = 0x0F,
    /// X-axis offset correction - r/w
    OFF_X = 0x10,
    /// Y-axis offset correction - r/w
    OFF_Y = 0x11,
    /// Z-axis offset correction - r/w
    OFF_Z = 0x12,
    /// Constant shift X - r/w
    CS_X = 0x13,
    /// Constant shift Y - r/w
    CS_Y = 0x14,
    /// Constant shift Z - r/w
    CS_Z = 0x15,
    /// Long counter register - r/w
    LC_L = 0x16,
    /// Long counter register - r/w
    LC_H = 0x17,
    /// Interrupt synchronization - r
    STAT = 0x18,
    /// Peak value - r
    PEAK1 = 0x19,
    /// Peak value - r
    PEAK2 = 0x1A,
    /// Vector filter coefficient 1 - r/w
    VFC_1 = 0x1B,
    /// Vector filter coefficient 2 - r/w
    VFC_2 = 0x1C,
    /// Vector filter coefficient 3 - r/w
    VFC_3 = 0x1D,
    /// Vector filter coefficient 4 - r/w
    VFC_4 = 0x1E,
    /// Threshold value 3 - r/w
    THRS3 = 0x1F,
    /// Control register - r/w
    CTRL_REG4 = 0x20,
    /// SM1 control register - r/w
    CTRL_REG1 = 0x21,
    /// SM2 control register - r/w
    CTRL_REG2 = 0x22,
    /// Control register - r/w
    CTRL_REG3 = 0x23,
    /// Control register - r/w
    CTRL_REG5 = 0x24,
    /// Control register - r/w
    CTRL_REG6 = 0x25,
    /// Status data register - r
    STATUS = 0x27,
    /// Output register - r
    OUT_X_L = 0x28,
    /// Output register - r
    OUT_X_H = 0x29,
    /// Output register - r
    OUT_Y_L = 0x2A,
    /// Output register - r
    OUT_Y_H = 0x2B,
    /// Output register - r
    OUT_Z_L = 0x2C,
    /// Output register - r
    OUT_Z_H = 0x2D,
    /// FIFO register - r/w
    FIFO_CTRL = 0x2E,
    /// FIFO register - r
    FIFO_SRC = 0x2F,
    /// SM1 code register (X =1-16) - w
    ST1_1 = 0x40,
    ST1_2 = 0x41,
    ST1_3 = 0x42,
    ST1_4 = 0x43,
    ST1_5 = 0x44,
    ST1_6 = 0x45,
    ST1_7 = 0x46,
    ST1_8 = 0x47,
    ST1_9 = 0x48,
    ST1_10 = 0x49,
    ST1_11 = 0x4A,
    ST1_12 = 0x4B,
    ST1_13 = 0x4C,
    ST1_14 = 0x4D,
    ST1_15 = 0x4E,
    ST1_16 = 0x4F,
    /// SM1 general timer - w
    TIM4_1 = 0x50,
    /// w SM1 general timer
    TIM3_1 = 0x51,
    /// SM1 general timer (length 2) - w
    TIM2_1 = 0x52,
    /// SM1 general timer (length 2) - w
    TIM1_1 = 0x54,
    /// w SM1 threshold value 1
    THRS2_1 = 0x56,
    /// SM1 threshold value 2 - w
    THRS1_1 = 0x57,
    /// SM1 axis and sign mask - w
    MASK1_B = 0x59,
    /// w SM1 axis and sign mask
    MASK1_A = 0x5A,
    /// 01011011 - SM1 detection settings - w
    SETT1 = 0x5B,
    /// Program-reset pointer - r
    PR1 = 0x5C,
    /// r Timer counter (length 2)
    TC1 = 0x5D,
    /// - Main set flag - r
    OUTS1 = 0x5F,
    /// SM2 code register (X =1-16) - w
    ST2_1 = 0x60,
    ST2_2 = 0x61,
    ST2_3 = 0x62,
    ST2_4 = 0x63,
    ST2_5 = 0x64,
    ST2_6 = 0x65,
    ST2_7 = 0x66,
    ST2_8 = 0x67,
    ST2_9 = 0x68,
    ST2_10 = 0x69,
    ST2_11 = 0x6A,
    ST2_12 = 0x6B,
    ST2_13 = 0x6C,
    ST2_14 = 0x6D,
    ST2_15 = 0x6E,
    ST2_16 = 0x6F,
    /// w SM2 general timer
    TIM4_2 = 0x70,
    /// w 01110001 - SM2 general timer
    TIM3_2 = 0x71,
    /// SM2 general timer (length 2) - w
    TIM2_2 = 0x72,
    /// w SM2 general timer (length 2)
    TIM1_2 = 0x74,
    /// SM2 threshold value 1 - w
    THRS2_2 = 0x76,
    /// SM2 threshold value 2 - w
    THRS1_2 = 0x77,
    /// w Decimation factor
    DES2 = 0x78,
    /// SM2 axis and sign mask - w
    MASK2_B = 0x79,
    /// SM2 axis and sign mask - w
    MASK2_A = 0x7A,
    /// w SM2 detection settings
    SETT2 = 0x7B,
    /// Program-reset pointer - r
    PR2 = 0x7C,
    /// Timer counter (length 2) - r
    TC2 = 0x7D,
    /// r Main set flag
    OUTS2 = 0x7F,
}

#[derive(Copy, Clone, Debug, defmt::Format)]
pub struct RawAccel {
    x: i16,
    y: i16,
    z: i16,
}

#[derive(Copy, Clone, Debug, defmt::Format)]
pub enum Error<SPIError> {
    WhoAmIMismatch,
    Spi(SPIError),
}

impl<SPIError> From<SPIError> for Error<SPIError> {
    fn from(value: SPIError) -> Self {
        Self::Spi(value)
    }
}
