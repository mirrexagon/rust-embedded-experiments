use crate::fmt::info;
use defmt::*;

use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{AnyPin, Level, Output, Speed};
use embassy_stm32::peripherals;
use embassy_stm32::spi::{Config, Instance, Spi, MODE_3};
use embassy_stm32::time::Hertz;

// LIS3DSH accelerometer is connected to SPI1:
//
// Max bux speed is 10 MHz
// SCK on PA5 AF5
// MISO in PA6 AF5
// MOSI on PA7 AF5
// CS (I2C vs. SPI) on PE3
// CPOL = 1 (clock is idle high)
// CPHA = 1 (read on trailing (rising) edge of clock)

pub struct Lis3dsh {
    spi: Spi<'static, peripherals::SPI1, NoDma, NoDma>,
    cs: Output<'static, peripherals::PE3>,
}

impl Lis3dsh {
    const REG_WHO_AM_I: u8 = 0x0F;
    const REG_CTRL_4: u8 = 0x20;
    const REG_CTRL_5: u8 = 0x24;

    const fn lis3dsh_make_address_byte(register_address: u8, read: bool) -> u8 {
        register_address | ((read as u8) << 7)
    }

    pub fn new(
        spi1: peripherals::SPI1,
        pa5: peripherals::PA5,
        pa7: peripherals::PA7,
        pa6: peripherals::PA6,
        pe3: peripherals::PE3,
    ) -> Self {
        let spi_config = {
            let mut s = Config::default();
            s.frequency = Hertz(10_000_000);
            s.mode = MODE_3;
            s
        };

        Lis3dsh {
            spi: Spi::new(spi1, pa5, pa7, pa6, NoDma, NoDma, spi_config),
            cs: Output::new(pe3, Level::High, Speed::VeryHigh),
        }
    }

    pub fn init(&mut self) {
        let mut buf = [
            Self::lis3dsh_make_address_byte(Self::REG_WHO_AM_I, true),
            0x00,
        ];
        self.cs.set_low();
        unwrap!(self.spi.blocking_transfer_in_place(&mut buf));
        self.cs.set_high();
        info!(
            "xfer {=[u8]:x}, expect second byte is {:x}",
            buf, 0b00111111
        );
    }
}
