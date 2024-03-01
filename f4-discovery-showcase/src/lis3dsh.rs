use embassy_stm32::dma;
use embassy_stm32::gpio;
use embassy_stm32::peripherals;
use embassy_stm32::spi;
use embassy_stm32::time::Hertz;

use crate::fmt::info;

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
    spi: spi::Spi<'static, peripherals::SPI1, dma::NoDma, dma::NoDma>,
    cs: gpio::Output<'static, peripherals::PE3>,
}

impl Lis3dsh {
    const REG_WHO_AM_I: u8 = 0x0F;
    const WHO_AM_I: u8 = 0b00111111;

    const REG_CTRL_4: u8 = 0x20;
    const REG_CTRL_5: u8 = 0x24;

    const fn make_address_byte(register_address: u8, read: bool) -> u8 {
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
            let mut s = spi::Config::default();
            s.frequency = Hertz(10_000_000);
            s.mode = spi::MODE_3;
            s
        };

        Lis3dsh {
            spi: spi::Spi::new(spi1, pa5, pa7, pa6, dma::NoDma, dma::NoDma, spi_config),
            cs: gpio::Output::new(pe3, gpio::Level::High, gpio::Speed::VeryHigh),
        }
    }

    pub fn init(&mut self) -> Result<(), Error> {
        let mut buf = [Self::make_address_byte(Self::REG_WHO_AM_I, true), 0x00];
        self.cs.set_low();
        self.spi.blocking_transfer_in_place(&mut buf).unwrap();
        self.cs.set_high();

        let id_byte = buf[1];

        if id_byte == Self::WHO_AM_I {
            Ok(())
        } else {
            Err(Error::WhoAmIMismatch)
        }
    }
}

#[derive(Debug)]
pub enum Error {
    WhoAmIMismatch,
}
