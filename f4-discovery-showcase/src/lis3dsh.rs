use core::fmt::Write;
use core::str::from_utf8;

use defmt::*;
use embassy_stm32::Peripherals;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi::{Config, Instance, Spi, MODE_3};
use embassy_stm32::time::Hertz;

// lis3dsh accelerometer is connected to SPI1:
// SCK on PA5 AF5
// MISO in PA6 AF5
// MOSI on PA7 AF5
// CS (I2C vs. SPI) on PE3
// CPOL = 1 (clock is idle high)
// CPHA = 1 (read on trailing (rising) edge of clock)

pub struct Lis3dsh<'d, T: Instance, Tx, Rx> {
    spi: Spi<'d, T, Tx, Rx>,
}

impl<'d, T: Instance, Tx, Rx> Lis3dsh<'d, T, Tx, Rx> {
    pub async fn new(p: &mut Peripherals) -> Self {
    }
}

