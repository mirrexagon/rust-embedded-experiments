#![no_std]
#![no_main]

mod fmt;

//mod lis3dsh;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{Duration, Timer};
use embassy_stm32::dma::NoDma;
use embassy_stm32::spi::{Config, Instance, Spi, MODE_3};
use embassy_stm32::time::Hertz;
use fmt::info;

const REG_WHO_AM_I: u8 = 0x0F;
const REG_CTRL_4: u8 = 0x20;
const REG_CTRL_5: u8 = 0x24;

const fn lis3dsh_make_address_byte(register_address: u8, read: bool) -> u8 {
    register_address | ((read as u8) << 7)
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_stm32::init(Default::default());

    let spi_config = {
        let mut s = Config::default();

        // LIS3DSH max bux speed is 10 MHz.
        s.frequency = Hertz(10_000_000);

        s.mode = MODE_3;

        s
    };

    let mut spi = Spi::new(p.SPI1, p.PA5, p.PA7, p.PA6, NoDma, NoDma, spi_config);
    let mut cs = Output::new(p.PE3, Level::High, Speed::VeryHigh);

    let mut buf = [lis3dsh_make_address_byte(REG_WHO_AM_I, true), 0x00];
    cs.set_low();
    unwrap!(spi.blocking_transfer_in_place(&mut buf));
    cs.set_high();
    info!("xfer {=[u8]:x}, expect second byte is {:x}", buf, 0b00111111);
}
