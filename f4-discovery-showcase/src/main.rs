#![no_std]
#![no_main]

mod fmt;
mod lis3dsh;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi::{Config, Instance, Spi, MODE_3};
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Timer};
use fmt::info;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_stm32::init(Default::default());

    let mut accel = lis3dsh::Lis3dsh::new(p.SPI1, p.PA5, p.PA7, p.PA6, p.PE3);
    accel.init();
}
