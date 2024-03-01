#![no_std]
#![no_main]

mod fmt;
mod lis3dsh;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_time::Timer;

use fmt::info;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_stm32::init(Default::default());

    let mut accel = lis3dsh::Lis3dsh::new(p.SPI1, p.PA5, p.PA7, p.PA6, p.PE3);
    accel.init().unwrap();

    loop {
        info!("{:?}", accel.read_raw_accel());
        Timer::after_millis(300).await;
    }
}
