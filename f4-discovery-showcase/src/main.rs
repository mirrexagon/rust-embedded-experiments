#![no_std]
#![no_main]

mod fmt;
mod lis3dsh;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::time::Hertz;
use embassy_time::Timer;

use fmt::info;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let config = {
        let mut config = embassy_stm32::Config::default();
        use embassy_stm32::rcc::*;

        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV8,
            mul: PllMul::MUL336,
            divp: Some(PllPDiv::DIV2),
            divq: Some(PllQDiv::DIV7),
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.sys = Sysclk::PLL1_P;

        config
    };
    let mut p = embassy_stm32::init(config);

    let mut accel = lis3dsh::Lis3dsh::new(p.SPI1, p.PA5, p.PA7, p.PA6, p.PE3);
    accel.init().unwrap();

    loop {
        info!("{:?}", accel.read_raw_accel());
        Timer::after_millis(100).await;
    }
}
