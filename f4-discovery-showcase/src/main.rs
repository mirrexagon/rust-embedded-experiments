#![no_std]
#![no_main]

// Hardware available:
// - Accelerometer (LIS3DSH) connected to SPI1
// - Digital microphone (MP45DT02)
// - audio DAC with integrated class D speaker driver (CS43L22)
// - PWMable LEDs
// - One user momentary button
// - USB OTG Micro-AB connector (host or device)

// Functionality:
// - When board is tilted, the LEDs light up following the direction and amount of tilt.
// - The DAC outputs a signal with frequency/pitch determined by amount of tilt.
// - While the user button is held, a song plays from the DAC instead (including a vibrato effect), and the LEDs brightness corresponds to the intensity of the sound.
// - The board presents as a USB audio device with one mono input (onboard mic) and one stereo input, mimicking the DAC output.
//   - Could present the DAC as a stereo output, but mimicking the DAC output shows the cross-communication between the sound generation, the DAC output, and the USB audio.

mod lis3dsh;

use core::cell::RefCell;
use num_traits::float::FloatCore;

use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio;
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};
use embassy_stm32::peripherals;
use embassy_stm32::spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::{raw::NoopRawMutex, NoopMutex};
use embassy_sync::signal::Signal;
use embassy_time::Timer;

use lis3dsh::RawAccel;

use static_cell::StaticCell;

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

static SPI1_BUS: StaticCell<NoopMutex<RefCell<spi::Spi<peripherals::SPI1, NoDma, NoDma>>>> =
    StaticCell::new();

static RAW_ACCEL_SIGNAL: Signal<CriticalSectionRawMutex, lis3dsh::RawAccel> = Signal::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = {
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

        embassy_stm32::init(config)
    };

    let spi1_bus = {
        let sck = p.PA5;
        let mosi = p.PA7;
        let miso = p.PA6;
        let mut config = spi::Config::default();
        config.frequency = Hertz(lis3dsh::MAXIMUM_SPI_FREQUENCY_HZ);

        let spi = spi::Spi::new(p.SPI1, sck, mosi, miso, NoDma, NoDma, config);
        let spi_bus = NoopMutex::new(RefCell::new(spi));
        SPI1_BUS.init(spi_bus)
    };

    unwrap!(spawner.spawn(accel_task(spi1_bus, p.PE3)));
    unwrap!(spawner.spawn(led_task(p.PD12, p.PD13, p.PD14, p.PD15, p.TIM4)));

    loop {
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn accel_task(
    spi1_bus: &'static NoopMutex<RefCell<spi::Spi<'static, peripherals::SPI1, NoDma, NoDma>>>,
    cs: peripherals::PE3,
) {
    let mut accel = {
        let cs = Output::new(cs, Level::High, Speed::Low);
        let spi_device = SpiDevice::new(spi1_bus, cs);
        lis3dsh::Lis3dsh::new(spi_device)
    };
    unwrap!(accel.init());

    loop {
        let raw_accel = unwrap!(accel.read_raw_accel());
        RAW_ACCEL_SIGNAL.signal(raw_accel);
        info!("{:?}", raw_accel);
        Timer::after_millis(10).await;
    }
}

#[embassy_executor::task]
async fn led_task(
    pd12: peripherals::PD12,
    pd13: peripherals::PD13,
    pd14: peripherals::PD14,
    pd15: peripherals::PD15,
    tim4: peripherals::TIM4,
) {
    // With silkscreen upright, mini-USB at top:
    //
    // Green, west, PD12 - TIM4_CH1
    // Orange, north, PD13 - TIM4_CH2
    // Red, east, PD14 - TIM4_CH3
    // Blue, south, PD15 - TIM4_CH4

    let ch1 = PwmPin::new_ch1(pd12, OutputType::PushPull);
    let ch2 = PwmPin::new_ch2(pd13, OutputType::PushPull);
    let ch3 = PwmPin::new_ch3(pd14, OutputType::PushPull);
    let ch4 = PwmPin::new_ch4(pd15, OutputType::PushPull);
    let mut pwm = SimplePwm::new(
        tim4,
        Some(ch1),
        Some(ch2),
        Some(ch3),
        Some(ch4),
        Hertz::khz(10),
        Default::default(),
    );
    let mut max_duty = pwm.get_max_duty() as f32;
    pwm.enable(timer::Channel::Ch1);
    pwm.enable(timer::Channel::Ch2);
    pwm.enable(timer::Channel::Ch3);
    pwm.enable(timer::Channel::Ch4);

    // Above this, there is no visible difference to the brightness of the LEDs.
    max_duty /= 4.0;

    let mut set_north =
        |intensity: f32| pwm.set_duty(timer::Channel::Ch2, (intensity * max_duty) as u16);
    let mut set_east =
        |intensity: f32| pwm.set_duty(timer::Channel::Ch3, (intensity * max_duty) as u16);
    let mut set_south =
        |intensity: f32| pwm.set_duty(timer::Channel::Ch4, (intensity * max_duty) as u16);
    let mut set_west =
        |intensity: f32| pwm.set_duty(timer::Channel::Ch1, (intensity * max_duty) as u16);

    // With the silkscreen upright, mini-USB at top:
    //
    // Tilt board so red LED (east) goes down , X increases
    // Tilt board so blue LED (south) goes down, Y decreases

    // Let's model the LEDs as being on a plane one unit each from the origin.
    // A point is placed some distance above the origin
    // As the board rotates, the LED plane rotates to match, but the point stays stationary.
    // An LED's intensity is determined by its distance from the point - closer is more intense.

    loop {
        let RawAccel { x, y, z } = RAW_ACCEL_SIGNAL.wait().await;

        let x = x as f32;
        let y = y as f32;
        let z = z as f32;

        let magnitude = (x.powi(2) + y.powi(2) + z.powi(2)).sqrt();
    }
}
