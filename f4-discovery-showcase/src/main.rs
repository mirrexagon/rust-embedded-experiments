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
// - While the user button is held, a song plays from the DAC instead (including a vibrato effect).
// - The board presents as a USB audio device with one mono input (onboard mic) and one stereo input, mimicking the DAC output.
//   - Could present the DAC as a stereo output, but mimicking the DAC output shows the cross-communication between the sound generation, the DAC output, and the USB audio.

mod lis3dsh;

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio;
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};
use embassy_stm32::peripherals;
use embassy_stm32::spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_time::Timer;

use embassy_sync::blocking_mutex::NoopMutex;

use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;

use static_cell::StaticCell;

use core::cell::RefCell;

static SPI1_BUS: StaticCell<NoopMutex<RefCell<spi::Spi<peripherals::SPI1, NoDma, NoDma>>>> =
    StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
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

    // Accelerometer init
    let mut accel = {
        let cs = Output::new(p.PE3, Level::High, Speed::Low);
        let spi_device = SpiDevice::new(spi1_bus, cs);
        lis3dsh::Lis3dsh::new(spi_device)
    };
    unwrap!(accel.init());

    // LEDs init
    //
    // With silkscreen upright, mini-USB at top:
    //
    // Green, west, PD12 - TIM4_CH1
    // Orange, north, PD13 - TIM4_CH2
    // Red, east, PD14 - TIM4_CH3
    // Blue, south, PD15 - TIM4_CH4

    let ch1 = PwmPin::new_ch1(p.PD12, OutputType::PushPull);
    let ch2 = PwmPin::new_ch2(p.PD13, OutputType::PushPull);
    let ch3 = PwmPin::new_ch3(p.PD14, OutputType::PushPull);
    let ch4 = PwmPin::new_ch4(p.PD15, OutputType::PushPull);
    let mut pwm = SimplePwm::new(
        p.TIM4,
        Some(ch1),
        Some(ch2),
        Some(ch3),
        Some(ch4),
        Hertz::khz(10),
        Default::default(),
    );
    let mut max_duty = pwm.get_max_duty();
    pwm.enable(timer::Channel::Ch1);
    pwm.enable(timer::Channel::Ch2);
    pwm.enable(timer::Channel::Ch3);
    pwm.enable(timer::Channel::Ch4);

    info!("PWM initialized");
    info!("PWM max duty {}", max_duty);

    max_duty /= 4;

    let mut set_duty = |duty| {
        pwm.set_duty(timer::Channel::Ch1, duty);
        pwm.set_duty(timer::Channel::Ch2, duty);
        pwm.set_duty(timer::Channel::Ch3, duty);
        pwm.set_duty(timer::Channel::Ch4, duty);
    };

    loop {
        let mut duty = 0;

        info!("Start up");
        while duty < max_duty {
            set_duty(duty);
            duty += 1;
            Timer::after_millis(1).await;
        }

        info!("Start down");
        while duty > 0 {
            set_duty(duty);
            duty -= 1;
            Timer::after_millis(1).await;
        }

        info!("{:?}", unwrap!(accel.read_raw_accel()));

        Timer::after_millis(100).await;
    }
}
