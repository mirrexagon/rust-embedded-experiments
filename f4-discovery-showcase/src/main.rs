#![no_std]
#![no_main]

mod lis3dsh;

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::gpio;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_time::Timer;

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
    let p = embassy_stm32::init(config);

    // Accelerometer init
    let mut accel = lis3dsh::Lis3dsh::new(p.SPI1, p.PA5, p.PA7, p.PA6, p.PE3);
    accel.init().unwrap();

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

        //info!("{:?}", accel.read_raw_accel().unwrap());
        //Timer::after_millis(100).await;
    }
}
