#![no_std]
#![no_main]
use core::{num::{NonZeroU16, NonZeroU8}, time::Duration};

use pac::{timer::{TimAdv, TimGp16}, RCC};
use stm32_metapac as pac;

use crate::{gpio::DynamicPin, pwm::Pwm};

use fdcan::{config::{DataBitTiming, FdCanConfig}, frame::TxFrameHeader, id::StandardId, FdCan};
use cortex_m_rt as _;
use panic_halt as _;

mod current_control;
mod gpio;
mod pwm;

use gpio::Pin;

struct FdCan1 {}

unsafe impl fdcan::message_ram::Instance for FdCan1 {
    const MSG_RAM: *mut fdcan::message_ram::RegisterBlock = unsafe { pac::FDCANRAM1.as_ptr() as _};
}
unsafe impl fdcan::Instance for FdCan1 {
    const REGISTERS: *mut fdcan::RegisterBlock = unsafe { pac::FDCAN1.as_ptr() as _ };
}

// pub const FDCAN1: can::Fdcan = unsafe { can::Fdcan::from_ptr(0x4000_6400 as usize as _) };
// pub const FDCAN2: can::Fdcan = unsafe { can::Fdcan::from_ptr(0x4000_6800 as usize as _) };
// pub const FDCAN3: can::Fdcan = unsafe { can::Fdcan::from_ptr(0x4000_6c00 as usize as _) };

#[cortex_m_rt::entry]
fn main() -> ! {


    // Setup PLL to generate clock of 16MHz * 8
    RCC.cr().write(|w| w.set_hsion(true));
    RCC.pllcfgr().write(|w| {
        w.set_plln(pac::rcc::vals::Plln::MUL16);
        w.set_pllm(pac::rcc::vals::Pllm::DIV1);
        w.set_pllsrc(pac::rcc::vals::Pllsrc::HSI);
        w.set_pllr(pac::rcc::vals::Pllr::DIV2);
        w.set_pllren(true);
    });
    // Enable the PLL
    RCC.cr().modify(|w| w.set_pllon(true));
    while !RCC.cr().read().pllrdy() {}

    const CLK_FREQ: u32 = 128_000_000;
    const PWM_FREQ: u32 = 100_000;


    pac::PWR.cr5().modify(|w| w.set_r1mode(true));
    pac::FLASH.acr().modify(|w| w.set_latency(pac::flash::vals::Latency::WS4));
    RCC.cfgr().modify(|w| {
        w.set_sw(pac::rcc::vals::Sw::PLL1_R);
        w.set_hpre(pac::rcc::vals::Hpre::DIV1);
        w.set_ppre1(pac::rcc::vals::Ppre::DIV1);
        w.set_ppre2(pac::rcc::vals::Ppre::DIV1);
    });


    RCC.apb1enr1().modify(|w| w.set_fdcanen(true));
    RCC.apb2enr().modify(|w| w.set_tim1en(true));
    RCC.ahb2enr().modify(|w| w.set_gpioaen(true));
    RCC.ahb2enr().modify(|w| w.set_gpioben(true));
    RCC.ahb2enr().modify(|w| w.set_gpiocen(true));
    RCC.ahb2enr().modify(|w| w.set_gpioden(true));

    // PWM outputs are spread seemingly randomly over 3 timers. Why aren't these on two timers? That
    // is a question for 4-months ago me. I can only imagine I mistakenly re-assigned some pins and
    // got lucky that they all landed on timer outputs?


    let gpios = gpio::gpios();

    // assign timer outputs to the appropriate AF for the timer channels
    let io_ch1_in1 = gpios.PC3;
    let io_ch1_in2 = gpios.PA6;
    let io_ch2_in1 = gpios.PA8;
    let io_ch2_in2 = gpios.PA7;
    let io_ch3_in1 = gpios.PA9;
    let io_ch3_in2 = gpios.PA15;
    let io_ch4_in1 = gpios.PB8;
    let io_ch4_in2 = gpios.PB9;

    io_ch1_in1.set_as_af(2, gpio::AFType::OutputPushPull);
    io_ch1_in2.set_as_af(2, gpio::AFType::OutputPushPull);
    io_ch2_in1.set_as_af(6, gpio::AFType::OutputPushPull);
    io_ch2_in2.set_as_af(2, gpio::AFType::OutputPushPull);
    io_ch3_in1.set_as_af(6, gpio::AFType::OutputPushPull);
    io_ch3_in2.set_as_af(2, gpio::AFType::OutputPushPull);
    io_ch4_in1.set_as_af(10, gpio::AFType::OutputPushPull);
    io_ch4_in2.set_as_af(10, gpio::AFType::OutputPushPull);


    // let pwm_ios: &[DynamicPin] = &[
    //     io_ch1_in1.into(),
    //     io_ch1_in2.into(),
    //     io_ch2_in1.into(),
    //     io_ch2_in2.into(),
    //     io_ch3_in1.into(),
    //     io_ch3_in2.into(),
    //     io_ch4_in1.into(),
    //     io_ch4_in2.into(),
    // ];

    let io_ch1_en1 = gpios.PA4;
    let io_ch1_en2 = gpios.PC5;
    let io_ch2_en1 = gpios.PC6;
    let io_ch2_en2 = gpios.PC4;
    let io_ch3_en1 = gpios.PA10;
    let io_ch3_en2 = gpios.PC10;
    let io_ch4_en1 = gpios.PC12;
    let io_ch4_en2 = gpios.PD2;
    let enable_ios: &mut [DynamicPin] = &mut [
        io_ch1_en1.into(),
        io_ch1_en2.into(),
        io_ch2_en1.into(),
        io_ch2_en2.into(),
        io_ch3_en1.into(),
        io_ch3_en2.into(),
        io_ch4_en1.into(),
        io_ch4_en2.into(),
    ];

    for io in enable_ios {
        io.set_high();
        io.set_as_output(gpio::Speed::Low);
    }

    // TODO: Why can the generic type here not be inferred from the argument?
    let timer1 = pwm::PwmTimer::<TimAdv>::new(pac::TIM1, PWM_FREQ, CLK_FREQ);
    let timer3 = pwm::PwmTimer::<TimGp16>::new(pac::TIM3, PWM_FREQ, CLK_FREQ);
    let timer8 = pwm::PwmTimer::<TimAdv>::new(pac::TIM8, PWM_FREQ, CLK_FREQ);


    let ch1_in1 = timer1.ch4();
    let ch1_in2 = timer3.ch1();
    let ch2_in1 = timer1.ch1();
    let ch2_in2 = timer3.ch2();
    let ch3_in1 = timer1.ch2();
    let ch3_in2 = timer8.ch1();
    let ch4_in1 = timer8.ch2();
    let ch4_in2 = timer8.ch3();

    let current1 = current_control::IChannel::new(ch1_in1, ch1_in2);
    current1.set_duty_cycle(-10000);
    let current2 = current_control::IChannel::new(ch2_in1, ch2_in2);
    current2.set_duty_cycle(10000);
    let current3 = current_control::IChannel::new(ch3_in1, ch3_in2);
    current3.set_duty_cycle(10000);
    let current4 = current_control::IChannel::new(ch4_in1, ch4_in2);
    current4.set_duty_cycle(10000);

    let mut can = FdCan::new(FdCan1 { } ).into_config_mode();
    let can_config = FdCanConfig::default();
    can_config.set_nominal_bit_timing(fdcan::config::NominalBitTiming{
        prescaler: NonZeroU16::new(1).unwrap(),
        seg1: NonZeroU8::new(16).unwrap(),
        seg2: NonZeroU8::new(2).unwrap(),
        sync_jump_width: NonZeroU8::new(1).unwrap(),
    });

    can.apply_config(can_config);
    let can = can.into_normal();

    let (_can_control, mut can_tx, can_rx0, _can_rx1) = can.split();

    let can_task = core::pin::pin!(async {
        let buffer = [0u8; 8];
        loop {
            lilos::time::sleep_for(Duration::from_millis(10)).await;
            can_tx.transmit(
                TxFrameHeader {
                    len: 8,
                    frame_format: fdcan::frame::FrameFormat::Standard,
                    id: fdcan::id::Id::Standard(StandardId::new(0x100).unwrap()),
                    bit_rate_switching: false,
                    marker: None
                },
                &buffer
            ).unwrap();
        }
    });


    lilos::exec::run_tasks(&mut [can_task], lilos::exec::ALL_TASKS);

}
