#![no_std]
#![no_main]
use core::{
    convert::Infallible,
    num::{NonZeroU16, NonZeroU8},
    result,
    time::Duration,
};

use cortex_m::interrupt;
use lilos::time::{Millis, TickTime};
use pac::{
    timer::{TimAdv, TimGp16},
    RCC,
};
use stm32_metapac as pac;

use crate::{gpio::DynamicPin, pwm::Pwm};

use cortex_m_rt as _;
use defmt_rtt as _;
use fdcan::{
    config::{DataBitTiming, FdCanConfig},
    frame::TxFrameHeader,
    id::StandardId,
    FdCan, Instance, NormalOperationMode,
};
use panic_probe as _;

mod adc;
mod current_control;
mod gpio;
mod pwm;

use gpio::Pin;

struct FdCan1 {}
struct FdCan2 {}

unsafe impl fdcan::message_ram::Instance for FdCan1 {
    const MSG_RAM: *mut fdcan::message_ram::RegisterBlock = pac::FDCANRAM1.as_ptr() as _;
}
unsafe impl fdcan::Instance for FdCan1 {
    const REGISTERS: *mut fdcan::RegisterBlock = pac::FDCAN1.as_ptr() as _;
}
unsafe impl fdcan::message_ram::Instance for FdCan2 {
    const MSG_RAM: *mut fdcan::message_ram::RegisterBlock = pac::FDCANRAM2.as_ptr() as _;
}
unsafe impl fdcan::Instance for FdCan2 {
    const REGISTERS: *mut fdcan::RegisterBlock = pac::FDCAN2.as_ptr() as _;
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
    const PWM_FREQ: u32 = 30_000;

    defmt::info!("HELLO WORLD!");
    pac::PWR.cr5().modify(|w| w.set_r1mode(true));
    pac::FLASH
        .acr()
        .modify(|w| w.set_latency(pac::flash::vals::Latency::WS4));
    RCC.cfgr().modify(|w| {
        w.set_sw(pac::rcc::vals::Sw::PLL1_R);
        w.set_hpre(pac::rcc::vals::Hpre::DIV1);
        w.set_ppre1(pac::rcc::vals::Ppre::DIV1);
        w.set_ppre2(pac::rcc::vals::Ppre::DIV1);
    });

    RCC.apb1enr1().modify(|w| {
        w.set_tim3en(true);
        w.set_fdcanen(true);
    });
    RCC.ccipr()
        .write(|w| w.set_fdcansel(pac::rcc::vals::Fdcansel::PCLK1));
    RCC.apb2enr().modify(|w| {
        w.set_tim1en(true);
        w.set_tim8en(true);
    });
    RCC.ahb2enr().modify(|w| {
        w.set_gpioaen(true);
        w.set_gpioben(true);
        w.set_gpiocen(true);
        w.set_gpioden(true);
        w.set_gpioeen(true);
        w.set_gpiofen(true);
        w.set_gpiogen(true);
    });

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

    adc::configure_adcs();
    adc::configure_op_amps();

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
    current1.set_duty_cycle(0);
    let current2 = current_control::IChannel::new(ch2_in1, ch2_in2);
    current2.set_duty_cycle(0);
    let current3 = current_control::IChannel::new(ch3_in1, ch3_in2);
    current3.set_duty_cycle(0);
    let current4 = current_control::IChannel::new(ch4_in1, ch4_in2);
    current4.set_duty_cycle(0);

    timer1.enable();
    timer3.enable();
    timer8.enable();

    let can_rx_pin = gpios.PB12;
    let mut can_tx_pin = gpios.PB13;
    can_tx_pin.set_high();
    can_tx_pin.set_as_output(gpio::Speed::High);

    can_rx_pin.set_as_af(9, gpio::AFType::Input);
    can_tx_pin.set_as_af(9, gpio::AFType::OutputPushPull);

    let mut can = FdCan::new(FdCan2 {}).into_config_mode();
    let can_config = FdCanConfig::default()
        .set_automatic_retransmit(false)
        .set_frame_transmit(fdcan::config::FrameTransmissionConfig::ClassicCanOnly)
        .set_data_bit_timing(DataBitTiming {
            transceiver_delay_compensation: false,
            prescaler: NonZeroU8::new(8).unwrap(),
            seg1: NonZeroU8::new(13).unwrap(),
            seg2: NonZeroU8::new(2).unwrap(),
            sync_jump_width: NonZeroU8::new(1).unwrap(),
        })
        .set_nominal_bit_timing(fdcan::config::NominalBitTiming {
            prescaler: NonZeroU16::new(8).unwrap(),
            seg1: NonZeroU8::new(13).unwrap(),
            seg2: NonZeroU8::new(2).unwrap(),
            sync_jump_width: NonZeroU8::new(1).unwrap(),
        });

    can.apply_config(can_config);
    let mut can = can.into_normal();

    //let (_can_control, mut can_tx, can_rx0, _can_rx1) = can.split();

    pac::DBGMCU.cr().modify(|w| {
        w.set_dbg_standby(true);
        w.set_dbg_sleep(true);
        w.set_dbg_stop(true);
    });

    let mut cp = cortex_m::Peripherals::take().unwrap();
    //p.NVIC.iser[0].modify(|w| w | 1<<)
    // Configure the systick timer for 1kHz ticks at 16MHz.
    lilos::time::initialize_sys_tick(&mut cp.SYST, 128_000_000);

    let mut next_tick = TickTime::from_millis_since_boot(1000);

    // let buffer = [0xaau8; 8];
    // let mut count = 0u16;
    // loop {
    //     let psr = pac::FDCAN2.psr().read();
    //     defmt::info!(
    //         "PSR LEC=0x{:x} DLEC=0x{:x}",
    //         psr.lec().to_bits(),
    //         psr.dlec()
    //     );

    //     while TickTime::now() < next_tick {}
    //     next_tick += Duration::from_millis(1000);

    //     count = count.overflowing_add(1).0;

    //     let result = can.transmit(
    //         TxFrameHeader {
    //             len: 8,
    //             frame_format: fdcan::frame::FrameFormat::Standard,
    //             id: fdcan::id::Id::Standard(StandardId::new(0x1bf).unwrap()),
    //             bit_rate_switching: false,
    //             marker: None,
    //         },
    //         &buffer,
    //     );
    //     match result {
    //         Ok(ret) => match ret {
    //             Some(_) => defmt::info!("Some"),
    //             None => defmt::info!("None"),
    //         },
    //         Err(_) => defmt::error!("CAN ERROR"),
    //     }
    //     defmt::info!("CAN {}", count);
    //     let errors = can.error_counters();
    //     defmt::info!("Errors: {} {}", errors.can_errors, errors.transmit_err);
    // }

    // let can_task = core::pin::pin!(async {
    //     let buffer = [0xaau8; 8];
    //     let mut count = 0u16;
    //     loop {
    //         // if can_tx_pin.is_out_high() {
    //         //     can_tx_pin.set_low();
    //         // } else {
    //         //     can_tx_pin.set_high();
    //         // }
    //         count = count.overflowing_add(1).0;
    //         lilos::time::sleep_for(Duration::from_millis(500)).await;

    //         if let Err(e) = can.transmit(
    //             TxFrameHeader {
    //                 len: 8,
    //                 frame_format: fdcan::frame::FrameFormat::Standard,
    //                 id: fdcan::id::Id::Standard(StandardId::new(0x1bf).unwrap()),
    //                 bit_rate_switching: false,
    //                 marker: None,
    //             },
    //             &buffer,
    //         ) {
    //             defmt::info!("CAN ERROR");
    //         }
    //     }
    // });

    //adc::trim_opamps();

    unsafe { cortex_m::interrupt::enable() };

    lilos::exec::run_tasks(&mut [core::pin::pin!(main_task(can))], lilos::exec::ALL_TASKS);
}

const CONTROL_INTERVAL: Millis = Millis(5);

async fn main_task(mut can: FdCan<FdCan2, NormalOperationMode>) -> Infallible {
    let mut periodic_gate =
        lilos::time::PeriodicGate::new_shift(CONTROL_INTERVAL.into(), Millis(0));
    loop {
        periodic_gate.next_time().await;

        let adc_values = adc::read_isense();
        let mut can_buffer = [0u8; 8];
        can_buffer[0..2].copy_from_slice(&adc_values[0].to_le_bytes());
        can_buffer[2..4].copy_from_slice(&adc_values[1].to_le_bytes());
        can_buffer[4..6].copy_from_slice(&adc_values[2].to_le_bytes());
        can_buffer[6..8].copy_from_slice(&adc_values[3].to_le_bytes());

        can.transmit(
            TxFrameHeader {
                len: 8,
                frame_format: fdcan::frame::FrameFormat::Standard,
                id: fdcan::id::Id::Standard(StandardId::new(0x100).unwrap()),
                bit_rate_switching: false,
                marker: None,
            },
            &can_buffer,
        ).ok();
    }
}
