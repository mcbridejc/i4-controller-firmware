#![no_std]
#![no_main]
use core::{
    borrow::BorrowMut, cell::RefCell, convert::Infallible, mem::MaybeUninit, num::{NonZeroU16, NonZeroU8}, pin::pin, sync::atomic::{AtomicBool, AtomicI16}
};

use cortex_m::interrupt::Mutex;
use lilos::time::{Millis, TickTime};
use pac::{
    timer::{TimAdv, TimGp16},
    RCC,
};
use pwm::PwmTimer;
use static_cell::StaticCell;
use stm32_metapac::{self as pac, interrupt};

use crate::{gpio::DynamicPin, step_timer::StepTimer, xydrive::XYDrive};

use cortex_m_rt as _;
use defmt_rtt as _;
use fdcan::{
    config::{DataBitTiming, FdCanConfig},
    frame::TxFrameHeader,
    id::StandardId,
    FdCan, NormalOperationMode,
};
use panic_probe as _;

mod adc;
mod current_control;
mod gpio;
mod pwm;
mod stepper;
mod step_timer;
mod xydrive;

use stepper::Stepper;

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

static XY_DRIVE: Mutex<RefCell<Option<XYDrive>>> = Mutex::new(RefCell::new(None));

static X_STEPPER: StaticCell<Stepper> = StaticCell::new();
static Y_STEPPER: StaticCell<Stepper> = StaticCell::new();
// References for the IRQs
static mut X_STEPPER_REF: Option<&Stepper> = None;
static mut Y_STEPPER_REF: Option<&Stepper> = None;

static TIMER1: StaticCell<PwmTimer<TimAdv>> = StaticCell::new();
static TIMER3: StaticCell<PwmTimer<TimGp16>> = StaticCell::new();
static TIMER8: StaticCell<PwmTimer<TimAdv>> = StaticCell::new();

const MIN_STEP_FREQ: i32 = 8;
const MAX_STEP_FREQ: i32 = 800;

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
        w.set_tim2en(true);
        w.set_tim3en(true);
        w.set_tim5en(true);
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
    let timer1 = TIMER1.init(pwm::PwmTimer::<TimAdv>::new(pac::TIM1, PWM_FREQ, CLK_FREQ));
    let timer3 = TIMER3.init(pwm::PwmTimer::<TimGp16>::new(pac::TIM3, PWM_FREQ, CLK_FREQ));
    let timer8 = TIMER8.init(pwm::PwmTimer::<TimAdv>::new(pac::TIM8, PWM_FREQ, CLK_FREQ));

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

    let x_stepper = X_STEPPER.init(Stepper::new(current1, current2));
    let y_stepper = Y_STEPPER.init(Stepper::new(current3, current4));
    unsafe {
        X_STEPPER_REF = Some(x_stepper);
        Y_STEPPER_REF = Some(y_stepper);
    }

    // let xy_drive = XYDrive::new(current1, current2, current3, current4);
    // cortex_m::interrupt::free(|cs| {
    //     XY_DRIVE.borrow(cs).replace(Some(xy_drive));
    // });

    let mut x_step_timer = StepTimer::new(pac::TIM2, CLK_FREQ);
    let mut y_step_timer = StepTimer::new(pac::TIM5, CLK_FREQ);
    x_step_timer.enable_irq();
    y_step_timer.enable_irq();
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM2);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM5);
    }

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
    let can = can.into_normal();
    let (_can_control, can_tx, can_rx0, _can_rx1) = can.split();

    pac::DBGMCU.cr().modify(|w| {
        w.set_dbg_standby(true);
        w.set_dbg_sleep(true);
        w.set_dbg_stop(true);
    });

    let mut cp = cortex_m::Peripherals::take().unwrap();
    //p.NVIC.iser[0].modify(|w| w | 1<<)
    // Configure the systick timer for 1kHz ticks at 16MHz.
    lilos::time::initialize_sys_tick(&mut cp.SYST, 128_000_000);

    //adc::trim_opamps();

    unsafe { cortex_m::interrupt::enable() };

    lilos::exec::run_tasks(
        &mut [
            pin!(main_task(can_tx)),
            pin!(can_rx_task(can_rx0, x_step_timer, y_step_timer, &x_stepper, &y_stepper)),
        ],

        lilos::exec::ALL_TASKS,
    );
}

const CONTROL_INTERVAL: Millis = Millis(5);

async fn main_task(mut can: fdcan::Tx<FdCan2, NormalOperationMode>) -> Infallible {
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
        )
        .ok();
    }
}

async fn can_rx_task(
    mut can_rx: fdcan::Rx<FdCan2, NormalOperationMode, fdcan::Fifo0>,
    mut x_step_timer: StepTimer,
    mut y_step_timer: StepTimer,
    x_stepper: &Stepper<'static>,
    y_stepper: &Stepper<'static>,
) -> Infallible {
    let mut buffer = [0u8; 8];
    let mut periodic_gate =
        lilos::time::PeriodicGate::new_shift(CONTROL_INTERVAL.into(), Millis(0));
    // HACKY hardcoded joystick offset calibration.
    const X_OFFSET: i16 = 16;
    const Y_OFFSET: i16 = 40;
    loop {
        periodic_gate.next_time().await;
        if let Ok(msg) = can_rx.receive(&mut buffer) {
            let msg = msg.unwrap();

            if msg.id == StandardId::new(0x130).unwrap().into() {
                let x_cmd = i16::from_le_bytes(buffer[6..8].try_into().unwrap()) - 2048 - X_OFFSET;
                let y_cmd = i16::from_le_bytes(buffer[4..6].try_into().unwrap()) - 2048 - Y_OFFSET;
                let x_vel = x_cmd as i32 * MAX_STEP_FREQ / 2048;
                let y_vel = -1 * y_cmd as i32 * MAX_STEP_FREQ / 2048;

                if x_vel.abs() > MIN_STEP_FREQ || y_vel.abs() > MIN_STEP_FREQ {
                    let x_reverse = x_vel < 0;
                    let y_reverse = y_vel < 0;
                    x_step_timer.set_overflow_freq(x_vel.abs() as u32);
                    y_step_timer.set_overflow_freq(y_vel.abs() as u32);
                    x_stepper.enable(x_reverse);
                    y_stepper.enable(y_reverse);
                } else {
                    x_step_timer.set_overflow_freq(0);
                    y_step_timer.set_overflow_freq(0);
                    x_stepper.disable();
                    y_stepper.disable();
                }
            }
        }

    }
}

#[pac::interrupt]
unsafe fn TIM2() {
    // Clear all interrupt flags
    pac::TIM2.sr().write(|w| w.0 = 0);

    X_STEPPER_REF.unwrap().step();
}

#[pac::interrupt]
unsafe fn TIM5() {
    pac::TIM5.sr().write(|w| w.0 = 0);

    Y_STEPPER_REF.unwrap().step();
}
