#![no_std]
#![no_main]
use core::{
    convert::Infallible,
    num::{NonZeroU8, NonZeroU16},
    pin::pin,
    sync::atomic::Ordering,
    time::Duration,
};

use grounded::uninit::GroundedCell;
use lilos::{exec::Notify, time::Millis};
use pac::{
    RCC,
    timer::{TimAdv, TimGp16},
};
use portable_atomic::AtomicBool;
use pwm::PwmTimer;
use static_cell::StaticCell;
use stm32_metapac::{self as pac, interrupt};
use zencan_node::{
    Node,
    common::{NodeId, messages::NmtState},
};

use crate::{gpio::DynamicPin, step_timer::StepTimer};

use cortex_m_rt as _;
use fdcan::{
    FdCan, Fifo0, NormalOperationMode,
    config::{DataBitTiming, FdCanConfig, GlobalFilter},
};
use panic_probe as _;
use rtt_target::{rtt_init, set_defmt_channel};

mod adc;
mod current_control;
mod gpio;
mod pwm;
mod step_timer;
mod stepper;

mod zencan {
    zencan_node::include_modules!(ZENCAN_CONFIG);
}

use stepper::Stepper;

use gpio::Pin;

fn get_serial() -> u32 {
    let mut ctx = md5::Context::new();
    ctx.consume(pac::UID.uid(0).read().to_le_bytes());
    ctx.consume(pac::UID.uid(1).read().to_le_bytes());
    ctx.consume(pac::UID.uid(2).read().to_le_bytes());
    let digest = ctx.compute();
    u32::from_le_bytes(digest.0[0..4].try_into().unwrap())
}

struct FdCan2 {}

unsafe impl fdcan::message_ram::Instance for FdCan2 {
    const MSG_RAM: *mut fdcan::message_ram::RegisterBlock = pac::FDCANRAM2.as_ptr() as _;
}
unsafe impl fdcan::Instance for FdCan2 {
    const REGISTERS: *mut fdcan::RegisterBlock = pac::FDCAN2.as_ptr() as _;
}

static X_STEPPER: GroundedCell<Stepper> = GroundedCell::uninit();
static Y_STEPPER: GroundedCell<Stepper> = GroundedCell::uninit();

/// RX object for CAN IRQ to access
static CAN_RX: GroundedCell<fdcan::Rx<FdCan2, NormalOperationMode, Fifo0>> = GroundedCell::uninit();
/// control object for CAN IRQ to access
static CAN_CTRL: GroundedCell<fdcan::FdCanControl<FdCan2, NormalOperationMode>> =
    GroundedCell::uninit();
/// Notify for NodeMbox notify callback to access
static CAN_NOTIFY: Notify = Notify::new();

static TIMER1: StaticCell<PwmTimer<TimAdv>> = StaticCell::new();
static TIMER3: StaticCell<PwmTimer<TimGp16>> = StaticCell::new();
static TIMER8: StaticCell<PwmTimer<TimAdv>> = StaticCell::new();

const MIN_STEP_FREQ: i32 = 8;
const MAX_STEP_FREQ: i32 = 2000;

fn notify_can_task() {
    CAN_NOTIFY.notify();
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let channels = rtt_init! {
        up: {
            0: {
                size: 512,
                name: "defmt",
            }
        }
    };

    set_defmt_channel(channels.up.0);

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

    let gpios = gpio::gpios();

    // Configure timer output GPIOS to the appropriate AF for the timer channels
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

    // adc::configure_adcs();
    // adc::configure_op_amps();

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
    let current2 = current_control::IChannel::new(ch2_in1, ch2_in2);
    let current3 = current_control::IChannel::new(ch3_in1, ch3_in2);
    let current4 = current_control::IChannel::new(ch4_in1, ch4_in2);
    current1.set_duty_cycle(0);
    current2.set_duty_cycle(0);
    current3.set_duty_cycle(0);
    current4.set_duty_cycle(0);

    // Safety: This is the only time we will mutate these statics
    let (x_stepper, y_stepper) = unsafe {
        *X_STEPPER.get() = Stepper::new(current1, current2);
        *Y_STEPPER.get() = Stepper::new(current3, current4);
        (&*X_STEPPER.get(), &*Y_STEPPER.get())
    };

    let mut x_step_timer = StepTimer::new(pac::TIM2, CLK_FREQ);
    let mut y_step_timer = StepTimer::new(pac::TIM5, CLK_FREQ);
    x_step_timer.enable_irq();
    y_step_timer.enable_irq();

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
        })
        .set_global_filter(GlobalFilter {
            handle_standard_frames: fdcan::config::NonMatchingFilter::IntoRxFifo0,
            handle_extended_frames: fdcan::config::NonMatchingFilter::IntoRxFifo0,
            reject_remote_standard_frames: false,
            reject_remote_extended_frames: false,
        });

    can.apply_config(can_config);
    let mut can = can.into_normal();

    can.enable_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg);
    can.enable_interrupt_line(fdcan::config::InterruptLine::_1, true);
    let (can_control, can_tx, can_rx0, _can_rx1) = can.split();

    // Store CAN objects for receive IRQ
    unsafe {
        core::ptr::write(CAN_CTRL.get(), can_control);
        core::ptr::write(CAN_RX.get(), can_rx0);
    }

    // Register a hook for notification when messages are received that are awaiting handling by
    // node process call. This will be called in the CAN IRQ context, and will set a notify to cause
    // the CAN task to promptly execute.
    zencan::NODE_MBOX.set_process_notify_callback(&notify_can_task);

    pac::DBGMCU.cr().modify(|w| {
        w.set_dbg_standby(true);
        w.set_dbg_sleep(true);
        w.set_dbg_stop(true);
    });
    // Have to enable DMA1 clock to keep RAM accessible for RTT during debug
    pac::RCC.ahb1enr().modify(|w| w.set_dma1en(true));

    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Configure the systick timer for 1kHz ticks at 16MHz.
    lilos::time::initialize_sys_tick(&mut cp.SYST, CLK_FREQ);

    unsafe {
        cortex_m::interrupt::enable();
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::FDCAN2_IT0);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM2);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM5);
    }
    let node = Node::init(
        NodeId::new(11).unwrap(),
        &zencan::NODE_MBOX,
        &zencan::NODE_STATE,
        &zencan::OD_TABLE,
    );
    // Use the UID register to set a unique serial number
    zencan::OBJECT1018.set_serial(get_serial());

    let node = node.finalize();

    // Create a flag to communicate status between CAN task and control task
    let operational_flag = portable_atomic::AtomicBool::new(false);
    let control_notify = Notify::new();

    defmt::info!("Starting tasks");

    lilos::exec::run_tasks(
        &mut [
            pin!(adc_task()),
            pin!(can_task(
                node,
                can_tx,
                &CAN_NOTIFY,
                &control_notify,
                &operational_flag
            )),
            pin!(control_task(
                x_step_timer,
                y_step_timer,
                x_stepper,
                y_stepper,
                &control_notify,
                &operational_flag
            )),
        ],
        lilos::exec::ALL_TASKS,
    );
}

fn zencan_to_fdcan_header(msg: &zencan_node::common::CanMessage) -> fdcan::frame::TxFrameHeader {
    let id: fdcan::id::Id = match msg.id() {
        zencan_node::common::messages::CanId::Extended(id) => {
            fdcan::id::ExtendedId::new(id).unwrap().into()
        }
        zencan_node::common::messages::CanId::Std(id) => {
            fdcan::id::StandardId::new(id).unwrap().into()
        }
    };
    fdcan::frame::TxFrameHeader {
        len: msg.dlc,
        frame_format: fdcan::frame::FrameFormat::Standard,
        id,
        bit_rate_switching: false,
        marker: None,
    }
}

/// Task to read the ISENSE adc channels
///
/// Note that these are basically junk due to design issues with the current sensing.
/// It's here waiting for a future hardware rev, and for supporting experiments.
async fn adc_task() -> Infallible {
    const ADC_INTERVAL: Millis = Millis(5);
    let mut periodic_gate = lilos::time::PeriodicGate::new_shift(ADC_INTERVAL, Millis(0));
    loop {
        periodic_gate.next_time().await;

        let adc_values = adc::read_isense();

        for (i, value) in adc_values.iter().enumerate() {
            zencan::OBJECT2000.set(i, *value).ok();
        }
    }
}

/// Task to run the zencan process
async fn can_task(
    mut node: Node,
    mut can_tx: fdcan::Tx<FdCan2, NormalOperationMode>,
    process_notify: &Notify,
    control_notify: &Notify,
    operational_flag: &AtomicBool,
) -> Infallible {
    let epoch = lilos::time::TickTime::now();
    loop {
        lilos::time::with_timeout(Duration::from_millis(10), process_notify.until_next()).await;
        let time_us = epoch.elapsed().0 * 1000;
        let objects_updated = node.process(time_us, &mut |msg| {
            let header = zencan_to_fdcan_header(&msg);
            if can_tx.transmit(header, msg.data()).is_err() {
                defmt::error!("Error transmitting CAN message");
            }
        });
        if objects_updated {
            control_notify.notify();
        }
        operational_flag.store(node.nmt_state() == NmtState::Operational, Ordering::Relaxed);
    }
}

fn run_duty_cycle_mode(x_stepper: &Stepper<'static>, y_stepper: &Stepper<'static>) {
    x_stepper.set_duty_cycles(
        zencan::OBJECT3100.get(0).unwrap(),
        zencan::OBJECT3100.get(1).unwrap(),
    );
    y_stepper.set_duty_cycles(
        zencan::OBJECT3100.get(2).unwrap(),
        zencan::OBJECT3100.get(3).unwrap(),
    );
}

fn run_stepper_mode(
    x_vel: &mut i32,
    y_vel: &mut i32,
    delta_t_ms: u32,
    x_step_timer: &mut StepTimer,
    y_step_timer: &mut StepTimer,
    x_stepper: &Stepper<'static>,
    y_stepper: &Stepper<'static>,
) {
    let x_vel_cmd = zencan::OBJECT3101.get(0).unwrap() as i32;
    let y_vel_cmd = zencan::OBJECT3101.get(1).unwrap() as i32;
    let accel = zencan::OBJECT3001.get_value();
    let power = zencan::OBJECT3002.get_value();
    if accel == 0 {
        *x_vel = x_vel_cmd;
        *y_vel = y_vel_cmd;
    } else {
        // Compute the maximum change in velocity, but never round to 0.
        let max_delta = (((delta_t_ms * accel as u32) / 1000) as i32).max(1);
        let mut x_delta = x_vel_cmd - *x_vel;
        if x_delta.abs() > max_delta {
            x_delta = x_delta.signum() * max_delta;
        }
        let mut y_delta = y_vel_cmd - *y_vel;
        if y_delta.abs() > max_delta {
            y_delta = y_delta.signum() * max_delta;
        }
        *x_vel += x_delta;
        *y_vel += y_delta;
    }

    if x_vel.abs() > MAX_STEP_FREQ {
        *x_vel = x_vel.signum() * MAX_STEP_FREQ;
    }
    if y_vel.abs() > MAX_STEP_FREQ {
        *y_vel = y_vel.signum() * MAX_STEP_FREQ;
    }

    x_stepper.set_power(power);
    y_stepper.set_power(power);

    if x_vel.abs() > MIN_STEP_FREQ || y_vel.abs() > MIN_STEP_FREQ {
        // Set the timer frequencies for the timer IRQs which will trigger steps
        let x_reverse = *x_vel < 0;
        let y_reverse = *y_vel < 0;

        x_step_timer.set_overflow_freq(x_vel.unsigned_abs());
        y_step_timer.set_overflow_freq(y_vel.unsigned_abs());
        // Set the appropriate directions
        x_stepper.enable(x_reverse);
        y_stepper.enable(y_reverse);
    } else {
        // If the step frequency is too low, just turn the current off
        x_step_timer.set_overflow_freq(0);
        y_step_timer.set_overflow_freq(0);
        x_stepper.disable();
        y_stepper.disable();
    }
}

async fn control_task(
    mut x_step_timer: StepTimer,
    mut y_step_timer: StepTimer,
    x_stepper: &Stepper<'static>,
    y_stepper: &Stepper<'static>,
    control_notify: &Notify,
    operational_flag: &AtomicBool,
) -> Infallible {
    const CONTROL_INTERVAL: Duration = Duration::from_millis(5);

    let sleep = || lilos::time::with_timeout(CONTROL_INTERVAL, control_notify.until_next());

    loop {
        // Do nothing until we enter operational mode
        while !operational_flag.load(Ordering::Relaxed) {
            sleep().await;
        }

        // Mode is latched once when transitioning to operational, and changes will not take effect
        // until you transition to PreOp and back.
        let mode = zencan::OBJECT3000.get_value();

        defmt::info!("Starting with mode = {}", mode);
        let mut x_vel = 0;
        let mut y_vel = 0;
        let mut previous_time = lilos::time::TickTime::now();
        loop {
            if mode == 0 {
                run_duty_cycle_mode(x_stepper, y_stepper);
            } else if mode == 1 {
                let now = lilos::time::TickTime::now();
                let delta_t_ms = now.millis_since(previous_time).0 as u32;
                previous_time = now;
                run_stepper_mode(
                    &mut x_vel,
                    &mut y_vel,
                    delta_t_ms,
                    &mut x_step_timer,
                    &mut y_step_timer,
                    x_stepper,
                    y_stepper,
                );
            }
            sleep().await;
            if !operational_flag.load(Ordering::Relaxed) {
                x_stepper.disable();
                y_stepper.disable();
                break;
            }
        }
    }
}

#[pac::interrupt]
unsafe fn FDCAN2_IT0() {
    // safety: Accept for during boot-up when we set it, we only access in this interrupt
    let ctrl = unsafe { &mut *CAN_CTRL.get() };
    let rx = unsafe { &mut *CAN_RX.get() };

    ctrl.clear_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg);

    let mut buffer = [0u8; 8];

    while let Ok(msg) = rx.receive(&mut buffer) {
        // ReceiveOverrun::unwrap() cannot fail
        let msg = msg.unwrap();

        let id = match msg.id {
            fdcan::id::Id::Standard(standard_id) => {
                zencan_node::common::messages::CanId::std(standard_id.as_raw())
            }
            fdcan::id::Id::Extended(extended_id) => {
                zencan_node::common::messages::CanId::extended(extended_id.as_raw())
            }
        };
        let msg = zencan_node::common::messages::CanMessage::new(id, &buffer[..msg.len as usize]);
        // Ignore error -- an Err is returned for messages that are not consumed by the node
        zencan::NODE_MBOX.store_message(msg).ok();
    }
}

#[pac::interrupt]
fn TIM2() {
    // Clear all interrupt flags
    pac::TIM2.sr().write(|w| w.0 = 0);

    unsafe {
        (*X_STEPPER.get()).step();
    }
}

#[pac::interrupt]
fn TIM5() {
    // Clear all interrupt flags
    pac::TIM5.sr().write(|w| w.0 = 0);

    unsafe {
        (*Y_STEPPER.get()).step();
    }
}
