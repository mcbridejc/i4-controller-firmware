use stm32_metapac::timer::{regs::Psc, vals, TimGp16};

use crate::pac::timer::TimAdv;

pub trait Pwm {
    fn set_duty(&self, duty: u16);
    fn set_high(&self);
    fn set_low(&self);
}

pub struct PwmTimer<T> {
    tim: T
}

trait MultiPwm {
    fn set_duty(&self, ch: u8, duty: u16);
    fn set_high(&self, ch: u8);
}

pub struct PwmChan<'a> {
    ch: u8,
    timer: &'a dyn MultiPwm,
}

impl<'a> Pwm for PwmChan<'a> {
    fn set_duty(&self, duty: u16) {
        self.timer.set_duty(self.ch, duty);
    }

    fn set_high(&self) {
        self.timer.set_high(self.ch);
    }

    fn set_low(&self) {
        todo!()
    }
}

impl PwmTimer<TimAdv> {
    pub fn new(tim: TimAdv, pwm_frequency: u32, clock_frequency: u32) -> Self {
        // timer input clock ticks per PWM period
        let ticks = clock_frequency / pwm_frequency;
        // Compute the prescaler to allow achieve the frequency with a 16-bit counter
        let psc = ((ticks - 1) / (1 << 16)) as u16;
        tim.psc().write(|w| w.set_psc(psc));

        // Compute ARR to achieve the precise frequency
        let arr = (ticks / (psc + 1) as u32) as u16;
        tim.arr().write(|w| w.set_arr(arr));

        // Enable all output channels

        for ccmr_n in 0..2 {
            for ch_n in 0..2 {
                tim.ccmr_output(ccmr_n).modify(|w| {
                    w.set_ocm(ch_n, vals::Ocm::PWMMODE1);
                    w.set_ocpe(ch_n, true)
                });
            }
        }
        for n in 0..4 {
            // capture/compare output enable
            tim.ccer().modify(|w| w.set_cce(n, true));
        }
        // Master out enable
        tim.bdtr().modify(|w| w.set_moe(true));

        // Enable timer
        tim.cr1().modify(|w| w.set_cen(true));

        Self { tim }
    }

    pub fn ch1<'a>(&'a self) -> PwmChan<'a> {
        PwmChan { ch: 0, timer: self }
    }

    pub fn ch2<'a>(&'a self) -> PwmChan<'a> {
        PwmChan { ch: 1, timer: self }
    }

    pub fn ch3<'a>(&'a self) -> PwmChan<'a> {
        PwmChan { ch: 2, timer: self }
    }

    pub fn ch4<'a>(&'a self) -> PwmChan<'a> {
        PwmChan { ch: 3, timer: self }
    }
}

impl MultiPwm for PwmTimer<TimAdv> {
    fn set_duty(&self, ch: u8, duty: u16) {
        assert!(ch < 4);

        // Set duty cycle
        let reload = duty as u32 * self.tim.arr().read().arr() as u32 / 65536;
        self.tim.ccr(ch as usize).write(|w| w.set_ccr(reload as u16));

        // Set to PWM mode
        let ccmr_reg = (ch / 2) as usize;
        let ccmr_ch = (ch % 2) as usize;
        self.tim.ccmr_output(ccmr_reg).modify(|w| w.set_ocm(ccmr_ch, vals::Ocm::PWMMODE1));
    }

    fn set_high(&self, ch: u8) {
        assert!(ch < 4);
        let reg = (ch / 2) as usize;
        let ch = (ch % 2) as usize;
        self.tim.ccmr_output(reg).modify(|w| w.set_ocm(ch, vals::Ocm::FORCEACTIVE))
    }
}


// Implementation for general purpose 16-bit timers. It's *almost* identical, but not quite.
impl PwmTimer<TimGp16> {
    pub fn new(tim: TimGp16, pwm_frequency: u32, clock_frequency: u32) -> Self {
        // timer input clock ticks per PWM period
        let ticks = clock_frequency / pwm_frequency;
        // Compute the prescaler to allow achieve the frequency with a 16-bit counter
        let psc = ((ticks - 1) / (1 << 16)) as u16;
        tim.psc().write(|w| w.set_psc(psc));

        // Compute ARR to achieve the precise frequency
        let arr = (ticks / (psc + 1) as u32) as u16;
        tim.arr().write(|w| w.set_arr(arr));

        // Enable all output channels

        for ccmr_n in 0..2 {
            for ch_n in 0..2 {
                tim.ccmr_output(ccmr_n).modify(|w| {
                    w.set_ocm(ch_n, vals::Ocm::PWMMODE1);
                    w.set_ocpe(ch_n, true)
                });
            }
        }
        for n in 0..4 {
            // capture/compare output enable
            tim.ccer().modify(|w| w.set_cce(n, true));
        }

        // Enable timer
        tim.cr1().modify(|w| w.set_cen(true));

        Self { tim }
    }

    pub fn ch1<'a>(&'a self) -> PwmChan<'a> {
        PwmChan { ch: 0, timer: self }
    }

    pub fn ch2<'a>(&'a self) -> PwmChan<'a> {
        PwmChan { ch: 1, timer: self }
    }

    pub fn ch3<'a>(&'a self) -> PwmChan<'a> {
        PwmChan { ch: 2, timer: self }
    }

    pub fn ch4<'a>(&'a self) -> PwmChan<'a> {
        PwmChan { ch: 3, timer: self }
    }
}

impl MultiPwm for PwmTimer<TimGp16> {
    fn set_duty(&self, ch: u8, duty: u16) {
        assert!(ch < 4);

        // Set duty cycle
        let reload = duty as u32 * self.tim.arr().read().arr() as u32 / 65536;
        self.tim.ccr(ch as usize).write(|w| w.set_ccr(reload as u16));

        // Set to PWM mode
        let ccmr_reg = (ch / 2) as usize;
        let ccmr_ch = (ch % 2) as usize;
        self.tim.ccmr_output(ccmr_reg).modify(|w| w.set_ocm(ccmr_ch, vals::Ocm::PWMMODE1));
    }

    fn set_high(&self, ch: u8) {
        assert!(ch < 4);
        let reg = (ch / 2) as usize;
        let ch = (ch % 2) as usize;
        self.tim.ccmr_output(reg).modify(|w| w.set_ocm(ch, vals::Ocm::FORCEACTIVE))
    }
}