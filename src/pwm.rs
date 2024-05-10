use stm32_metapac::timer::regs::{CcmrOutput, Ccr16};
use stm32_metapac::timer::{vals, TimGp16};
use stm32_metapac::common::{Reg, RW};

use crate::pac::timer::TimAdv;

pub trait Pwm {
    fn set_duty(&self, duty: u16);
    fn set_high(&self);
    fn set_low(&self);
}

pub struct PwmTimer<T> {
    tim: T
}

trait MultiPwm: Send + Sync {
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

#[allow(dead_code)]
impl PwmTimer<TimAdv> {
    pub fn new(tim: TimAdv, pwm_frequency: u32, clock_frequency: u32) -> Self {
        // timer input clock ticks per PWM period
        let ticks = clock_frequency / pwm_frequency / 2;
        // Compute the prescaler to allow achieve the frequency with a 16-bit counter
        let psc = ((ticks - 1) / (1 << 16)) as u16;
        tim.psc().write(|w| w.set_psc(psc));

        // Compute ARR to achieve the precise frequency
        let arr = (ticks / (psc + 1) as u32) as u16;
        tim.arr().write(|w| w.set_arr(arr));

        defmt::info!("ARR: {}", arr);

        tim.cr1().modify(|w| w.set_cms(vals::Cms::CENTERALIGNED3));

        // Enable all output channels
        for ccmr_n in 0..2 {
            for ch_n in 0..2 {
                tim.ccmr_output(ccmr_n).modify(|w| {
                    w.set_ocm(ch_n, vals::Ocm::PWMMODE1);
                    w.set_ocpe(ch_n, true)
                });
            }
        }

        // Missing from metapac
        // Turn on CC5
        let ccmr3: Reg<CcmrOutput, RW> = unsafe { Reg::from_ptr((tim.as_ptr() as *mut u8).add(0x50) as _) };
        ccmr3.modify(|w| {
            w.set_ocm(0, vals::Ocm::TOGGLE);
            w.set_ocpe(0, true);
        });

        for n in 0..4 {
            // capture/compare output enable
            tim.ccer().modify(|w| w.set_cce(n, true));
        }
        // Master out enable
        tim.bdtr().modify(|w| w.set_moe(true));

        // Setup CCR5 as trgo2 to trigger ADC
        let ccr5: Reg<Ccr16, RW> = unsafe { Reg::from_ptr((tim.as_ptr() as *mut u8).add(0x48) as _) };
        //ccr5.write(|w| w.set_ccr(arr));
        ccr5.write(|w| w.set_ccr(30));

        // // Set CC5E bit, which is not in metapac
        // tim.ccer().modify(|w| w.0 |= 1<<16);

        // CR2 MMS register definitions are broken so hack it a bit
        //tim.cr2().modify(|w| w.0 |= 0b1000<<20);
        tim.cr2().modify(|w| w.0 |= 0b1000<<20);
        Self { tim }
    }

    #[inline]
    pub fn enable(&self) {
        // Enable timer
        self.tim.cr1().modify(|w| w.set_cen(true));
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
        let reload = duty as u32 * self.tim.arr().read().arr() as u32 / 65536;
        let ccmr_reg = (ch / 2) as usize;
        let ccmr_ch = (ch % 2) as usize;
        // Set duty cycle
        self.tim.ccr(ch as usize).write(|w| w.set_ccr(reload as u16));
        // Set to PWM mode
        self.tim.ccmr_output(ccmr_reg).modify(|w| w.set_ocm(ccmr_ch, vals::Ocm::PWMMODE1));
    }

    fn set_high(&self, ch: u8) {
        assert!(ch < 4);
        let reg = (ch / 2) as usize;
        let ch = (ch % 2) as usize;
        self.tim.ccmr_output(reg).modify(|w| w.set_ocm(ch, vals::Ocm::FORCEACTIVE))
    }
}


#[allow(dead_code)]
// Implementation for general purpose 16-bit timers. It's *almost* identical, but not quite.
impl PwmTimer<TimGp16> {
    pub fn new(tim: TimGp16, pwm_frequency: u32, clock_frequency: u32) -> Self {
        // timer input clock ticks per PWM period
        let ticks = clock_frequency / pwm_frequency / 2;
        // Compute the prescaler to allow achieve the frequency with a 16-bit counter
        let psc = ((ticks - 1) / (1 << 16)) as u16;
        tim.psc().write(|w| w.set_psc(psc));

        // Compute ARR to achieve the precise frequency
        let arr = (ticks / (psc + 1) as u32) as u16;
        tim.arr().write(|w| w.set_arr(arr));


        tim.cr1().modify(|w| w.set_cms(vals::Cms::CENTERALIGNED3));

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

        Self { tim }
    }

    #[inline]
    pub fn enable(&self) {
        // Enable timer
        self.tim.cr1().modify(|w| w.set_cen(true));
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