use crate::pac;

pub struct StepTimer {
    tim: pac::timer::TimGp32,
    tick_freq: u32,
}

impl StepTimer {
    pub fn new(tim: pac::timer::TimGp32, tick_freq: u32) -> Self {
        tim.cr1().modify(|w| {
            w.set_arpe(true);
            w.set_cen(true);
        });

        let mut obj = Self { tim, tick_freq };

        obj.set_overflow_freq(10);
        obj.tim.egr().write(|w| w.set_ug(true));
        obj
    }

    pub fn set_overflow_freq(&mut self, ovf_freq: u32) {
        if ovf_freq < 4 {
            // Too slow; just turn off the timer
            self.disable_irq();
        } else {
            let arr = self.tick_freq / ovf_freq;
            self.enable_irq();
            self.tim.arr().write(|w| w.set_arr(arr));
        }
    }

    #[inline]
    pub fn enable_irq(&mut self) {
        self.tim.dier().modify(|w| w.set_uie(true));
    }

    #[inline]
    pub fn disable_irq(&mut self) {
        self.tim.dier().modify(|w| w.set_uie(false));
    }
}
