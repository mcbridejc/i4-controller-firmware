use crate::pwm::{Pwm, PwmChan};



pub struct IChannel<'a> {
    in1: PwmChan<'a>,
    in2: PwmChan<'a>,
}

impl<'a> IChannel<'a> {
    pub fn new(in1: PwmChan<'a>, in2: PwmChan<'a>) -> Self {
        Self { in1, in2 }
    }

    pub fn set_duty_cycle(&self, duty: i16) {
        let high_side_duty = (duty as i32).abs() as u16 * 2;
        if duty > 0 {
            self.in1.set_duty(high_side_duty);
            self.in2.set_duty(0);
        } else {
            self.in1.set_duty(0);
            self.in2.set_duty(high_side_duty);
        }
    }
}