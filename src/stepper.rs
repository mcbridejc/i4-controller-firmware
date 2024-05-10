// TODO: Would like to build this table and make NSTEPS a const generic, but need to get const trig functions working first.
// const fn sin_table_u16<const N: usize>() -> [u16; N] {
//     let mut i = 0;
//     let lut = [0; N];
//     while i < N {
//         let x = (2.0 * core::f32::consts::PI * i as f32 / N as f32).radians();
//         let y = const_trig::sin(x, None);
//         lut[i] = (32767.0 * y) as i16;
//     }
//     lut
// }

use core::{cell::Cell, sync::atomic::{AtomicBool, AtomicU16, Ordering}};

use critical_section::Mutex;
use crossbeam::atomic::AtomicCell;

use crate::current_control::IChannel;

pub struct BipolarMicrostepper {
    scaled_lut: [u16; 4],
}

impl BipolarMicrostepper {
    // NSTEPS must be divisible by 4
    const NSTEPS: usize = 16;
    // partial lookup table for sin function; takes advantage of symmetry (N must be odd)
    const LUT: [u16; Self::NSTEPS/4] = [12539, 23169, 30272, 32767];

    pub fn new() -> Self {
        let mut scaled_lut = [0; 4];

        for i in 0..Self::LUT.len() {
            scaled_lut[i] = Self::LUT[i];
        }

        Self {
            scaled_lut
        }
    }

    pub fn set_scale(&mut self, full_scale: u16) {
        for i in 0..Self::LUT.len() {
            self.scaled_lut[i] = ((Self::LUT[i] as i32 * full_scale as i32) / 32768) as u16;
        }
    }

    pub fn scale(&self) -> u16 {
        self.scaled_lut[Self::NSTEPS/4 - 1]
    }

    fn lookup(&self, idx: usize) -> i32 {
        let negate = idx >= Self::NSTEPS / 2;
        let mod_index = idx % (Self::NSTEPS / 2);

        // In order to shrink the table, we take advantage of some symmetry.
        // This works as long as NSTEPS % 4 == 0.
        // First, the second half of the sine function is the negative of the first.
        // Second, the first sample is always 0, so we don't store it.
        // Finally, after the peak, there's another mirroring.
        // Does this result in smaller flash? For small tables, maybe not. The increased code size
        // could outweigh the benefit? I haven't checked.

        let mut lut_value = if mod_index == 0 {
            0
        } else if mod_index <= Self::NSTEPS / 4 {
            self.scaled_lut[mod_index - 1] as i32
        } else {
            self.scaled_lut[Self::NSTEPS / 2 - mod_index - 1] as i32
        };
        if negate {
            lut_value = -lut_value;
        }

        lut_value
    }

    pub fn get(&self, step: usize) -> (i32, i32) {
        assert!(step < Self::NSTEPS);
        let a = self.lookup(step);
        let b = self.lookup((step + Self::NSTEPS / 4) % Self::NSTEPS);
        (a, b)
    }

    pub fn nsteps(&self) -> usize {
        Self::NSTEPS
    }
}

#[derive(Clone, Copy, Debug)]
pub enum Mode {
    Off,
    Forward,
    Reverse,
}

pub struct Stepper<'a> {
    stepper: BipolarMicrostepper,
    phase_a: Mutex<IChannel<'a>>,
    phase_b: Mutex<IChannel<'a>>,
    mode: AtomicCell<Mode>,
    stepper_pos: Mutex<Cell<u16>>,
}

impl<'a> Stepper<'a> {

    pub fn new(phase_a: IChannel<'a>, phase_b: IChannel<'a>) -> Self {
        let stepper = BipolarMicrostepper::new();

        Self {
            stepper,
            phase_a: Mutex::new(phase_a),
            phase_b: Mutex::new(phase_b),
            mode: AtomicCell::new(Mode::Off),
            stepper_pos: Mutex::new(Cell::new(0)),
        }
    }

    pub fn disable(&self) {
        critical_section::with(|cs| {
            let a = self.phase_a.borrow(cs);
            let b = self.phase_b.borrow(cs);
            a.set_duty_cycle(0);
            b.set_duty_cycle(0);
            self.mode.store(Mode::Off);
        });
    }

    pub fn enable(&self, reverse: bool) {
        let mode = if reverse { Mode::Reverse } else { Mode::Forward };
        self.mode.store(mode);
    }

    pub fn step(&self) {

        const POWER: i32 = 25000;

        let reverse = match self.mode.load() {
            Mode::Off => return,
            Mode::Forward => false,
            Mode::Reverse => true,
        };

        let mut new_pos = 0;
        critical_section::with(|cs| {
            let stepper_pos = self.stepper_pos.borrow(cs).get();
            new_pos = if reverse {
                stepper_pos.overflowing_sub(1).0
            } else {
                stepper_pos.overflowing_add(1).0
            };
            new_pos = new_pos % self.stepper.nsteps() as u16;
            self.stepper_pos.borrow(cs).set(new_pos);
        });

        let (a, b) = self.stepper.get(new_pos as usize);
        critical_section::with(|cs| {
            let phase_a = self.phase_a.borrow(cs);
            let phase_b = self.phase_b.borrow(cs);
            phase_a.set_duty_cycle((a * POWER / 32768) as i16);
            phase_b.set_duty_cycle((b * POWER / 32768) as i16);
        });
    }
}
