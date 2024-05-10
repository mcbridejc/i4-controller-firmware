use crate::current_control::IChannel;

pub struct XYDrive<'a> {
    x1: IChannel<'a>,
    x2: IChannel<'a>,
    y1: IChannel<'a>,
    y2: IChannel<'a>,
}

pub enum Mode {
    Off,
    On
}

impl<'a> XYDrive<'a> {
    pub fn new(x1: IChannel<'a>, x2: IChannel<'a>, y1: IChannel<'a>, y2: IChannel<'a>) -> Self {
        Self { x1, x2, y1, y2 }
    }

    pub fn set_mode() {

    }

}