use std::f32::consts::PI;

pub struct Motor(pub f32);

impl Motor {
    pub const WHEEL: Motor = Motor(2.0 * PI / (4.0 * 400.0 * 20.0));
    pub const RUDDER: Motor = Motor(2.0 * PI / 16384.0);

    pub fn pluses_to_rad(&self, pulses: i32) -> f32 {
        (pulses as f32) * self.0
    }

    pub fn rad_to_pulses(&self, rad: f32) -> i32 {
        (rad as f32 / self.0).round() as i32
    }
}
