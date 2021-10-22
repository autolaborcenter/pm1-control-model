use crate::Physical;
use std::{
    f32::consts::{FRAC_PI_2, FRAC_PI_3, FRAC_PI_6},
    time::Duration,
};

#[derive(Clone, Copy, Debug)]
pub struct Optimizer {
    angular_attenuation: f32,
    delta_speed: f32,
}

impl Optimizer {
    pub fn new(angular_attenuation: f32, acceleration: f32, period: Duration) -> Self {
        Self {
            angular_attenuation,
            delta_speed: acceleration * period.as_secs_f32(),
        }
    }

    pub fn optimize_speed(&self, target: Physical, current: Physical) -> f32 {
        let mut speed = target.speed;
        if !target.rudder.is_nan() {
            // 当前速度越快越允许后轮不吻合
            let width = current.speed * FRAC_PI_3 + FRAC_PI_6;
            let diff = (target.rudder - current.rudder).abs();
            speed *=
            // 基于性能的限速：后轮转速有限
            f32::max(0.0,1.0 - diff / width) *
            // 基于现象的限速：转弯不要太快
            ((1.0 - target.rudder.abs() / FRAC_PI_2) * (1.0 - self.angular_attenuation) + self.angular_attenuation);
        }
        // 基于现象的限速：加速不要太快
        if speed > current.speed {
            f32::min(current.speed + self.delta_speed, speed)
        } else {
            f32::max(current.speed - self.delta_speed, speed)
        }
    }
}
