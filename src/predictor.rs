use crate::{optimizer::Optimizer, Physical};
use std::time::Duration;

pub struct Predictor {
    delta_rudder: f32,    // 后轮最大步进量
    optimizer: Optimizer, // 优化器
    current: Physical,    // 当前状态
    target: Physical,     // 目标状态
}

impl Predictor {
    pub fn new(
        w_rudder: f32,
        angular_attenuation: f32,
        acceleration: f32,
        period: Duration,
    ) -> Self {
        Self {
            delta_rudder: w_rudder * period.as_secs_f32(),
            optimizer: Optimizer::new(angular_attenuation, acceleration, period),
            current: Physical::ZERO,
            target: Physical::ZERO,
        }
    }

    pub fn reset(&mut self) {
        self.current = Physical::ZERO;
        self.target = Physical::ZERO;
    }

    pub fn set_target(&mut self, target: Physical) {
        self.target = target;
    }
}

impl Iterator for Predictor {
    type Item = Physical;

    fn next(&mut self) -> Option<Self::Item> {
        if self.current != self.target {
            self.current = Physical {
                speed: self.optimizer.optimize_speed(self.target, self.current),
                rudder: if self.target.rudder > self.current.rudder {
                    f32::min(self.current.rudder + self.delta_rudder, self.target.rudder)
                } else {
                    f32::max(self.current.rudder - self.delta_rudder, self.target.rudder)
                },
            };
            Some(self.current)
        } else {
            None
        }
    }
}
