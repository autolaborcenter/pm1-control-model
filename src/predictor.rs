use crate::{optimizer::Optimizer, Physical};
use std::time::Duration;

pub struct Predictor {
    delta_rudder: f32, // 后轮最大步进量
    period: Duration,
    optimizer: Optimizer,
    time: Duration,
    current: Physical,
    target: Physical,
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
            period,
            optimizer: Optimizer::new(angular_attenuation, acceleration, period),
            time: Duration::ZERO,
            current: Physical::ZERO,
            target: Physical::ZERO,
        }
    }

    pub fn reset(&mut self) {
        self.time = Duration::ZERO;
        self.current = Physical::ZERO;
        self.target = Physical::ZERO;
    }

    pub fn set_target(&mut self, target: Physical) {
        self.target = target;
    }
}

impl Iterator for Predictor {
    type Item = (Duration, Physical);

    fn next(&mut self) -> Option<Self::Item> {
        self.time += self.period;
        self.current = Physical {
            speed: self.optimizer.optimize_speed(self.target, self.current),
            rudder: if self.target.rudder > self.current.rudder {
                f32::min(self.current.rudder + self.delta_rudder, self.target.rudder)
            } else {
                f32::max(self.current.rudder - self.delta_rudder, self.target.rudder)
            },
        };
        Some((self.time, self.current))
    }
}
