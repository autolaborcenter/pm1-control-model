use crate::{optimizer::Optimizer, Physical};

#[derive(Clone)]
pub struct Predictor {
    pub rudder_step: f32,     // 后轮最大步进量
    pub optimizer: Optimizer, // 优化器
    pub current: Physical,    // 当前状态
    pub target: Physical,     // 目标状态
}

impl Iterator for Predictor {
    type Item = Physical;

    fn next(&mut self) -> Option<Self::Item> {
        if self.current != self.target {
            self.current = Physical {
                speed: self.optimizer.optimize_speed(self.target, self.current),
                rudder: if self.target.rudder > self.current.rudder {
                    f32::min(self.current.rudder + self.rudder_step, self.target.rudder)
                } else {
                    f32::max(self.current.rudder - self.rudder_step, self.target.rudder)
                },
            };
        }
        Some(self.current)
    }
}
