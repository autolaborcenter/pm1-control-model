use crate::{optimizer::Optimizer, Physical};

/// 预测器，用于给出下一步控制量的大小
///
///     结合当前状态和目标状态，以及后轮最大步进量，给出下一步的最大速度及轮转角。
#[derive(Clone)]
pub struct Predictor {
    /// 后轮最大步进量，单位rad
    pub rudder_step: f32,
    /// 最大速度优化器
    pub optimizer: Optimizer,
    /// 当前状态
    pub current: Physical,
    /// 目标状态
    pub target: Physical,
}

impl Iterator for Predictor {
    type Item = Physical;
    /// 获取下一步（Physical）控制量
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
