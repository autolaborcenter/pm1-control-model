use crate::{
    optimizer::{self, Optimizer},
    Physical, Pm1Model,
};
use chassis::StatusPredictor;
use std::time::Duration;

/// 状态预测器
///
/// 利用给定的机器人参数，根据当前状态和目标状态预测下一周期机器人的状态。
#[derive(Clone)]
pub struct Pm1Predictor {
    rudder_step: f32, // 后轮最大步进量，单位 rad
    optimizer: Optimizer,
    /// 当前状态
    pub current: Physical,
    /// 目标状态
    pub target: Physical,
}

impl StatusPredictor for Pm1Predictor {
    type Model = Pm1Model;

    fn predict(&mut self) -> Option<<Self::Model as chassis::ChassisModel>::State> {
        // 目标是停住且已经停住
        if self.target.is_released() && self.current.is_static() {
            None
        }
        // 未达到目标状态
        else {
            if self.current != self.target {
                self.current = Physical {
                    speed: self.optimize_speed(),
                    rudder: self.optimize_rudder(),
                };
            }
            Some(self.current)
        }
    }
}

impl Pm1Predictor {
    /// 给定底盘实际使用的优化器和控制周期，生成状态预测器
    pub fn new(optimizer: Optimizer, period: Duration) -> Self {
        Self {
            rudder_step: period.as_secs_f32(),
            optimizer,
            current: Physical::ZERO,
            target: Physical::RELEASED,
        }
    }

    #[inline]
    fn optimize_speed(&self) -> f32 {
        self.optimizer.optimize_speed(self.target, self.current)
    }

    #[inline]
    fn optimize_rudder(&self) -> f32 {
        optimizer::step_limited(self.current.rudder, self.rudder_step, self.target.rudder)
    }
}

#[test]
fn test_status_predictor() {
    // 打印出来看看
    const PERIOD: Duration = Duration::from_millis(40);
    let mut pre = Pm1Predictor::new(Optimizer::new(0.5, 1.2, PERIOD), PERIOD);
    pre.current = Physical {
        speed: 0.4,
        rudder: 0.0,
    };
    pre.target = Physical::RELEASED;
    while let Some(s) = pre.predict() {
        println!("{:?}", s);
        std::thread::sleep(Duration::from_millis(500));
    }
}
