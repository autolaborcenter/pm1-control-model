use crate::{
    optimizer::{self, Optimizer},
    ChassisModel, Odometry, Physical,
};
use std::time::Duration;

/// 状态预测器
///
/// 利用给定的机器人参数，根据当前状态和目标状态预测下一周期机器人的状态。
#[derive(Clone)]
pub struct StatusPredictor {
    rudder_step: f32, // 后轮最大步进量，单位 rad
    optimizer: Optimizer,
    /// 当前状态
    pub current: Physical,
    /// 目标状态
    pub target: Physical,
}

impl StatusPredictor {
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

impl Iterator for StatusPredictor {
    type Item = Physical;

    /// 预测下一周期以 [`Physical`] 表示的状态
    fn next(&mut self) -> Option<Self::Item> {
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

#[test]
fn test_status_predictor() {
    // 打印出来看看
    const PERIOD: Duration = Duration::from_millis(40);
    let mut pre = StatusPredictor::new(Optimizer::new(0.5, 1.2, PERIOD), PERIOD);
    pre.current = Physical {
        speed: 0.4,
        rudder: 0.0,
    };
    pre.target = Physical::RELEASED;
    for s in pre {
        println!("{:?}", s);
        std::thread::sleep(Duration::from_millis(500));
    }
}

/// 轨迹预测器
///
/// 利用给定的机器人参数，根据当前状态和目标状态预测一个周期中机器人里程的增量
#[derive(Clone)]
pub struct TrajectoryPredictor {
    pub period: Duration,           // 控制周期
    pub model: ChassisModel,        // 机器人模型
    pub predictor: StatusPredictor, // 状态预测器
}

impl Iterator for TrajectoryPredictor {
    type Item = (Duration, Odometry);

    /// 预测下一周期以 [`Duration`] 表示的时间**增量**和以 [`Odometry`] 表示的里程**增量**
    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        self.predictor.next().map(|Physical { speed, rudder }| {
            (
                self.period,
                self.model.physical_to_odometry(Physical {
                    speed: speed * self.period.as_secs_f32(),
                    rudder,
                }),
            )
        })
    }
}
