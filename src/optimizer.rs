use crate::Physical;
use std::{
    f32::consts::{FRAC_PI_2, FRAC_PI_3, FRAC_PI_6},
    time::Duration,
};

/// 针对最大速度的优化器
#[derive(Clone, Copy, Debug)]
pub struct Optimizer {
    angular_attenuation: f32, // 舵转角衰减因子
    speed_step: f32,          // 每个控制周期速度的最大步进量（m/s）
}

impl Optimizer {
    /// - `angular_attenuation`: 舵转角衰减因子，用于额外限制转身速度。取值限制在 [0,1]，越大转身越慢
    /// - `acceleration`: 加速度，单位 m/s^2
    /// - `period`: 参考控制周期，单位 s
    #[inline]
    pub fn new(angular_attenuation: f32, acceleration: f32, period: Duration) -> Self {
        Self {
            angular_attenuation,
            speed_step: acceleration * period.as_secs_f32(),
        }
    }

    /// 优化速度
    ///
    /// 从当前状态 `current` 出发，根据优化参数 `self` 逼近目标状态 `target`。
    ///
    /// 返回下一个控制周期的目标速度。
    pub fn optimize_speed(&self, target: Physical, current: Physical) -> f32 {
        let mut speed = target.speed;
        if !target.rudder.is_nan() {
            // 当前速度越快越允许后轮不吻合
            // 经验参数，最高速时 width = π/2，此时斜率最小；最低速时，width = π/6，此时斜率最大
            let width = current.speed * FRAC_PI_3 + FRAC_PI_6;
            let diff = (target.rudder - current.rudder).abs();
            speed *=
            // 基于性能的限速：后轮转速有限
            f32::max(0.0,1.0 - diff / width) *
            // 基于现象的限速：转弯不要太快
            ((1.0 - target.rudder.abs() / FRAC_PI_2) * (1.0 - self.angular_attenuation) + self.angular_attenuation);
        }
        // 基于现象的限速：加速不要太快
        step_limited(current.speed, self.speed_step, speed)
    }
}

/// 有限步进
///
/// 从当前状态 `current` 向目标状态 `target` 逼近，但最多变化 step
///
/// `step` 不能小于 0。
#[inline]
pub(crate) fn step_limited(current: f32, step: f32, target: f32) -> f32 {
    use std::cmp::Ordering::*;
    match target.partial_cmp(&current) {
        Some(Equal) | None => current,
        Some(Less) => f32::max(current - step, target),
        Some(Greater) => f32::min(current + step, target),
    }
}
