//! 模块用于控制底盘运动，计算里程计。
//!
//! 包括底盘模块（ChassisModel），里程计模块（Odometery），电机模块（Motor），最大速度优化器（Optimizer），控制量预测器（Preditor）。
//!
//! 机器人的控制、解算、预测等算法都建立在类阿卡曼物理模型 [`Physical`]、差动模型 [`Wheels`] 和刚体速度模型 [`Velocity`] 之上。

mod model;
mod optimizer;
mod predict;

pub use chassis::*;
pub use model::Pm1Model;
pub use optimizer::Optimizer;
pub use predict::Pm1Predictor;

#[cfg(feature = "motor")]
mod motor;

#[cfg(feature = "motor")]
pub use motor::Motor;

/// 类阿卡曼物理模型
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(C)]
pub struct Physical {
    /// 机器人上各轮相对地面的最大线速度，单位 m/s，
    /// 理论范围 [-1.0, 1.0]，根据具体性能可能有变化
    pub speed: f32,
    /// 后轮转角（弧度），[-π/2, π/2]
    pub rudder: f32,
}

/// 两轮差动模型
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Wheels {
    /// 左轮角速度 rad/s
    pub left: f32,
    /// 右轮角速度 rad/s
    pub right: f32,
}

impl Physical {
    /// 完全静止
    pub const RELEASED: Physical = Physical {
        speed: 0.0,
        rudder: f32::NAN,
    };

    /// 零状态
    ///
    /// 用于初始化不支持 `RELEASE` 状态的变量
    pub const ZERO: Physical = Physical {
        speed: 0.0,
        rudder: 0.0,
    };

    /// 静止
    #[inline]
    pub fn is_static(&self) -> bool {
        self.speed == 0.0
    }

    /// 后轮不受控
    #[inline]
    pub fn is_released(&self) -> bool {
        self.rudder.is_nan()
    }
}

impl std::ops::Mul<f32> for Physical {
    type Output = Self;

    #[inline]
    fn mul(mut self, rhs: f32) -> Self::Output {
        self.speed *= rhs;
        self
    }
}

impl std::ops::Mul<f32> for Wheels {
    type Output = Self;

    #[inline]
    fn mul(mut self, rhs: f32) -> Self::Output {
        self.left *= rhs;
        self.right *= rhs;
        self
    }
}
