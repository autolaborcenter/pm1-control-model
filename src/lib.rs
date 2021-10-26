//! 模块用于控制底盘运动，计算里程计。
//!
//! 包括底盘模块（ChassisModel），里程计模块（Odometery），电机模块（Motor），最大速度优化器（Optimizer），控制量预测器（Preditor）。

mod model;
mod optimizer;

pub use model::ChassisModel;
pub use optimizer::Optimizer;

#[cfg(feature = "odometry")]
pub extern crate nalgebra as na;

#[cfg(feature = "odometry")]
mod odometry;

#[cfg(feature = "odometry")]
pub use odometry::Odometry;

#[cfg(feature = "motor")]
mod motor;

#[cfg(feature = "motor")]
pub use motor::Motor;

#[cfg(feature = "predict")]
mod predictor;

#[cfg(feature = "predict")]
pub use predictor::Predictor;

/// 三轮阿卡曼模型
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(C)]
pub struct Physical {
    /// 机器人上各轮相对地面的最大线速度
    pub speed: f32,
    /// 后轮转角
    pub rudder: f32,
}

///两轮差动模型
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Wheels {
    /// 左轮角速度
    pub left: f32,
    /// 右轮角速度
    pub right: f32,
}

///里程计模型
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Velocity {
    /// 旋转中心相对地面线速度
    pub v: f32,
    /// 旋转中心相对地面角速度
    pub w: f32,
}

impl Physical {
    pub const RELEASED: Physical = Physical {
        speed: 0.0,
        rudder: f32::NAN,
    };

    pub const ZERO: Physical = Physical {
        speed: 0.0,
        rudder: 0.0,
    };
}
