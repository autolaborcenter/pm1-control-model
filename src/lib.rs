pub mod model;
pub mod motor;
pub mod optimizer;
pub mod predictor;

/// - `speed`: 机器人上各轮相对地面的最大线速度
/// - `rudder`: 后轮转角
#[derive(Clone, Copy, Debug)]
pub struct Physical {
    pub speed: f32,
    pub rudder: f32,
}

/// - `left`: 左轮角速度
/// - `right`: 右轮角速度
#[derive(Clone, Copy, Debug)]
pub struct Wheels {
    pub left: f32,
    pub right: f32,
}

/// - `v`: 旋转中心相对地面线速度
/// - `w`: 旋转中心相对地面角速度
#[derive(Clone, Copy, Debug)]
pub struct Velocity {
    pub v: f32,
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
