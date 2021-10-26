use std::f32::consts::PI;
/// 电机模型
///
/// 用于转换编码器测到的脉冲数和轮转过的弧度。

pub struct Motor(
    /// 编码器每脉冲转过的弧度
    pub f32,
);

impl Motor {
    /// 驱动轮模型
    ///
    /// 脉冲数/圈 = 400 x 4 倍频 x 20 倍减速
    pub const WHEEL: Motor = Motor(2.0 * PI / (4.0 * 400.0 * 20.0));

    /// 舵轮模型
    ///
    /// 脉冲数/圈 = 16384
    pub const RUDDER: Motor = Motor(2.0 * PI / 16384.0);

    /// 脉冲数 -> 弧度
    pub fn pluses_to_rad(&self, pulses: i32) -> f32 {
        (pulses as f32) * self.0
    }

    /// 弧度 -> 脉冲数
    pub fn rad_to_pulses(&self, rad: f32) -> i32 {
        (rad as f32 / self.0).round() as i32
    }
}
