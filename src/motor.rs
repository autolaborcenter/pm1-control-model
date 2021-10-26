use std::f32::consts::PI;
/// 电机模型
/// 
/// 分为两种：驱动轮电机WHEEL；后轮转向电机RUDDER
/// 
///     包含一个参数是单位编码器脉冲数对应的角度
///     对于WHEEL来说，车轮转一圈的脉冲数为编码器400线*4倍频*20倍减速机；
///     对于RUDDER来说，电机转一圈的脉冲数为16384；
pub struct Motor(pub f32);

impl Motor {
    pub const WHEEL: Motor = Motor(2.0 * PI / (4.0 * 400.0 * 20.0));
    pub const RUDDER: Motor = Motor(2.0 * PI / 16384.0);
    ///根据脉冲数换算成轮转角
    pub fn pluses_to_rad(&self, pulses: i32) -> f32 {
        (pulses as f32) * self.0
    }
    ///根据轮转角换算成脉冲数
    pub fn rad_to_pulses(&self, rad: f32) -> i32 {
        (rad as f32 / self.0).round() as i32
    }
}
