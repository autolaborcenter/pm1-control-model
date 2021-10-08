use crate::{Physical, Velocity, Wheels};

/// 描述底盘结构并用于转换控制量空间的结构体。
///
/// 除了描述结构必要的三个参数之外，此结构体还额外缓存了临界角。
/// 当后轮转角（的绝对值）大于临界角，后轮相对于地面的线速度将大于前轮，
/// 此时，[`Physical`] 中 `speed` 表示后轮线速度；
/// 否则，`speed` 表示较快的前轮的线速度。
pub struct ChassisModel {
    pub width: f32,
    pub length: f32,
    pub wheel: f32,
    critical_rudder: f32,
}

impl Default for ChassisModel {
    fn default() -> Self {
        Self::new(0.465, 0.355, 0.105)
    }
}

impl ChassisModel {
    pub fn new(width: f32, length: f32, wheel: f32) -> Self {
        Self {
            width,
            length,
            wheel,
            critical_rudder: (length * length / width - width / 2.0).atan2(length),
        }
    }
}

impl ChassisModel {
    pub fn physical_to_velocity(&self, physical: Physical) -> Velocity {
        if physical.rudder == 0.0 {
            Velocity {
                v: physical.speed,
                w: 0.0,
            }
        } else {
            let r_chassis = -self.length / physical.rudder.tan();
            let w = if physical.rudder.abs() > self.critical_rudder {
                // speed 为后轮线速度
                let r_rudder = -self.length / physical.rudder.sin();
                physical.speed / r_rudder
            } else if physical.rudder > 0.0 {
                // speed 为左轮线速度
                physical.speed / (r_chassis - self.width / 2.0)
            } else {
                // speed 为右轮线速度
                physical.speed / (r_chassis + self.width / 2.0)
            };
            Velocity {
                v: w * r_chassis,
                w,
            }
        }
    }

    pub fn velocity_to_physical(&self, velocity: Velocity) -> Physical {
        if velocity.w == 0.0 {
            Physical {
                speed: velocity.v,
                rudder: 0.0,
            }
        } else {
            let r_chassis = velocity.v / velocity.w;
            let rudder = self.length.atan2(r_chassis);
            let speed = velocity.w
                * if rudder.abs() > self.critical_rudder {
                    -self.length / rudder.sin() // r_rudder
                } else if rudder > 0.0 {
                    r_chassis - self.width / 2.0
                } else {
                    r_chassis + self.width / 2.0
                };
            Physical { speed, rudder }
        }
    }

    pub fn wheels_to_velocity(&self, wheels: Wheels) -> Velocity {
        Velocity {
            v: (wheels.right + wheels.left) * self.wheel / 2.0,
            w: (wheels.right - wheels.left) * self.wheel / self.width,
        }
    }

    pub fn velocity_to_wheels(&self, velocity: Velocity) -> Wheels {
        Wheels {
            left: (velocity.v - self.width / 2.0 * velocity.w) / self.wheel,
            right: (velocity.v + self.width / 2.0 * velocity.w) / self.wheel,
        }
    }

    pub fn physical_to_wheels(&self, physical: Physical) -> Wheels {
        self.velocity_to_wheels(self.physical_to_velocity(physical))
    }

    pub fn wheels_to_physical(&self, wheels: Wheels) -> Physical {
        self.velocity_to_physical(self.wheels_to_velocity(wheels))
    }

    pub fn wheels_rad_to_delta(&self, d_rad: (f32, f32)) -> (f32, f32) {
        let Velocity { v: s, w: a } = self.wheels_to_velocity(Wheels {
            left: self.wheel * d_rad.0,
            right: self.wheel * d_rad.1,
        });
        (s, a)
    }
}
