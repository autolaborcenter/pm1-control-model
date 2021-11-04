use crate::{Physical, Velocity, Wheels};

/// 描述底盘结构并用于转换控制量空间的结构体。
///
/// 上位机控制机器人运动采用 [`Physical`] 模型：
///
/// 利用 [`Physical`] 作为控制量，转换得到左右轮速 `lv`、`rv` 及自身的 `rudder` 三个控制量，发送给底盘
/// 先用 `physical_to_velocity(&self, physical: Physical) -> Velocity`，得到中间状态
/// 再利用 `velocity_to_wheels(&self, velocity: Velocity) -> Wheels`，得到 `lv`，`rv`
///
/// 里程计由 [`Wheels`] 模型转换得到 [`Velocity`] 模型
///
/// 利用 `wheels_to_velocity(&self, wheels: Wheels) -> Velocity`，利用 [`Velocity`] 进行预测
///
/// 增设临界角
///
/// 除了描述结构必要的三个参数之外，此结构体还额外缓存了临界角。
/// 临界角的目的是控制整车最大速度，避免出现原地转时的速度远大于前进的速度。
/// 当后轮转角（的绝对值）大于临界角，后轮相对于地面的线速度将大于前轮，
/// 此时，[`Physical`] 中 `speed` 表示后轮线速度；
/// 否则，`speed` 表示较快的前轮的线速度。
#[derive(Clone)]
pub struct ChassisModel {
    /// 左右车轮宽度 m
    pub width: f32,
    /// 前后轮距离 m
    pub length: f32,
    /// 车轮半径 m
    pub wheel: f32,
    /// 后轮临界角 rad
    critical_rudder: f32,
}

impl Default for ChassisModel {
    /// 默认底盘数据
    fn default() -> Self {
        Self::new(0.465, 0.355, 0.105)
    }
}

impl ChassisModel {
    /// 新建底盘模型
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
    /// 后轮物理模型换算到线、角速度模型
    pub fn physical_to_velocity(&self, physical: Physical) -> Velocity {
        if physical.rudder.is_nan() {
            Velocity { v: 0.0, w: 0.0 }
        } else if physical.rudder == 0.0 {
            Velocity {
                v: physical.speed,
                w: 0.0,
            }
        } else {
            // 底盘转弯半径
            let r_chassis = -self.length / physical.rudder.tan();
            let w = if physical.rudder.abs() > self.critical_rudder {
                // speed 为后轮线速度

                // 后轮转弯半径
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

    /// 线、角速度模型换算到后轮物理模型
    pub fn velocity_to_physical(&self, velocity: Velocity) -> Physical {
        if velocity.w == 0.0 {
            if velocity.v == 0.0 {
                Physical::RELEASED
            } else {
                Physical {
                    speed: velocity.v,
                    rudder: 0.0,
                }
            }
        } else {
            // 底盘转弯半径
            let r_chassis = velocity.v / velocity.w;
            // **NOTICE**
            // 此处需要关注 atan2 定义
            // 为了使结果在指定的象限内，需要设计两个量的符号
            // 此处需要 `r_chassis` 为正
            let rudder = (r_chassis.signum() * -self.length).atan2(r_chassis.abs());
            let speed = velocity.w
                * if rudder.abs() > self.critical_rudder {
                    // 后轮相对地面转弯半径
                    -self.length / rudder.sin()
                } else if rudder > 0.0 {
                    // 右轮相对地面转弯半径
                    r_chassis - self.width / 2.0
                } else {
                    // 左轮相对地面转弯半径
                    r_chassis + self.width / 2.0
                };
            Physical { speed, rudder }
        }
    }

    /// 左右轮速模型换算到线、角速度模型
    pub fn wheels_to_velocity(&self, wheels: Wheels) -> Velocity {
        Velocity {
            v: (wheels.right + wheels.left) * self.wheel / 2.0,
            w: (wheels.right - wheels.left) * self.wheel / self.width,
        }
    }

    /// 线、角速度模型换算到左右轮速模型
    pub fn velocity_to_wheels(&self, velocity: Velocity) -> Wheels {
        Wheels {
            left: (velocity.v - self.width / 2.0 * velocity.w) / self.wheel,
            right: (velocity.v + self.width / 2.0 * velocity.w) / self.wheel,
        }
    }

    /// 后轮物理模型换算到左右轮速模型
    pub fn physical_to_wheels(&self, physical: Physical) -> Wheels {
        self.velocity_to_wheels(self.physical_to_velocity(physical))
    }

    /// 左右轮速换算到后轮物理模型
    pub fn wheels_to_physical(&self, wheels: Wheels) -> Physical {
        self.velocity_to_physical(self.wheels_to_velocity(wheels))
    }
}

#[cfg(feature = "odometry")]
mod o {
    use crate::{odometry::Odometry, Physical, Velocity, Wheels};

    impl super::ChassisModel {
        pub fn velocity_to_odometry(&self, velocity: Velocity) -> Odometry {
            velocity.into()
        }

        pub fn wheels_to_odometry(&self, wheels: Wheels) -> Odometry {
            self.wheels_to_velocity(wheels).into()
        }

        pub fn physical_to_odometry(&self, physical: Physical) -> Odometry {
            self.physical_to_velocity(physical).into()
        }
    }
}

#[cfg(test)]
mod unit {
    use std::f32::consts::FRAC_PI_4;

    use super::*;

    #[inline]
    fn float_equal(a: f32, b: f32) -> bool {
        (a.is_nan() && b.is_nan()) || (a - b).abs() <= f32::EPSILON
    }

    #[test]
    fn test_physic_to_chassis() {
        let model: ChassisModel = ChassisModel::new(0.4, 0.3, 0.1);
        let physicals = [
            Physical::RELEASED,
            // Physical::ZERO, 只要机器人不移动，统一恢复为 Physical::RELEASED
            Physical {
                speed: 0.5,
                rudder: FRAC_PI_4,
            },
            Physical {
                speed: 0.5,
                rudder: -FRAC_PI_4,
            },
            Physical {
                speed: -0.5,
                rudder: FRAC_PI_4,
            },
            Physical {
                speed: -0.5,
                rudder: -FRAC_PI_4,
            },
        ];

        for physical in physicals {
            {
                // 验证 Physical 与 Velocity 互转
                let velocity = model.physical_to_velocity(physical);
                let restore = model.velocity_to_physical(velocity);
                assert!(
                    float_equal(physical.speed, restore.speed),
                    "{:?} != {:?}",
                    physical.speed,
                    restore.speed
                );
                assert!(
                    float_equal(physical.rudder, restore.rudder),
                    "{:?} != {:?}",
                    physical.rudder,
                    restore.rudder
                );
            }
            {
                // 验证 Physical 与 Wheels 互转
                let wheels = model.physical_to_wheels(physical);
                let restore = model.wheels_to_physical(wheels);
                assert!(
                    float_equal(physical.speed, restore.speed),
                    "{:?} != {:?}",
                    physical.speed,
                    restore.speed
                );
                assert!(
                    float_equal(physical.rudder, restore.rudder),
                    "{:?} != {:?}",
                    physical.rudder,
                    restore.rudder
                );
            }
        }
    }
}
