use crate::{Physical, Velocity, Wheels};

/// 描述底盘结构并用于转换控制量空间的结构体。
///
/// 控制量空间一共有三种：
///     三轮阿卡曼（Physical）模型：最快轮速Speed，后轮转角Rudder
///     质点控制（Velocity）模型：线速度V、角速度W
///     两轮差动控制（Wheels）模型：左轮轮速LV,右轮轮速RV
///
/// 上位机控制机器人运动采用Physical模型：
///     利用Physical作为控制量，转换得到左右轮速LV、RV及自身的Rudder三个控制量，发送给底盘
///     先用physical_to_velocity(&self, physical: Physical) -> Velocity，得到中间状态
///     再利用velocity_to_wheels(&self, velocity: Velocity) -> Wheels，得到LV，RV
/// 里程计由Wheels模型转换得到Velocity模型    
///     利用wheels_to_velocity(&self, wheels: Wheels) -> Velocity，利用Velocity进行预测
///
/// 增设临界角
///     除了描述结构必要的三个参数之外，此结构体还额外缓存了临界角。
///     临界角的目的是控制整车最大速度，避免出现原地转时的速度远大于前进的速度。
///     当后轮转角（的绝对值）大于临界角，后轮相对于地面的线速度将大于前轮，
///     此时，[`Physical`] 中 `speed` 表示后轮线速度；
///     否则，`speed` 表示较快的前轮的线速度。
#[derive(Clone)]
pub struct ChassisModel {
    ///左右车轮宽度，单位m
    pub width: f32,
    ///前后轮距离，单位m
    pub length: f32,
    ///车轮半径，单位m
    pub wheel: f32,
    ///后轮临界角，单位rad
    critical_rudder: f32,
}
/// 加载默认底盘数据（PM1底盘）
impl Default for ChassisModel {
    fn default() -> Self {
        Self::new(0.465, 0.355, 0.105)
    }
}

/// 新建底盘模型
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
    ///后轮物理模型换算到线、角速度模型
    pub fn physical_to_velocity(&self, physical: Physical) -> Velocity {
        if physical.rudder == 0.0 {
            Velocity {
                v: physical.speed,
                w: 0.0,
            }
        } else {
            let r_chassis = -self.length / physical.rudder.tan(); //转弯半径，由轴距length及后轮转角rudder确定
            let w = if physical.rudder.abs() > self.critical_rudder {
                // speed 为后轮线速度                                        //如果后轮轮速最大，角速度由后轮确定
                let r_rudder = -self.length / physical.rudder.sin(); //后轮转弯半径r_rudder, 由轮距length及后轮转角rudder确定
                physical.speed / r_rudder //角速度w，由线速度speed及后轮转弯半径r_rudder确定
            } else if physical.rudder > 0.0 {
                // speed 为左轮线速度
                physical.speed / (r_chassis - self.width / 2.0) //如果左轮轮速最大，角速度由左轮确定
            } else {
                // speed 为右轮线速度
                physical.speed / (r_chassis + self.width / 2.0) //如果右轮轮速最大，角速度由右轮确定
            };
            Velocity {
                v: w * r_chassis,
                w,
            }
        }
    }

    ///线、角速度模型换算到后轮物理模型
    pub fn velocity_to_physical(&self, velocity: Velocity) -> Physical {
        if velocity.w == 0.0 {
            Physical {
                speed: velocity.v,
                rudder: 0.0,
            }
        } else {
            let r_chassis = velocity.v / velocity.w; //阿卡曼模型，转动中心到车辆中心的距离 r_chassis = v/w;
            let rudder = self.length.atan2(r_chassis); //阿卡曼模型，轮转角 rudder = atan（轮距 length / 转动半径 r_chassis）
            let speed = velocity.w                      //最大速度由当前后轮轮转角及w共同决定
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

    ///左右轮速模型换算到线、角速度模型
    pub fn wheels_to_velocity(&self, wheels: Wheels) -> Velocity {
        Velocity {
            v: (wheels.right + wheels.left) * self.wheel / 2.0,
            w: (wheels.right - wheels.left) * self.wheel / self.width,
        }
    }

    ///线、角速度模型换算到左右轮速模型
    pub fn velocity_to_wheels(&self, velocity: Velocity) -> Wheels {
        Wheels {
            left: (velocity.v - self.width / 2.0 * velocity.w) / self.wheel,
            right: (velocity.v + self.width / 2.0 * velocity.w) / self.wheel,
        }
    }

    //后轮物理模型换算到左右轮速模型
    pub fn physical_to_wheels(&self, physical: Physical) -> Wheels {
        self.velocity_to_wheels(self.physical_to_velocity(physical))
    }

    ///左右轮速换算到后轮物理模型
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
mod tests {
    use nalgebra::ComplexField;

    use super::*;

    #[test]
    fn test_physic_to_chassis() {
        let chassis_model = ChassisModel::new(0.4, 0.3, 0.1);
        let phyical = Physical {
            speed: 0.4,
            rudder: std::f32::consts::FRAC_PI_2 - 0.1,
        };
        let velocity = chassis_model.physical_to_velocity(phyical);
        let wheels = chassis_model.velocity_to_wheels(velocity);
        let physical1 = chassis_model.wheels_to_physical(wheels);
        let physical2 = chassis_model.velocity_to_physical(velocity);
        assert_eq!(phyical, physical2);

        assert!((std::f32::consts::PI - physical2.rudder - phyical.rudder).abs() < 1e-3);
        assert!((std::f32::consts::PI - physical1.rudder - phyical.rudder).abs() < 1e-3);
    }
}
