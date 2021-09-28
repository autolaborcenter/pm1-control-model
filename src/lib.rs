use std::f32::consts::PI;

pub struct ChassisModel {
    pub width: f32,
    pub length: f32,
    pub wheel: f32,
    critical_rudder: f32, // 临界角
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

pub struct Physical {
    speed: f32,
    rudder: f32,
}

pub struct Wheels {
    left: f32,
    right: f32,
}

pub struct Velocity {
    v: f32,
    w: f32,
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
}

pub struct MotorModel(f32);

pub const WHEEL_MOTOR: MotorModel = MotorModel(2.0 * PI / (4.0 * 400.0 * 20.0));
pub const RUDDER_MOTOR: MotorModel = MotorModel(2.0 * PI / 16384.0);

impl MotorModel {
    pub fn pluses_to_rad(&self, pulses: i32) -> f32 {
        (pulses as f32) * self.0
    }

    pub fn rad_to_pulses(&self, rad: f32) -> i32 {
        (rad as f32 / self.0).round() as i32
    }
}
