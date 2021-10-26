use std::fmt::Display;

use crate::Velocity;
use nalgebra::{ArrayStorage, Complex, Isometry2, SVector, Translation, Unit, Vector2};

/// 里程计模型
///
/// s：用于显示的总里程
///
/// a：用于显示的总转角
///
/// pose：  包含位置和角度
///
///         用一个SE(2)来表示，即一个2维变换和一个2维旋转组(theta)成
///         Isometry2<f32> = Isometry<f32, UnitComplex<f32>, 2>
///                                    R            T
///                                   旋转         变换
///                              theta ： a + bi   [x,y]
///
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Odometry {
    pub s: f32,
    pub a: f32,
    pub pose: Isometry2<f32>,
}

///初始化里程计
impl Odometry {
    pub const ZERO: Self = Self {
        s: 0.0,
        a: 0.0,
        pose: Isometry2 {
            translation: Translation {
                vector: SVector::from_array_storage(ArrayStorage([[0.0, 0.0]])),
            },
            rotation: Unit::new_unchecked(Complex { re: 1.0, im: 0.0 }),
        },
    };
}

///定义里程计从Velocity转成Odometry
impl From<Velocity> for Odometry {
    fn from(vel: Velocity) -> Self {
        let Velocity { v: s, w: theta } = vel;
        let a = theta.abs();

        Self {
            s: s.abs(),
            a,
            pose: Isometry2::new(
                if a < f32::EPSILON {
                    Vector2::new(s, 0.0)
                } else {
                    Vector2::new(theta.sin(), 1.0 - theta.cos()) * (s / theta)
                },
                theta,
            ),
        }
    }
}

///定义里程计（+=）运算，位姿的叠加在SE（2）中用乘法表示
impl std::ops::AddAssign for Odometry {
    fn add_assign(&mut self, rhs: Self) {
        self.s += rhs.s;
        self.a += rhs.a;
        self.pose *= rhs.pose;
    }
}

///定义里程计加法                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
impl std::ops::Add for Odometry {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        let mut copy = self.clone();
        copy += rhs;
        copy
    }
}

///显示里程计信息
impl Display for Odometry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Odometry: {{ s: {}, a: {}, x: {}, y: {}, theta: {} }}",
            self.s,
            self.a,
            self.pose.translation.vector[0],
            self.pose.translation.vector[1],
            self.pose.rotation.angle()
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn odometry_test() {
        let od = Odometry::ZERO;
        assert_eq!(
            format!("{}", od),
            "Odometry: { s: 0, a: 0, x: 0, y: 0, theta: 0 }"
        );

        let v1 = Velocity { v: 0.4, w: 0.3 };
        let v2 = Velocity { v: 0.6, w: 0.1 };
        let mut od1 = Odometry::from(v1);
        let od2 = Odometry::from(v2);

        let od11 = od1 + od2;
        od1 += od2;

        assert_eq!(format!("{}", od11), format!("{}", od1));
    }
}
