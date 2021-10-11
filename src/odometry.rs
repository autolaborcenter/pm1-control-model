use crate::Velocity;
use nalgebra::{ArrayStorage, Complex, Isometry2, SVector, Translation, Unit, Vector2};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Odometry {
    pub s: f32,
    pub a: f32,
    pub pose: Isometry2<f32>,
}

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

impl From<Velocity> for Odometry {
    fn from(v: Velocity) -> Self {
        let Velocity { v: s, w: theta } = v;
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

impl std::ops::AddAssign for Odometry {
    fn add_assign(&mut self, rhs: Self) {
        self.s += rhs.s;
        self.a += rhs.a;
        self.pose *= rhs.pose;
    }
}

impl std::ops::Add for Odometry {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        let mut copy = self.clone();
        copy += rhs;
        copy
    }
}
