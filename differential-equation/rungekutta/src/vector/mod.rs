use std::ops::{Add, Sub};
use std::ops::{Div, Mul};

#[derive(Clone, Copy, Debug)]
pub struct Vector3<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T> Vector3<T> {
    pub fn new(x: T, y: T, z: T) -> Self {
        Self { x, y, z }
    }
}

impl Vector3<f64> {
    pub fn zeros() -> Self {
        Self::new(0.0f64, 0.0f64, 0.0f64)
    }
}

impl Add<Vector3<f64>> for Vector3<f64> {
    type Output = Vector3<f64>;

    fn add(self, rhs: Vector3<f64>) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Add<&Vector3<f64>> for Vector3<f64> {
    type Output = Vector3<f64>;

    fn add(self, rhs: &Vector3<f64>) -> Self::Output {
        Self::Output::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Add<Vector3<f64>> for &Vector3<f64> {
    type Output = Vector3<f64>;

    fn add(self, rhs: Vector3<f64>) -> Self::Output {
        Self::Output::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl<'a, 'b> Add<&'b Vector3<f64>> for &'a Vector3<f64> {
    type Output = Vector3<f64>;

    fn add(self, rhs: &'b Vector3<f64>) -> Self::Output {
        Self::Output::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Sub<Vector3<f64>> for Vector3<f64> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Mul<Vector3<f64>> for Vector3<f64> {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self {
        Self::new(self.x * rhs.x, self.y * rhs.y, self.z * rhs.z)
    }
}

impl Mul<f64> for Vector3<f64> {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Vector3::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl Mul<f64> for &Vector3<f64> {
    type Output = Vector3<f64>;

    fn mul(self, rhs: f64) -> Self::Output {
        Vector3::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl Div<f64> for Vector3<f64> {
    type Output = Vector3<f64>;

    fn div(self, rhs: f64) -> Self::Output {
        Vector3::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

impl Div<f64> for &Vector3<f64> {
    type Output = Vector3<f64>;

    fn div(self, rhs: f64) -> Self::Output {
        Vector3::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

impl Mul<Vector3<f64>> for f64 {
    type Output = Vector3<f64>;

    fn mul(self, rhs: Vector3<f64>) -> Self::Output {
        Vector3::new(self * rhs.x, self * rhs.y, self * rhs.z)
    }
}

impl Mul<&Vector3<f64>> for f64 {
    type Output = Vector3<f64>;

    fn mul(self, rhs: &Vector3<f64>) -> Self::Output {
        Vector3::new(self * rhs.x, self * rhs.y, self * rhs.z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vector3_new() {
        let v = Vector3::new(0.0f64, 1.0, 2.0);
        assert_eq!(v.x, 0.0);
        assert_eq!(v.y, 1.0);
        assert_eq!(v.z, 2.0);
    }

    #[test]
    fn vector3_new_zeros() {
        let v = Vector3::zeros();
        assert_eq!(v.x, 0.0);
        assert_eq!(v.y, 0.0);
        assert_eq!(v.z, 0.0);
    }
}
