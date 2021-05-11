use crate::vector;

type Vector3 = vector::Vector3<f64>;

pub trait RungeKutta4th {
    fn solve(&mut self, t: f64);
    fn to_velocity(&self, _t: f64, _position: &Vector3, _velocity: &Vector3) -> Vector3;
    fn to_accel(&self, t: f64, _position: &Vector3, _velocity: &Vector3) -> Vector3;

    // p1 = p0 + k1 * dt / 2
    fn step1st(dt: f64, x: &Vector3, dx: &Vector3) -> Vector3 {
        dx * (dt / 2.) + x
    }

    // p2 = p0 + k2 * dt / 2
    fn step2nd(dt: f64, x: &Vector3, dx: &Vector3) -> Vector3 {
        dx * (dt / 2.) + x
    }

    // p3 =  p0 + k3 * dt / 2
    fn step3rd(dt: f64, x: &Vector3, dx: &Vector3) -> Vector3 {
        dx * dt + x
    }

    fn step4th(dt: f64, x1: &Vector3, x2: &Vector3, x3: &Vector3, x4: &Vector3) -> Vector3 {
        dt * (x1 + 2.0 * x2 + 2.0 * x3 + x4) / 6.
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Mock {
        delta_time: f64,
        position: Vector3,
        velocity: Vector3,
        gravity: f64,
        _delta_position: Vector3,
        _delta_velocity: Vector3,
    }

    impl Default for Mock {
        fn default() -> Self {
            Self {
                delta_time: 0.1,
                position: Vector3::zeros(),
                velocity: Vector3::zeros(),
                gravity: -9.8,
                _delta_position: Vector3::zeros(),
                _delta_velocity: Vector3::zeros(),
            }
        }
    }

    impl RungeKutta4th for Mock {
        fn solve(&mut self, t: f64) {
            let dt = self.delta_time;

            // 0th
            let k1v = self.to_velocity(t, &self.position, &self.velocity);
            let k1a = self.to_accel(t, &self.position, &self.velocity);

            // 1th
            let v1 = Self::step1st(dt, &self.position, &k1v);
            let a1 = Self::step1st(dt, &self.velocity, &k1a);
            let k2v = self.to_velocity(t + dt / 2., &v1, &a1);
            let k2a = self.to_accel(t + dt / 2., &v1, &a1);

            // 2nd
            let v2 = Self::step2nd(dt, &self.position, &k2v);
            let a2 = Self::step2nd(dt, &self.velocity, &k2a);
            let k3v = self.to_velocity(t + dt / 2., &v2, &a2);
            let k3a = self.to_accel(t + dt / 2., &v2, &a2);

            // 3rd
            let v3 = Self::step3rd(dt, &self.position, &k3v);
            let a3 = Self::step3rd(dt, &self.velocity, &k3a);
            let k4v = self.to_velocity(t + dt, &v3, &a3);
            let k4a = self.to_accel(t + dt, &v3, &a3);

            // 4th
            self._delta_position = Self::step4th(dt, &k1v, &k2v, &k3v, &k4v);
            self._delta_velocity = Self::step4th(dt, &k1a, &k2a, &k3a, &k4a);
        }

        fn to_velocity(&self, _t: f64, _position: &Vector3, verocity: &Vector3) -> Vector3 {
            Vector3::new(verocity.x, verocity.y, verocity.z)
        }

        fn to_accel(&self, _t: f64, _: &Vector3, _: &Vector3) -> Vector3 {
            Vector3::new(0.0, 0.0, self.gravity)
        }
    }

    #[test]
    fn rungekutta4th_step1st() {
        let v = Mock::step1st(4., &Vector3::new(1., 2., 3.), &Vector3::new(10., 20., 30.));
        assert_eq!(v.x, 21.);
        assert_eq!(v.y, 42.);
        assert_eq!(v.z, 63.);
    }

    #[test]
    fn rungekutta4th_step2nd() {
        let v = Mock::step2nd(4., &Vector3::new(1., 2., 3.), &Vector3::new(10., 20., 30.));
        assert_eq!(v.x, 21.);
        assert_eq!(v.y, 42.);
        assert_eq!(v.z, 63.);
    }

    #[test]
    fn rungekutta4th_step3rd() {
        let v = Mock::step3rd(2., &Vector3::new(1., 2., 3.), &Vector3::new(10., 20., 30.));
        assert_eq!(v.x, 21.);
        assert_eq!(v.y, 42.);
        assert_eq!(v.z, 63.);
    }

    #[test]
    fn rungekutta4th_step4th() {
        let v = Mock::step4th(
            6.,
            &Vector3::new(1., 2., 3.),
            &Vector3::new(10., 20., 30.),
            &Vector3::new(100., 200., 300.),
            &Vector3::new(1000., 2000., 3000.),
        );
        assert_eq!(v.x, 1221.);
        assert_eq!(v.y, 2442.);
        assert_eq!(v.z, 3663.);
    }
}
