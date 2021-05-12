use crate::vector;

type Vector3 = vector::Vector3<f64>;

pub trait RungeKutta4thOrder2nd {
    fn delta_time(&self) -> f64;

    fn position(&self) -> &Vector3;
    fn velocity(&self) -> &Vector3;

    fn add_position(&mut self, p: Vector3);
    fn add_velocity(&mut self, v: Vector3);

    fn to_velocity(&self, _t: f64, _position: &Vector3, _velocity: &Vector3) -> Vector3;
    fn to_accel(&self, t: f64, _position: &Vector3, _velocity: &Vector3) -> Vector3;

    fn solve(&mut self, t: f64) {
        let dt = self.delta_time();
        let p = self.position();
        let v = self.velocity();

        // let p0 = self.current_values();

        // 0th
        let k0v = self.to_velocity(t, &p, &v); // slope
        let k0a = self.to_accel(t, &p, &v); // slope

        // let k0 = self.to_slopes(t, &p0);

        // 1th
        let (p1, v1) = (next_value(&p, &k0v, dt / 2.), next_value(&v, &k0a, dt / 2.));
        let k1v = self.to_velocity(t, &p1, &v1); // slope
        let k1a = self.to_accel(t, &p1, &v1); // slope

        // let p1 = self.next_values(p0, k0, dt / 2.);
        // let k1 = self.to_slopes(t, &p1);

        // 2nd
        let (p2, v2) = (next_value(&p, &k1v, dt / 2.), next_value(&v, &k1a, dt / 2.)); // point
        let k2v = self.to_velocity(t, &p2, &v2); // slope
        let k2a = self.to_accel(t, &p2, &v2); // slope

        // let p2 = self.next_values(p1, k1, dt / 2.);
        // let k2 = self.to_slopes(t, &p2);

        // 3rd
        let (p3, v3) = (next_value(&p, &k2v, dt), next_value(&v, &k2a, dt)); // point
        let k3v = self.to_velocity(t, &p3, &v3); // slope
        let k3a = self.to_accel(t, &p3, &v3); // slope

        // let p3 = self.next_values(p2, k2, dt / 2.);
        // let k3 = self.to_slopes(t, &p3);

        // 4th
        self.add_position(average_slope(dt, &k0v, &k1v, &k2v, &k3v));
        self.add_velocity(average_slope(dt, &k0a, &k1a, &k2a, &k3a));
        // self.update_values(average_slope(dt, k0, k1, k2, k3));
    }
}

fn next_value(x0: &Vector3, k: &Vector3, dt: f64) -> Vector3 {
    x0 + k * dt
}

fn average_slope(dt: f64, k1: &Vector3, k2: &Vector3, k3: &Vector3, k4: &Vector3) -> Vector3 {
    dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.
}

#[cfg(test)]
mod tests {
    use super::*;

    //struct Mock {
    //    position: Vector3,
    //    velocity: Vector3,
    //    gravity: f64,
    //}

    //impl Default for Mock {
    //    fn default() -> Self {
    //        Self {
    //            delta_time: 0.1,
    //            position: Vector3::zeros(),
    //            velocity: Vector3::zeros(),
    //            gravity: -9.8,
    //        }
    //    }
    //}

    //    impl RungeKutta4th for Mock {
    //        fn to_velocity(
    //            &self,
    //            _t: f64,
    //            _position: &Vector3,
    //            verocity: &Vector3,
    //        ) -> Vector3 {
    //            Vector3::new(verocity.x, verocity.y, verocity.z)
    //        }
    //
    //        fn to_accel(&self, _t: f64, _: &Vector3, _: &Vector3) -> Vector3 {
    //            Vector3::new(0.0, 0.0, self.gravity)
    //        }
    //    }

    #[test]
    fn next_value_test() {
        let v = next_value(&Vector3::new(1., 2., 3.), &Vector3::new(10., 20., 30.), 2.);
        assert_eq!(v.x, 21.);
        assert_eq!(v.y, 42.);
        assert_eq!(v.z, 63.);
    }

    #[test]
    fn average_slope_test() {
        let v = average_slope(
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
