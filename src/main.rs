mod physics;
use std::f64::consts::{FRAC_PI_4, PI, FRAC_PI_2};

use nalgebra::{vector, Vector3, UnitQuaternion};

use crate::physics::PhysBody;

const DT: f64 = 0.01;
fn main() {
    let mut missile = PhysBody::new(
        5.18,
        0.36,
        581.0,
        vector![0.0, 0.0, 10.0],
        vector![0.0, 0.0, 0.0],
        UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
        vector![0.0, 0.0, 0.0],
        true,
    );

    for t in 0..((1.0 / DT) + 1.0) as i32 {
        println!("t = {}", t);
        //println!("{}", missile.att.transform_vector(&Vector3::x_axis()));
        println!("{}", missile.pos);
        missile.msum += vector![0.0, 0.0, 0.0];
        missile.step(DT);
    }
}
