mod physics;
mod aero;
use pid::Pid;

use physics::PhysBody;
use aero::AeroBody;

pub fn from_desired_acc(
    body: PhysBody,
    acc: Vector3<f64>,
) {
    let current_acc = body.fsum * body.mass;
}