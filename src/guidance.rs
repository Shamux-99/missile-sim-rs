use crate::aero::AeroBody;
use crate::physics::PhysBody;

pub fn pronav(
    missile: AeroBody,
    target: PhysBody,
    pitch: &mut f64,
    yaw: &mut f64,
) {
    let los = missile.phys.att.inverse_transform_vector(&(target.pos - missile.phys.pos).normalize());
    let dlos = missile.phys.att.inverse_transform_vector(&((target.pos + target.vel) - (missile.phys.pos + missile.phys.vel)).normalize()) - los;
    //let current_acc = missile.phys.fsum * missile.phys.mass;

    //let nav = 3.0 * dlos * missile.v_r;

    *pitch = 3.0 * dlos.z;
    *yaw = 3.0 * -dlos.y;
}
