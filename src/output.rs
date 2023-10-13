use crate::physics::PhysBody;

use std::error::Error;
use std::fs::File;
use nalgebra::Vector3;
use csv::Writer;

pub fn write(wtr: &mut Writer<File>, object: &PhysBody, frame: i32, dt: f64) -> Result<(), Box<dyn Error>> {
    let pos = object.pos;
    let vel = object.vel;
    let att = object.att.transform_vector(&Vector3::x_axis());

    wtr.write_record(&[
        pos[00].to_string(),
        pos[01].to_string(),
        pos[02].to_string(),

        vel[00].to_string(),
        vel[01].to_string(),
        vel[02].to_string(),
        
        att[00].to_string(),
        att[01].to_string(),
        att[02].to_string(),

        vel.norm().to_string(),

        (frame as f64 * dt).to_string(),
    ])?;
    wtr.flush()?;
    Ok(())
}