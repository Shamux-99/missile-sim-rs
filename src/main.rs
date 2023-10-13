mod physics;
mod aero;
mod output;
mod guidance;

use std::{f64::consts::FRAC_PI_2, process::{self, Command}, fs::{OpenOptions, self}};
use nalgebra::{vector, UnitQuaternion};

use aero::AeroBody;
use physics::PhysBody;
use output::write;
use guidance::pronav;

const DT: f64 = 0.05;
fn main() {
    let mut aero_bodies: Vec<AeroBody> = Vec::new();

    let mut missile = AeroBody::new(
    PhysBody::new(
        5.18,
        0.36,
        581.0,
        vector![0.0, 0.0, 0.0],
        vector![0.0, 0.0, 0.0],
        UnitQuaternion::from_euler_angles(0.0, -FRAC_PI_2, 0.0),
        vector![0.0, 0.0, 0.0],
        true,
        ),
    1.17,
    0.2,
    2.0, 
    2.0,
    0.5,
    0.2,
    2.5,
    );
    aero_bodies.push(missile);

    let mut target = AeroBody::new(
    PhysBody::new(
        15.0,
        2.0,
        10930.0,
        vector![-4000.0, 0.0, 8000.0],
        vector![150.0, -150.0, -50.0],
        UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
        vector![0.0, 0.0, 0.0],
        false,
        ),
    1.17,
    0.2,
    2.0,
    2.0,
    0.5,
    0.2,
    2.5,
    );
    aero_bodies.push(target);

    fs::remove_file("outputs/missile_state.csv").expect("could not remove file");
    let missile_file = OpenOptions::new()
    .write(true)
    .create(true)
    .append(true)
    .open("outputs/missile_state.csv")
    .unwrap();

    let mut missile_wtr = csv::Writer::from_writer(missile_file);

    fs::remove_file("outputs/target_state.csv").expect("could not remove file");
    let target_file = OpenOptions::new()
    .write(true)
    .create(true)
    .append(true)
    .open("outputs/target_state.csv")
    .unwrap();

    let mut target_wtr = csv::Writer::from_writer(target_file);
    
    missile_wtr.write_record(&["xp", "yp", "zp", "xv", "yv", "zv", "xd", "yd", "zd", "spd", "t"]).expect("Could not write record");
    target_wtr.write_record(&["xp", "yp", "zp", "xv", "yv", "zv", "xd", "yd", "zd", "spd", "t"]).expect("Could not write record");

    //program main loop
    for t in 0..((40.0 / DT) + 1.0) as i32 {

        if missile.phys.pos[02] < 0.0 || (missile.phys.pos - target.phys.pos).norm() < 50.0 {
            break;
        }

        println!("t = {}", t);
        //println!("{}", missile.att.transform_vector(&Vector3::x_axis()));
        //println!("{}", missile.phys.pos);

        //write current missile state to csv
        if let Err(err) = write(&mut missile_wtr, &missile.phys, t, DT) {
            println!("{}", err);
            process::exit(1);
        }

        //write current target state to csv
        if let Err(err) = write(&mut target_wtr, &target.phys, t, DT) {
            println!("{}", err);
            process::exit(1);
        }

        //target physics calculations
        target.phys.step(DT);

        //missile physics calculations
        missile.phys.fsum += vector![24000.0, 0.0, 0.0];
        pronav(missile, target.phys, &mut missile.csp, &mut missile.csy);
        missile.aerostate();    
        missile.bodydrag();
        missile.csurf(missile.csp, missile.csy);
        missile.phys.step(DT);
    }
    Command::new("/home/jkurn/plot/.venv/bin/python")
        .arg("/home/jkurn/plot/plotblit.py").spawn().expect("Could not run plotblit.py");
}