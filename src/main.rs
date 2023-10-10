mod physics;
mod aero;
mod output;

use std::{f64::consts::{FRAC_PI_4, PI, FRAC_PI_2}, process::{self, Command}, fs::{OpenOptions, self}};
use nalgebra::{vector, UnitQuaternion};

use aero::AeroBody;
use physics::PhysBody;
use output::write;

const DT: f64 = 0.05;
fn main() {
    fs::remove_file("state.csv").expect("could not remove file");
    let file = OpenOptions::new()
    .write(true)
    .create(true)
    .append(true)
    .open("state.csv")
    .unwrap();

    let mut wtr = csv::Writer::from_writer(file);
    
    wtr.write_record(&["xp", "yp", "zp", "xv", "yv", "zv", "xd", "yd", "zd", "spd", "t"]).expect("Could not write record");

    let mut missile = AeroBody::new(
    PhysBody::new(
        5.18,
        0.36,
        581.0,
        vector![0.0, 0.0, 2000.0],
        vector![100.0, 0.0, 0.0],
        UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
        vector![0.0, 0.0, 0.0],
        false,
    ),
    1.17,
    0.2,
    10.0, 
    2.0,
    0.5,
    0.2,
    2.5,
    );

    //program main loop
    for t in 0..((20.0 / DT) + 1.0) as i32 {
        println!("t = {}", t);
        //println!("{}", missile.att.transform_vector(&Vector3::x_axis()));
        //println!("{}", missile.phys.pos);

        //write current missile state to csv
        if let Err(err) = write(&mut wtr, &missile, t, DT) {
            println!("{}", err);
            process::exit(1);
        }

        //missile physics calculations
        missile.aerostate();
        missile.bodydrag();
        //missile.csurf(0.0, 0.0);
        missile.phys.step(DT);
    }
    Command::new("/home/jkurn/plot/.venv/bin/python")
        .arg("/home/jkurn/plot/plotblit.py").spawn().expect("Could not run plotblit.py");
}
