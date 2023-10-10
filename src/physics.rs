use nalgebra::{vector, Vector3, UnitQuaternion};

#[derive(Clone, Copy)]

pub struct PhysBody {
    //dimensions of body (m)
    pub length: f64,
    pub radius: f64,

    //mass of body (kg)
    pub mass: f64,

    //mass moment of inertia of body around given axis (kg.m^2)
    moi: f64,
    moj: f64,
    mok: f64,

    //position and velocity of object
    pub pos: Vector3<f64>,
    pub vel: Vector3<f64>,

    //attitude of object (transformation from inertial to missile coordinate system)
    pub att: UnitQuaternion<f64>,
    pub rat: Vector3<f64>,

    //sum of forces and moments acting upon object
    pub fsum: Vector3<f64>,
    pub msum: Vector3<f64>,

    //whether the object is affected by gravity
    pub grav: bool,
}

impl PhysBody {
    pub fn new(
        length: f64,
        radius: f64,
        mass: f64, 
        pos: Vector3<f64>,
        vel: Vector3<f64>,
        att: UnitQuaternion<f64>,
        rat: Vector3<f64>,
        grav: bool,
    ) -> Self {
        PhysBody {
            mass: mass,
            radius: radius,
            length: length,

            moi: f64::powi(mass * radius, 2) / 2.0,
            moj: (mass / 12.0) * (3.0 * f64::powi(radius, 2) + f64::powi(length, 2)),
            mok: (mass / 12.0) * (3.0 * f64::powi(radius, 2) + f64::powi(length, 2)),

            pos: pos,
            vel: vel,
            att: att,
            rat: rat,

            fsum: vector![0.0, 0.0, 0.0],
            msum: vector![0.0, 0.0, 0.0],

            grav: grav,
        }
    }

    pub fn step(
        &mut self,
        dt: f64,
    ) {
        //force to linear acceleration in absolute coordinate system
        let mut lacc = self.fsum / self.mass;
        lacc = self.att.transform_vector(&lacc);
        
        //moment to angular acceleration in absolute coordinate system
        let mut aacc = self.msum.component_div(&Vector3::new(self.moi, self.moj, self.mok));
        aacc = self.att.transform_vector(&aacc);

        //Euler integration of linear motion
        self.vel += lacc * dt;
        self.pos += self.vel * dt;

        //Euler integration of angular motion
        self.rat += aacc * dt;
        self.att *= UnitQuaternion::from_scaled_axis(self.rat * dt);

        //reset force and vector sums
        self.fsum = vector![0.0, 0.0, 0.0];
        self.msum = vector![0.0, 0.0, 0.0];
    }
}