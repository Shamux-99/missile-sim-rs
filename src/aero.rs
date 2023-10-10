use nalgebra::{vector, Vector2, Vector3, Rotation2, Rotation3};

use crate::physics::PhysBody;

pub struct AeroBody {
    pub phys: PhysBody,
    cxa: f64,
    pfa: f64,
    csa: f64,
    csd: f64,
    k_n: f64,
    k_a: f64,
    k_m: f64,
    k_p: f64,
    l_f: f64,
    rho: f64,
    v_r: f64,
    dbs: f64,
}

impl AeroBody {
    pub fn new(
        phys: PhysBody,
        k_n: f64,
        k_a: f64,
        k_m: f64,
        k_p: f64,
        l_f: f64,
        csa: f64,
        csd: f64,
    ) -> Self {
        AeroBody {
            phys: phys,
            cxa: f64::powi(phys.radius * 2.0, 2),
            pfa: phys.radius * 2.0 * phys.length,
            k_n: k_n,
            k_a:k_a,
            k_m: k_m,
            k_p: k_p,
            l_f: l_f,
            csa: csa,
            csd: csd,
            rho: 1.225 * f64::powi(1.0 - (0.00002255691 * phys.pos[02]),2),
            v_r: phys.vel.norm(),
            dbs: 1.225 * f64::powi(1.0 - (0.00002255691 * phys.pos[02]),2) * f64::powi(phys.vel.norm(), 2),
        }
    }
    pub fn aerostate(
        &mut self,
    ) {
        //air density at altitude
        self.rho = 1.225 * f64::powi(1.0 - (0.00002255691 * self.phys.pos[02]),2);

        //missile physics velocity vector magnitude (speed)
        self.v_r = self.phys.vel.norm();

        //drag base for current speed and altitude
        self.dbs = self.rho * f64::powi(self.v_r, 2);
    }

    pub fn bodydrag(
        &mut self,
    ) {
        //unit velocity vector in missile coordinate system
        let relvel = self.phys.att.inverse_transform_vector(&self.phys.vel).normalize();
        let xcomp = -relvel.dot(&Vector3::x_axis());
        let ycomp = -relvel.dot(&Vector3::y_axis());
        let zcomp = -relvel.dot(&Vector3::z_axis());

        //local force summation of linear drag forces acting on missile
        let lfs = Vector3::new(
            xcomp * self.dbs * self.cxa * self.k_a,
            ycomp * self.dbs * self.pfa * self.k_n,
            zcomp * self.dbs * self.pfa * self.k_n,
        );
        println!("{}", lfs);

        let dmp = Vector3::new(
            0.0,
            self.rho * f64::powi(self.cxa, 2) * self.v_r * self.phys.att.inverse_transform_vector(&self.phys.rat)[01] * self.k_m,
            self.rho * f64::powi(self.cxa, 2) * self.v_r * self.phys.att.inverse_transform_vector(&self.phys.rat)[02] * self.k_m,
        );

        if f64::is_nan(lfs[00]) {
            self.phys.fsum += vector![0.0, 0.0, 0.0];
            self.phys.msum += vector![0.0, 0.0, 0.0];
        } else {
            self.phys.fsum += lfs;
            self.phys.msum += vector![0.0, lfs[02] * self.l_f, lfs[01] * -self.l_f] - dmp;
        }
    }

    pub fn csurf(
        &mut self,
        pitch: f64,
        yaw: f64,
    ) {
        //unit velocity vector in missile coordinate system
        let relvel = self.phys.att.inverse_transform_vector(&self.phys.vel).normalize();

        //unit velocity vector in pitch control surface coordinate system
        let rvp = Rotation3::new(vector![0.0, -pitch, 0.0]).transform_vector(&relvel);
        //println!("{}", rvp);

        //unit velocity vector in yaw control surface coordinate system
        let rvy = Rotation3::new(vector![0.0, 0.0, yaw]).transform_vector(&relvel);
        println!("{}", rvy);

        //drag on pitch control surface in missile coordinate system
        let pd: Vector2<f64> = Rotation2::new(pitch).inverse_transform_vector(&vector![
            self.dbs * self.csa * 2.0 * 0.1 * rvp.x,
            self.dbs * self.csa * 2.0 * 1.3 * rvp.y,
        ]);
        //println!("{}", pd);

        //drag on yaw control surface in missile coordinate system
        let yd: Vector2<f64> = Rotation2::new(-yaw).inverse_transform_vector(&vector![
            self.dbs * self.csa * 2.0 * 0.1 * rvy.x,
            self.dbs * self.csa * 2.0 * 1.3 * rvy.y,
        ]);
        //println!("{}", yd);

        let lfs = vector![-(pd.x + yd.x), yd.y, pd.y];
        //println!("{}", lfs);

        if f64::is_nan(lfs[00]) {
            self.phys.fsum += vector![0.0, 0.0, 0.0];
            self.phys.msum += vector![0.0, 0.0, 0.0];
        } else {
            self.phys.fsum += lfs;
            //self.phys.msum += vector![0.0, lfs[02] * self.csd, lfs[01] * self.csd];
        }
    }
}
