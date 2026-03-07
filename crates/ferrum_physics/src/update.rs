use ferrum_core::math::{Float, Quat, Vec3};
use crate::rigidbody::RigidBodySet;
pub struct Physics {
    pub rigidbodies: RigidBodySet
}

impl Physics{
    pub fn physics_update(&mut self, dt: Float) {
        // Mutably access an instance and change its position
        self.rigidbodies.translate(Vec3::new(1.0 * dt, 0.0, 0.0), 0) ;
        self.rigidbodies.rotate(Quat::from_axis_angle(Vec3::new(1.0, 1.0, 1.0).normalize(), 3.0 * dt), 0);
        //println!("{:?}", self.rigidbodies.get_orientation(0));
        // Rebuild the raw instance data and write to the buffer
    }
}
