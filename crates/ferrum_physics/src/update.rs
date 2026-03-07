use ferrum_core::math::{Float, Vec3};
use crate::rigidbody::RigidBodySet;
pub struct Physics {
    pub rigidbodies: RigidBodySet
}

impl Physics{
    pub fn physics_update(&mut self, delta_time: Float) {
        // Mutably access an instance and change its position
        self.rigidbodies.translate(Vec3::new(1.0 * delta_time, 0.0, 0.0), 0) ;

        // Rebuild the raw instance data and write to the buffer
    }
}
