use ferrum_core::math::{Float, Quat, Vec3};
use ferrum_core::dormand_prince::ode45_step;
use crate::rigidbody::RigidBodySet;
pub struct Physics {
    pub rigidbodies: RigidBodySet
}

impl Physics{
    pub fn physics_update(&mut self, dt: Float) {
        // Mutably access an instance and change its position
        for body_id in 0..self.rigidbodies.len() {
            let force = | _: Float, _: Vec3, _: Vec3 | {Vec3::new(0.0, 0.0, 0.0)};
            let (next_x, next_v) = ode45_step(0.0, self.rigidbodies.get_position(body_id), self.rigidbodies.get_velocity(body_id), dt, self.rigidbodies.get_inv_mass(body_id), force);
            self.rigidbodies.positions[body_id] = next_x;
            self.rigidbodies.velocities[body_id] = next_v;
            self.rigidbodies.rotate(Quat::from_axis_angle(Vec3::new(3.0, 1.0, 1.0).normalize(), 3.0 * dt), body_id) ;
        }
    }
}
