use ferrum_core::math::{Float, Quat, Vec3};
use ferrum_core::dormand_prince::ode45_step;
use crate::rigidbody::RigidBodySet;
use crate::rigidbodybuilder::RigidBody;

pub struct Physics {
    pub rigidbodies: RigidBodySet
}

impl Physics{
    pub fn physics_update(&mut self, dt: Float) {
        // Mutably access an instance and change its position
        let snapshot = self.rigidbodies.clone();
        let mut next_bodies: RigidBodySet = self.rigidbodies.clone();
        let r = &self.rigidbodies;
        for body_id in 0..r.len() {
            //let force = | _: Float, _: Vec3, _: Vec3 | {Vec3::new(0.0, 9.8, 0.0)};
            let force = |dt_offset: Float, my_pos: Vec3, _my_vel: Vec3| {
                let mut accel = Vec3::new(0.0, 0.0, 0.0) / self.rigidbodies.inv_mass[body_id].clone();
                for j in 0..snapshot.len() {
                    if body_id == j {continue; }

                    let other_pos_at_t = snapshot.positions[j] + snapshot.velocities[j] * dt_offset;

                    let diff = other_pos_at_t - my_pos;
                    let dist_sq = diff.length_squared() + 1e-14;
                    accel += diff * (1.0 / snapshot.inv_mass[j] / (dist_sq * dist_sq.sqrt()));
                }
                accel
            };
            let (next_x, next_v) = ode45_step(0.0, self.rigidbodies.get_position(body_id), self.rigidbodies.get_velocity(body_id), dt, self.rigidbodies.get_inv_mass(body_id), &force);
            next_bodies.positions[body_id] = next_x;
            next_bodies.velocities[body_id] = next_v;
            next_bodies.rotate(Quat::from_axis_angle(next_bodies._omega[body_id].normalize(), next_bodies._omega[body_id].length() * dt), body_id);
        }
        self.rigidbodies = next_bodies;
    }
}
