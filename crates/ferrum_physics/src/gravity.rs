use ferrum_core::integrators::dormand_prince::ode45_step;
use ferrum_core::math::{Float, Vec3};
use crate::update::Physics;

impl Physics{
    pub fn update_gravity(&self, dt: Float) -> Vec<(Vec3, Vec3)>{
        // Mutably access an instance and change its position
        let snap_positions = self.rigidbodies.positions.clone();
        let snap_velocities = self.rigidbodies.velocities.clone();
        let snap_mass = self.rigidbodies.mass.clone();
        let mut next_values: Vec<(Vec3, Vec3)> = Vec::with_capacity(self.rigidbodies.len());
        let r = &self.rigidbodies;
        for body_id in 0..r.len() {
            //let force = | _: Float, _: Vec3, _: Vec3 | {Vec3::new(0.0, 9.8, 0.0)};
            let force = |dt_offset: Float, my_pos: Vec3, _my_vel: Vec3| {
                let mut accel = Vec3::new(0.0, 0.0, 0.0) / self.rigidbodies.inv_mass[body_id];
                for j in 0..snap_positions.len() {
                    if body_id == j { continue; }

                    let other_pos_at_t = snap_positions[j] + snap_velocities[j] * dt_offset;

                    let diff = other_pos_at_t - my_pos;
                    let dist_sq = diff.length_squared() + 1e-14;
                    const G: Float = 0.25;
                    accel += G * diff * (snap_mass[j] * r.mass[body_id] * r.mass[body_id]) / (dist_sq * dist_sq.sqrt());
                }
                accel
            };
            next_values.push(ode45_step(0.0, self.rigidbodies.get_position(body_id), self.rigidbodies.get_velocity(body_id), dt, self.rigidbodies.get_inv_mass(body_id), &force));
        }
        next_values
    }
}
