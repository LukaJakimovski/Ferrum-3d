use ferrum_core::math::{Float, Vec3};
use ferrum_core::integrators::dormand_prince::ode45_step;
use ferrum_core::integrators::rk4::integrate_rk4;
use crate::energy::Energy;
use crate::physics_vertex::{Polyhedron};
use crate::rigidbody::RigidBodySet;

pub struct Physics {
    pub rigidbodies: RigidBodySet,
    pub polyhedrons: Vec<Polyhedron>,
    pub energy: Energy
}

impl Physics{
    pub fn physics_update(&mut self, dt: &mut f64) {
        *dt = 0.001 * *dt;
        let dt = *dt as Float;
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
                    if body_id == j {continue; }

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
        for body_id in 0..self.rigidbodies.len() {
            integrate_rk4(&mut self.rigidbodies.orientations[body_id], &mut self.rigidbodies.omega[body_id], self.rigidbodies.inertia[body_id], self.rigidbodies.inv_inertia[body_id], self.rigidbodies.torques[body_id], dt);
            self.rigidbodies.positions[body_id] = next_values[body_id].0;
            self.rigidbodies.velocities[body_id] = next_values[body_id].1;
        }
        self.energy.update_energy(&self.rigidbodies);
    }
}
