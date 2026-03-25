use ferrum_core::integrators::dormand_prince::ode45_step;
use ferrum_core::integrators::rk4::integrate_rk4;
use ferrum_core::math::{Float, Vec3};
use crate::{GravityMode, Physics};

impl Physics {
    pub fn integrate_linear(&mut self, dt: Float) {
        // Mutably access an instance and change its position
        let snap_positions = self.rigidbodies.positions.clone();
        let snap_velocities = self.rigidbodies.velocities.clone();
        let snap_mass = self.rigidbodies.mass.clone();
        let g_mode = self.parameters.gravity_mode;
        let uniform_g = self.parameters.uniform_gravity;
        let g_const = self.parameters.gravity_constant;
        let mut next_values: Vec<(Vec3, Vec3)> = Vec::with_capacity(self.rigidbodies.len());
        let r = &self.rigidbodies;


        for body_id in 0..r.len() {
            let force = |dt_offset: Float, my_pos: Vec3, _my_vel: Vec3| {
                let mut accel = Vec3::new(0.0, 0.0, 0.0);
                accel += match g_mode {
                    GravityMode::Off => { Vec3::ZERO },
                    GravityMode::Uniform => { uniform_g },
                    GravityMode::Newtonian => { Self::newtonian_gravity(dt_offset, my_pos, _my_vel, g_const, &snap_positions, body_id, &snap_velocities, &snap_mass, r) },
                };
                accel
            };
            next_values.push(ode45_step(0.0, self.rigidbodies.get_position(body_id), self.rigidbodies.get_velocity(body_id), dt, self.rigidbodies.get_inv_mass(body_id), &force));
        }

        for body_id in 0..self.rigidbodies.len() {
            self.rigidbodies.positions[body_id] = next_values[body_id].0;
            self.rigidbodies.velocities[body_id] = next_values[body_id].1;
        }
    }

    pub fn integrate_angular(&mut self, dt: Float) {
        for body_id in 0..self.rigidbodies.len() {
            integrate_rk4(&mut self.rigidbodies.orientations[body_id], &mut self.rigidbodies.omega[body_id], self.rigidbodies.inertia[body_id], self.rigidbodies.inv_inertia[body_id], self.rigidbodies.torques[body_id], dt);
        }
    }
}