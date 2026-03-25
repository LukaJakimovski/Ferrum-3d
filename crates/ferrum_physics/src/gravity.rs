use ferrum_core::integrators::dormand_prince::ode45_step;
use ferrum_core::math::{Float, Vec3};
use crate::{GravityMode, Physics};
use crate::rigidbody_set::RigidBodySet;

impl Physics{
    pub fn integrate_linear(&self, dt: Float) -> Vec<(Vec3, Vec3)>{
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
                    GravityMode::Off => {Vec3::ZERO},
                    GravityMode::Uniform => {uniform_g},
                    GravityMode::Newtonian => {Self::newtonian_gravity(dt_offset, my_pos, _my_vel, g_const, &snap_positions, body_id, &snap_velocities, &snap_mass, r)},
                };
                accel

            };
            next_values.push(ode45_step(0.0, self.rigidbodies.get_position(body_id), self.rigidbodies.get_velocity(body_id), dt, self.rigidbodies.get_inv_mass(body_id), &force));
        }
        next_values
    }

    #[inline]
    pub fn newtonian_gravity(dt_offset: Float, my_pos: Vec3, _my_vel: Vec3, g_const: Float, snap_positions: &Vec<Vec3>, body_id: usize, snap_velocities: &Vec<Vec3>, snap_mass: &Vec<Float>, r: &RigidBodySet) -> Vec3{
        let mut accel = Vec3::ZERO;
        for j in 0..snap_positions.len() {
            if body_id == j { continue; }

            let other_pos_at_t = snap_positions[j] + snap_velocities[j] * dt_offset;

            let diff = other_pos_at_t - my_pos;
            let dist_sq = diff.length_squared() + 1e-14;
            accel += g_const * diff * (snap_mass[j] * r.mass[body_id] * r.mass[body_id]) / (dist_sq * dist_sq.sqrt());
        }
        accel
    }
}
