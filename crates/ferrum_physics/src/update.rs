use ferrum_core::integrators::rk4::integrate_rk4;
pub use crate::Physics;

impl Physics{
    pub fn physics_update(&mut self, dt: f64) {
        self.resolve_collisions();
        let next_values = self.integrate_linear(dt);
        for body_id in 0..self.rigidbodies.len() {
            integrate_rk4(&mut self.rigidbodies.orientations[body_id], &mut self.rigidbodies.omega[body_id], self.rigidbodies.inertia[body_id], self.rigidbodies.inv_inertia[body_id], self.rigidbodies.torques[body_id], dt);
            self.rigidbodies.positions[body_id] = next_values[body_id].0;
            self.rigidbodies.velocities[body_id] = next_values[body_id].1;
        }
        self.energy.update_energy(&self.rigidbodies);
    }
}
