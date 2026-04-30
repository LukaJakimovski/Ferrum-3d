pub use crate::Physics;

impl Physics{
    pub fn physics_update(&mut self, dt: f64) {
        self.resolve_collisions(dt);
        self.integrate_linear(dt);
        self.integrate_angular(dt);
        self.energy.update_energy(&self.rigidbodies, &self.parameters);
    }
}
