pub use crate::Physics;

impl Physics{
    pub fn physics_update(&mut self, dt: f64) {
        self.integrate_linear(dt);
        self.integrate_angular(dt);
        self.resolve_collisions();
        self.energy.update_energy(&self.rigidbodies);
    }
}
