pub use crate::Physics;

impl Physics{
    pub fn physics_update(&mut self, dt: f64) {
        self.integrate_linear(dt);
        self.integrate_angular(dt);
        // Resolve after integrating so penetration created by this step is
        // pushed out before the frame is drawn.
        self.resolve_collisions(dt);
        self.energy.update_energy(&self.rigidbodies, &self.parameters);
    }
}
