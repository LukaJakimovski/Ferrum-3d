use ferrum_core::math::Float;
use crate::rigidbody::RigidBodySet;

#[derive(Default)]
pub struct Energy {
    pub kinetic_energy: Float,
    pub rotational_kinetic_energy: Float,
    pub gravitational_potential_energy: Float,
    pub total_energy: Float,
    pub start_energy: Float,
}


impl Energy {
    pub fn update_energy(&mut self, rigidbodies: &RigidBodySet) {
        self.update_kinetic_energy(rigidbodies);
        self.update_rotational_kinetic_energy(rigidbodies);
        self.update_gravitational_potential_energy(rigidbodies);
        self.total_energy = self.kinetic_energy + self.gravitational_potential_energy + self.rotational_kinetic_energy;
    }

    fn update_kinetic_energy(&mut self, rigidbodies: &RigidBodySet) {
        self.kinetic_energy = 0.0;
        for i in 0..rigidbodies.len(){
            self.kinetic_energy = self.kinetic_energy + 0.5 * rigidbodies.mass[i] * rigidbodies.velocities[i].length_squared();
        }
    }

    fn update_rotational_kinetic_energy(&mut self, rigidbodies: &RigidBodySet) {
        self.rotational_kinetic_energy = 0.0;
        for i in 0..rigidbodies.len(){
            self.rotational_kinetic_energy = self.rotational_kinetic_energy + 0.5 * (rigidbodies.inertia[i] * rigidbodies.omega[i]).length_squared();
        }
    }

    fn update_gravitational_potential_energy(&mut self, rigidbodies: &RigidBodySet) {
        self.gravitational_potential_energy = 0.0;
        for i in 0..rigidbodies.len() {
            for j in (i + 1)..rigidbodies.len() {
                let r = rigidbodies.positions[j] - rigidbodies.positions[i];
                let distance = r.length();
                self.gravitational_potential_energy = self.gravitational_potential_energy - 0.5 * rigidbodies.mass[i] * rigidbodies.mass[j] / distance;
            }
        }
    }
}