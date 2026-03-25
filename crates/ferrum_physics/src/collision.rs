use ferrum_collision::gjk::gjk_intersects;
use crate::update::Physics;

impl Physics {
    pub fn resolve_collisions(&mut self) {
        for i in 0..self.rigidbodies.len() {
            for j in i + 1..self.rigidbodies.len() {
                let mesh_a = self.rigidbodies.mesh[i];
                let mesh_b = self.rigidbodies.mesh[j];
                let shape_a = &self.polyhedrons[mesh_a];
                let shape_b = &self.polyhedrons[mesh_b];
                let offset = self.rigidbodies.positions[j] - self.rigidbodies.positions[i];
                let result = gjk_intersects(&*shape_a.vert, &*shape_b.vert, offset);
                self.rigidbodies.colliding[i] = result;
                self.rigidbodies.colliding[j] = result;
                if result {
                    println!("Collision!");
                }
            }
        }
    }
}