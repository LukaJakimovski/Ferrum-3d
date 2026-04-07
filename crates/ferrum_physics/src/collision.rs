use ferrum_collision::aabb::Aabb;
use ferrum_collision::gjk::gjk_intersects;
use ferrum_core::math::Vec3;
use crate::Physics;

impl Physics {
    pub fn resolve_collisions(&mut self) {
        let n = self.rigidbodies.len();

        // Reset flags.
        for i in 0..n {
            self.rigidbodies.colliding[i] = false;
        }

        for i in 0..n {
            for j in (i + 1)..n {
                let mesh_a = self.rigidbodies.mesh[i];
                let mesh_b = self.rigidbodies.mesh[j];
                let shapes_a = &self.collision_meshes[mesh_a]; // &Vec<Vec<Vec3>>
                let shapes_b = &self.collision_meshes[mesh_b];
                let mesh_a = &self.polyhedrons[mesh_a]; // &Vec<Vec<Vec3>>
                let mesh_b = &self.polyhedrons[mesh_b];

                let rot_a = self.rigidbodies.orientations[i];
                let rot_b = self.rigidbodies.orientations[j];
                let offset = self.rigidbodies.positions[j] - self.rigidbodies.positions[i];

                // ---- Broadphase AABB check ----
                // Build world-space AABBs for each body and skip GJK entirely
                // if they don't overlap.
                let aabb_a = Aabb::from_shapes(&mesh_a.vert).transformed(rot_a, Vec3::ZERO);
                let aabb_b = Aabb::from_shapes(&mesh_b.vert).transformed(rot_b, offset);
                if !aabb_a.intersects(&aabb_b) {
                    continue;
                }

                // ---- Narrowphase: test each convex sub-shape pair ----
                // For concave meshes, a collision exists if ANY sub-shape pair collides.
                // We must NOT overwrite a true result with a later false one.
                'outer: for sub_a in shapes_a.vert.iter() {
                    for sub_b in shapes_b.vert.iter() {
                        let aabb_a = Aabb::from_shapes(sub_a).transformed(rot_a, Vec3::ZERO);
                        let aabb_b = Aabb::from_shapes(sub_b).transformed(rot_b, offset);
                        if !aabb_a.intersects(&aabb_b) {
                            continue;
                        }
                        if gjk_intersects(sub_a, rot_a, sub_b, rot_b, offset) {
                            // Mark both bodies and stop testing this pair immediately.
                            self.rigidbodies.colliding[i] = true;
                            self.rigidbodies.colliding[j] = true;
                            break 'outer;
                        }
                    }
                }
            }
        }
    }
}