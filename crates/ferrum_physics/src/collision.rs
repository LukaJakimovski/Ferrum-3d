use ferrum_collision::aabb::Aabb;
use ferrum_collision::epa::{gjk_epa, Contact, GjkResult};
use crate::rigidbody_set::RigidBodySet;
use crate::Physics;
use ferrum_core::math::{Float, Vec3};

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
                let shapes_a = &self.collision_meshes[mesh_a];
                let shapes_b = &self.collision_meshes[mesh_b];
                let mesh_a = &self.polyhedrons[mesh_a];
                let mesh_b = &self.polyhedrons[mesh_b];

                let rot_a = self.rigidbodies.orientations[i];
                let rot_b = self.rigidbodies.orientations[j];
                let offset = self.rigidbodies.positions[j] - self.rigidbodies.positions[i];

                let aabb_a = Aabb::from_shapes(&mesh_a.vert).transformed(rot_a, Vec3::ZERO);
                let aabb_b = Aabb::from_shapes(&mesh_b.vert).transformed(rot_b, offset);
                if !aabb_a.intersects(&aabb_b) {
                    continue;
                }

                let mut deepest: Option<Contact> = None;

                for sub_a in shapes_a.vert.iter() {
                    for sub_b in shapes_b.vert.iter() {
                        let aabb_a = Aabb::from_shapes(sub_a).transformed(rot_a, Vec3::ZERO);
                        let aabb_b = Aabb::from_shapes(sub_b).transformed(rot_b, offset);
                        if !aabb_a.intersects(&aabb_b) {
                            continue;
                        }
                        if let GjkResult::Intersecting(contact) =
                            gjk_epa(sub_a, rot_a, sub_b, rot_b, offset)
                        {
                            let deeper = deepest
                                .map_or(true, |d: Contact| contact.depth > d.depth);
                            if deeper {
                                deepest = Some(contact);
                            }
                        }
                    }
                }

                if let Some(contact) = deepest {
                    self.rigidbodies.colliding[i] = true;
                    self.rigidbodies.colliding[j] = true;

                    Self::apply_collision_response(
                        &mut self.rigidbodies,
                        i, j,
                        contact,
                    );
                }
            }
        }
    }
    pub fn apply_collision_response(
        bodies: &mut RigidBodySet,
        i: usize,
        j: usize,
        contact: Contact,
    ) {
        let n = contact.normal; // points i ← j
        const SLOP: Float = 0.01;
        const BAUMGARTE: Float = 0.8;
        let m1 = bodies.inv_mass[i];
        let m2 = bodies.inv_mass[j];
        let v1 = bodies.velocities[i];
        let v2 = bodies.velocities[j];

        let correction_mag = ((contact.depth - SLOP).max(0.0) * BAUMGARTE)
            / (m1 + m2).max(1e-10);
        let correction = n * correction_mag;
        bodies.positions[i] -= correction * m1;
        bodies.positions[j] += correction * m2;

        // Impulse
        let rel_vel = v2 - v1;
        let vel_along_normal = rel_vel.dot(n);

        if vel_along_normal > 0.0 {
            return;
        }

        let e = bodies.restitution[i].min(bodies.restitution[j]);

        let impulse_scalar = -(1.0 + e) * vel_along_normal
            / (m1 + m2);

        let impulse = n * impulse_scalar;
        bodies.velocities[i] -= impulse * m1;
        bodies.velocities[j] += impulse * m2;
    }
}