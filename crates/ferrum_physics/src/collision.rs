use ferrum_collision::aabb::Aabb;
use ferrum_collision::epa::{gjk_epa, Contact, GjkResult};
use crate::rigidbody_set::RigidBodySet;
use crate::Physics;
use ferrum_core::math::{Float, Vec3};
use ferrum_collision::collision_manifold::{find_contact_manifold, ContactPoint};

impl Physics {
    pub fn resolve_collisions(&mut self) {
        let n = self.rigidbodies.len();

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
                let pos_a = self.rigidbodies.positions[i];
                let pos_b = self.rigidbodies.positions[j];
                let offset = pos_b - pos_a;

                let aabb_a = Aabb::from_shapes(&mesh_a.vert).transformed(rot_a, Vec3::ZERO);
                let aabb_b = Aabb::from_shapes(&mesh_b.vert).transformed(rot_b, offset);
                if !aabb_a.intersects(&aabb_b) {
                    continue;
                }

                let mut deepest: Option<(Contact, Vec<ContactPoint>)> = None;

                for sub_a in shapes_a.vert.iter() {
                    for sub_b in shapes_b.vert.iter() {
                        let aabb_a =
                            Aabb::from_shapes(sub_a).transformed(rot_a, Vec3::ZERO);
                        let aabb_b =
                            Aabb::from_shapes(sub_b).transformed(rot_b, offset);
                        if !aabb_a.intersects(&aabb_b) {
                            continue;
                        }
                        if let GjkResult::Intersecting(contact) =
                            gjk_epa(sub_a, rot_a, sub_b, rot_b, offset)
                        {
                            let is_deeper = deepest
                                .as_ref()
                                .map_or(true, |(d, _)| contact.depth > d.depth);

                            if is_deeper {
                                let manifold = find_contact_manifold(
                                    sub_a, rot_a, pos_a,
                                    sub_b, rot_b, pos_b,
                                    contact.normal,
                                );
                                deepest = Some((contact, manifold));
                            }
                        }
                    }
                }

                if let Some((contact, manifold)) = deepest {
                    self.rigidbodies.colliding[i] = true;
                    self.rigidbodies.colliding[j] = true;

                    Self::apply_collision_response(
                        &mut self.rigidbodies,
                        i, j,
                        contact,
                        &manifold,
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
        manifold: &[ContactPoint],
    ) {
        let n = contact.normal; // points from B toward A
        let pos_i = bodies.positions[i];
        let pos_j = bodies.positions[j];

        // --- Positional correction (Baumgarte) --------------------------------
        const SLOP: Float = 0.01;
        const BAUMGARTE: Float = 0.8;

        let m1 = bodies.inv_mass[i];
        let m2 = bodies.inv_mass[j];

        let correction_mag =
            ((contact.depth - SLOP).max(0.0) * BAUMGARTE) / (m1 + m2).max(1e-10);
        let correction = n * correction_mag;
        bodies.positions[i] -= correction * m1;
        bodies.positions[j] += correction * m2;

        // --- Per-contact-point impulse ----------------------------------------
        // We average the impulse over all contact points so that a face-face
        // contact (4 points) doesn't quadruple the total impulse magnitude.
        let num_contacts = manifold.len().max(1);
        let weight = 1.0 / num_contacts as Float;

        let e = bodies.restitution[i].min(bodies.restitution[j]);

        // Inertia tensors rotated into world space:
        //   I_world = R * I_local * R^T
        // `inertia` is stored in local space; we need the world-space inverse.
        // If inv_inertia is already world-space (updated each tick by the
        // integrator) use it directly; otherwise rotate it here.
        let inv_i_i = bodies.inv_inertia[i];
        let inv_i_j = bodies.inv_inertia[j];

        // Centre-of-mass offsets in world space.
        let com_i = pos_i + bodies.mass_center[i];
        let com_j = pos_j + bodies.mass_center[j];

        for cp in manifold {
            // Vectors from each body's CoM to the contact point.
            let r_i = cp.position - com_i;
            let r_j = cp.position - com_j;

            // Relative velocity at the contact point (including angular terms).
            //   v_contact = v_cm + omega × r
            let v_i = bodies.velocities[i] + bodies.omega[i].cross(r_i);
            let v_j = bodies.velocities[j] + bodies.omega[j].cross(r_j);
            let rel_vel = v_j - v_i;

            let vel_along_normal = rel_vel.dot(n);

            // Only resolve if objects are approaching.
            if vel_along_normal >= 0.0 {
                continue;
            }
            
            let ang_i = (inv_i_i * r_i.cross(n)).cross(r_i);
            let ang_j = (inv_i_j * r_j.cross(n)).cross(r_j);
            let inv_mass_eff = m1 + m2 + ang_i.dot(n) + ang_j.dot(n);

            if inv_mass_eff < 1e-10 {
                continue;
            }

            let j_scalar = -(1.0 + e) * vel_along_normal / inv_mass_eff * weight;
            let impulse = n * j_scalar;

            // Linear impulse.
            bodies.velocities[i] -= impulse * m1;
            bodies.velocities[j] += impulse * m2;

            // Angular impulse:  Δω = I^-1 * (r × J)
            bodies.omega[i] -= inv_i_i * r_i.cross(impulse);
            bodies.omega[j] += inv_i_j * r_j.cross(impulse);
        }

        // if manifold was empty, apply a centre-of-mass-only impulse
        if manifold.is_empty() {
            let rel_vel = bodies.velocities[j] - bodies.velocities[i];
            let vel_along_normal = rel_vel.dot(n);
            if vel_along_normal < 0.0 {
                let inv_mass_eff = (m1 + m2).max(1e-10);
                let j_scalar = -(1.0 + e) * vel_along_normal / inv_mass_eff;
                let impulse = n * j_scalar;
                bodies.velocities[i] -= impulse * m1;
                bodies.velocities[j] += impulse * m2;
            }
        }
    }
}
