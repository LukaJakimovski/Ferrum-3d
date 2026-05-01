use ferrum_collision::aabb::Aabb;
use ferrum_collision::epa::{gjk_epa, Contact, GjkResult};
use crate::rigidbody_set::RigidBodySet;
use crate::Physics;
use ferrum_core::math::{Float, Mat3, Vec3};
use ferrum_collision::collision_manifold::{find_contact_manifold, ContactPoint};

impl Physics {
    pub fn resolve_collisions(&mut self, dt: Float) {
        let n = self.rigidbodies.len();

        for i in 0..n {
            self.rigidbodies.colliding[i] = false;
        }

        for i in 0..n {
            for j in (i+1)..n {
                if i == j {continue;}
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

                for sub_a in shapes_a.shapes.iter() {
                    for sub_b in shapes_b.shapes.iter() {
                        let aabb_a =
                            Aabb::from_shapes(&sub_a.verts).transformed(rot_a, Vec3::ZERO);
                        let aabb_b =
                            Aabb::from_shapes(&sub_b.verts).transformed(rot_b, offset);
                        if !aabb_a.intersects(&aabb_b) {
                            continue;
                        }
                        if let GjkResult::Intersecting(contact) = gjk_epa(&sub_a.verts, rot_a, &sub_b.verts, rot_b, offset) {
                            let is_deeper = deepest.as_ref().map_or(true, |(d, _)| contact.depth > d.depth);
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
                        dt
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
        dt: Float,
    ) {
        let n = contact.normal;
        let m1 = bodies.inv_mass[i];
        let m2 = bodies.inv_mass[j];

        let r_mat_i = Mat3::from_quat(bodies.orientations[i]);
        let r_mat_j = Mat3::from_quat(bodies.orientations[j]);
        let inv_i_i = r_mat_i * bodies.inv_inertia[i] * r_mat_i.transpose();
        let inv_i_j = r_mat_j * bodies.inv_inertia[j] * r_mat_j.transpose();
        let com_i = bodies.positions[i] + bodies.mass_center[i];
        let com_j = bodies.positions[j] + bodies.mass_center[j];
        // --- Positional correction (Baumgarte) ------------------------------------
        const SLOP: Float = 0.005;
        const BAUMGARTE: Float = 0.6; // low — just enough to prevent sinking
        let correction_mag =
            ((contact.depth - SLOP).max(0.0) * BAUMGARTE) / (m1 + m2).max(1e-10);
        bodies.positions[i] -= n * correction_mag * m1;
        bodies.positions[j] += n * correction_mag * m2;

        let e = bodies.restitution[i].min(bodies.restitution[j]);
        let mu = (bodies.friction[i] + bodies.friction[j]) * 0.5;

        for cp in manifold {
            let r_i = cp.position - com_i;
            let r_j = cp.position - com_j;

            let vel_i = bodies.velocities[i] + bodies.omega[i].cross(r_i);
            let vel_j = bodies.velocities[j] + bodies.omega[j].cross(r_j);
            let rel_vel = vel_j - vel_i;
            let vn = rel_vel.dot(n);

            if vn >= 0.0 {
                continue;
            }

            // NORMAL IMPULSE (with rotation)
            let rn_i = r_i.cross(n);
            let rn_j = r_j.cross(n);
            let inv_mass_eff =
                m1 + m2 + (inv_i_i * rn_i).cross(r_i).dot(n) + (inv_i_j * rn_j).cross(r_j).dot(n);

            if inv_mass_eff < 1e-10 {
                continue;
            }
            let e = if vn.abs() < 2.0 * 9.81 * dt { 0.0 } else { e };
            let j_n = -(1.0 + e) * vn / inv_mass_eff;

            let impulse_n = n * j_n;
            bodies.velocities[i] -= impulse_n * m1;
            bodies.velocities[j] += impulse_n * m2;
            bodies.omega[i] -= inv_i_i * r_i.cross(impulse_n);
            bodies.omega[j] += inv_i_j * r_j.cross(impulse_n);


            if e == 0.0 {
                let v_i_after = bodies.velocities[i];
                let v_j_after = bodies.velocities[j];
                let vn_after = (v_j_after - v_i_after).dot(n);
                if vn_after < 0.0 {
                    let cancel = n * vn_after / (m1 + m2).max(1e-10);
                    bodies.velocities[i] += cancel * m1;
                    bodies.velocities[j] -= cancel * m2;
                }
            }

            // FRICTION
            let vel_i = bodies.velocities[i] + bodies.omega[i].cross(r_i);
            let vel_j = bodies.velocities[j] + bodies.omega[j].cross(r_j);

            let rel_vel = vel_j - vel_i;

            let tangent = (rel_vel - n * rel_vel.dot(n)).normalize_or_zero();
            let vt = rel_vel.dot(tangent);

            let rt_i = r_i.cross(tangent);
            let rt_j = r_j.cross(tangent);

            let inv_mass_eff_t =
                m1 + m2 + (inv_i_i * rt_i).cross(r_i).dot(tangent) + (inv_i_j * rt_j).cross(r_j).dot(tangent);

            if inv_mass_eff_t < 1e-10 {
                continue;
            }

            let j_t = (-vt / inv_mass_eff_t)
                .clamp(-mu * j_n, mu * j_n);

            let impulse_t = tangent * j_t;

            bodies.velocities[i] -= impulse_t * m1;
            bodies.velocities[j] += impulse_t * m2;
            bodies.omega[i] -= inv_i_i * r_i.cross(impulse_t);
            bodies.omega[j] += inv_i_j * r_j.cross(impulse_t);
        }
    }
}
