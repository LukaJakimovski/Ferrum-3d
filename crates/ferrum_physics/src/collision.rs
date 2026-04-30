use ferrum_collision::aabb::Aabb;
use ferrum_collision::epa::{gjk_epa, Contact, GjkResult};
use crate::rigidbody_set::RigidBodySet;
use crate::Physics;
use ferrum_core::math::{lerp, Float, Mat3, Vec3};
use ferrum_collision::collision_manifold::{find_contact_manifold, ContactPoint};

impl Physics {
    pub fn resolve_collisions(&mut self, dt: Float) {
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

        let num_contacts = manifold.len().max(1);
        let weight = 1.0 / num_contacts as Float;

        let e = bodies.restitution[i].min(bodies.restitution[j]);
        let mu = (bodies.friction[i] + bodies.friction[j]) * 0.5;

        for cp in manifold {
            let r_i = cp.position - com_i;
            let r_j = cp.position - com_j;

            // Velocity at the contact point on each body (linear + angular).
            let vel_i = bodies.velocities[i] + bodies.omega[i].cross(r_i);
            let vel_j = bodies.velocities[j] + bodies.omega[j].cross(r_j);
            let rel_vel = vel_j - vel_i;
            let vn = rel_vel.dot(n);


            println!("contact pos: {:?}", cp.position);
            println!("com_i: {:?}", com_i);
            println!("r_i: {:?}", r_i);
            println!("rel_vel: {:?}", rel_vel);
            println!("vn: {:?}", vn);
            println!("normal: {:?}", n);
            println!("tangent_vel: {:?}", rel_vel - n * vn);
            
            // Only resolve if bodies are approaching.
            if vn >= 0.0 {
                continue;
            }

            // --- Normal impulse ---------------------------------------------------
            // Effective inverse mass: translational + rotational resistance.
            let rn_i = r_i.cross(n);
            let rn_j = r_j.cross(n);
            let inv_mass_eff =
                m1 + m2 + (inv_i_i * rn_i).cross(r_i).dot(n) + (inv_i_j * rn_j).cross(r_j).dot(n);

            if inv_mass_eff < 1e-10 {
                continue;
            }

            // Zero restitution for slow contacts — prevents gravity accumulation
            // bouncing. Threshold: one frame of freefall.
            let e_actual = if vn.abs() < 2.0 * 9.81 * dt { 0.0 } else { e };
            let j_n = -(1.0 + e_actual) * vn / inv_mass_eff * weight;

            let impulse_n = n * j_n;
            bodies.velocities[i] -= impulse_n * m1;
            bodies.velocities[j] += impulse_n * m2;
            bodies.omega[i] -= inv_i_i * r_i.cross(impulse_n);
            bodies.omega[j] += inv_i_j * r_j.cross(impulse_n);

            // --- Friction impulse -------------------------------------------------
            // Re-sample after normal impulse so we see the corrected velocity.
            let vel_i = bodies.velocities[i] + bodies.omega[i].cross(r_i);
            let vel_j = bodies.velocities[j] + bodies.omega[j].cross(r_j);
            let rel_vel = vel_j - vel_i;

            // Tangential component only.
            let vt_vec = rel_vel - n * rel_vel.dot(n);
            let vt_speed = vt_vec.length();

            if vt_speed < 1e-6 {
                continue;
            }

            let t = vt_vec / vt_speed;

            let rt_i = r_i.cross(t);
            let rt_j = r_j.cross(t);
            let inv_mass_eff_t =
                m1 + m2 + (inv_i_i * rt_i).cross(r_i).dot(t) + (inv_i_j * rt_j).cross(r_j).dot(t);

            if inv_mass_eff_t < 1e-10 {
                continue;
            }

            let j_t_unclamped = -vt_speed / inv_mass_eff_t * weight;

            // Coulomb cone: tangential impulse cannot exceed mu * normal impulse.
            let j_t = j_t_unclamped.clamp(-mu * j_n, mu * j_n);

            let impulse_t = t * j_t;
            bodies.velocities[i] -= impulse_t * m1;
            bodies.velocities[j] += impulse_t * m2;
            bodies.omega[i] -= inv_i_i * r_i.cross(impulse_t);
            bodies.omega[j] += inv_i_j * r_j.cross(impulse_t);
        }

        // --- Fallback for empty manifold ------------------------------------------
        if manifold.is_empty() {
            let rel_vel = bodies.velocities[j] - bodies.velocities[i];
            let vn = rel_vel.dot(n);
            if vn < 0.0 {
                let j_n = -(1.0 + e) * vn / (m1 + m2).max(1e-10);
                let impulse = n * j_n;
                bodies.velocities[i] -= impulse * m1;
                bodies.velocities[j] += impulse * m2;
            }
        }

        // --- Rolling/spinning friction (damping hack) -----------------------------
        // Coulomb friction handles sliding but not rolling or spinning in place.
        // The standard approach per Gaffer on Games is velocity damping that is
        // stronger at low speeds and nearly zero at high speeds, so fast motion
        // is unaffected but resting objects settle naturally.
        if !manifold.is_empty() {
            let speed_i = bodies.omega[i].length();
            let speed_j = bodies.omega[j].length();

            // Damping factor: approaches 1.0 (no damping) at high speed,
            // drops to ~0.85 at rest. Tune LOW_SPEED and the lerp range to taste.
            const LOW_SPEED: Float = 1.0;
            const DAMP_REST: Float = 0.85;
            const DAMP_FAST: Float = 0.9995;

            let factor_i = lerp(DAMP_REST, DAMP_FAST, (speed_i / LOW_SPEED).min(1.0));
            let factor_j = lerp(DAMP_REST, DAMP_FAST, (speed_j / LOW_SPEED).min(1.0));

            bodies.omega[i] *= factor_i;
            bodies.omega[j] *= factor_j;

            // Small linear damping too so sliding objects don't coast forever.
            let lspeed_i = bodies.velocities[i].length();
            let lspeed_j = bodies.velocities[j].length();
            let lfactor_i = lerp(0.9, DAMP_FAST, (lspeed_i / LOW_SPEED).min(1.0));
            let lfactor_j = lerp(0.9, DAMP_FAST, (lspeed_j / LOW_SPEED).min(1.0));
            bodies.velocities[i] *= lfactor_i;
            bodies.velocities[j] *= lfactor_j;
        }
    }
}
