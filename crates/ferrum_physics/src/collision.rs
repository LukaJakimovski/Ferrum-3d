use ferrum_collision::aabb::Aabb;
use ferrum_collision::collision_manifold::find_contact_manifold;
use ferrum_collision::epa::{gjk_epa, GjkResult};
use ferrum_collision::gjk::ConvexShape;
use crate::rigidbody_set::RigidBodySet;
use crate::Physics;
use ferrum_core::math::{Float, Mat3, Vec3};

/// Sequential impulse passes over all contacts per step.
const VELOCITY_ITERATIONS: usize = 10;
/// Position projection passes over all contacts per step.
const POSITION_ITERATIONS: usize = 4;
/// Penetration that is allowed to remain, keeps resting contacts from jittering.
const SLOP: Float = 0.005;
/// Fraction of the remaining penetration removed by each position pass.
const BAUMGARTE: Float = 0.3;
/// Approach speeds below this don't bounce, so resting bodies settle.
const RESTITUTION_THRESHOLD: Float = 0.5;

/// A single contact point between two bodies, prepared for the solver.
struct ContactConstraint {
    a: usize,
    b: usize,
    /// Points from body `a` toward body `b`.
    normal: Vec3,
    tangents: [Vec3; 2],
    r_a: Vec3,
    r_b: Vec3,
    depth: Float,
    normal_mass: Float,
    tangent_mass: [Float; 2],
    /// Desired normal velocity after solving (restitution bounce, or the
    /// allowed approach speed for a point that isn't touching yet).
    target_velocity: Float,
    friction: Float,
    normal_impulse: Float,
    tangent_impulse: [Float; 2],
}

/// A contact found by the narrowphase, before solver data is computed.
struct RawContact {
    a: usize,
    b: usize,
    normal: Vec3,
    position: Vec3,
    depth: Float,
}

impl Physics {
    pub fn resolve_collisions(&mut self, dt: Float) {
        let n = self.rigidbodies.len();
        for i in 0..n {
            self.rigidbodies.colliding[i] = false;
        }

        let contacts = self.find_contacts();
        if contacts.is_empty() {
            return;
        }
        for c in &contacts {
            self.rigidbodies.colliding[c.a] = true;
            self.rigidbodies.colliding[c.b] = true;
        }

        let bodies = &mut self.rigidbodies;
        let inv_inertia_world: Vec<Mat3> = (0..n)
            .map(|i| {
                if bodies.inv_mass[i] == 0.0 {
                    return Mat3::ZERO;
                }
                let r = Mat3::from_quat(bodies.orientations[i]);
                r * bodies.inv_inertia[i] * r.transpose()
            })
            .collect();

        let mut constraints: Vec<ContactConstraint> = contacts
            .iter()
            .filter_map(|c| Self::prepare_contact(bodies, &inv_inertia_world, c, dt))
            .collect();

        for _ in 0..VELOCITY_ITERATIONS {
            for c in constraints.iter_mut() {
                Self::solve_contact_velocity(bodies, &inv_inertia_world, c);
            }
        }

        Self::correct_positions(bodies, &constraints);
    }

    /// Broadphase (sweep and prune on body AABBs), midphase (sub-shape AABBs)
    /// and narrowphase (GJK/EPA + face clipping) for every pair of bodies.
    fn find_contacts(&self) -> Vec<RawContact> {
        let bodies = &self.rigidbodies;
        let n = bodies.len();

        let body_aabbs: Vec<Aabb> = (0..n)
            .map(|i| {
                self.collision_meshes[bodies.mesh[i]]
                    .aabb
                    .transformed(bodies.orientations[i], bodies.positions[i])
            })
            .collect();

        let mut order: Vec<usize> = (0..n).collect();
        order.sort_by(|&i, &j| body_aabbs[i].min.x.total_cmp(&body_aabbs[j].min.x));

        // World space bounds of every sub-shape, only computed for bodies that
        // survive the broadphase.
        let mut sub_aabbs: Vec<Option<Vec<Aabb>>> = vec![None; n];
        let mut contacts = Vec::new();

        for (k, &i) in order.iter().enumerate() {
            for &j in &order[k + 1..] {
                if body_aabbs[j].min.x > body_aabbs[i].max.x {
                    break;
                }
                if !body_aabbs[i].intersects(&body_aabbs[j]) {
                    continue;
                }
                if bodies.inv_mass[i] == 0.0 && bodies.inv_mass[j] == 0.0 {
                    continue;
                }
                let (a, b) = (i.min(j), i.max(j));

                for body in [a, b] {
                    if sub_aabbs[body].is_none() {
                        let (rot, pos) = (bodies.orientations[body], bodies.positions[body]);
                        sub_aabbs[body] = Some(
                            self.collision_meshes[bodies.mesh[body]]
                                .shapes
                                .iter()
                                .map(|s| s.aabb.transformed(rot, pos))
                                .collect(),
                        );
                    }
                }

                self.collide_pair(a, b, &body_aabbs, &sub_aabbs, &mut contacts);
            }
        }

        contacts
    }

    fn collide_pair(
        &self,
        a: usize,
        b: usize,
        body_aabbs: &[Aabb],
        sub_aabbs: &[Option<Vec<Aabb>>],
        contacts: &mut Vec<RawContact>,
    ) {
        let bodies = &self.rigidbodies;
        let shapes_a = &self.collision_meshes[bodies.mesh[a]].shapes;
        let shapes_b = &self.collision_meshes[bodies.mesh[b]].shapes;
        let (Some(sub_a), Some(sub_b)) = (&sub_aabbs[a], &sub_aabbs[b]) else {
            return;
        };

        let (rot_a, pos_a) = (bodies.orientations[a], bodies.positions[a]);
        let (rot_b, pos_b) = (bodies.orientations[b], bodies.positions[b]);

        // Only sub-shapes that reach into the other body's bounds can touch it.
        let candidates_b: Vec<usize> = (0..shapes_b.len())
            .filter(|&s| sub_b[s].intersects(&body_aabbs[a]))
            .collect();
        if candidates_b.is_empty() {
            return;
        }

        for (sa, shape_a) in shapes_a.iter().enumerate() {
            if !sub_a[sa].intersects(&body_aabbs[b]) {
                continue;
            }
            let convex_a = ConvexShape::from_sub_shape(shape_a, rot_a, pos_a);

            for &sb in &candidates_b {
                if !sub_a[sa].intersects(&sub_b[sb]) {
                    continue;
                }
                let shape_b = &shapes_b[sb];
                let convex_b = ConvexShape::from_sub_shape(shape_b, rot_b, pos_b);

                let GjkResult::Intersecting(contact) = gjk_epa(&convex_a, &convex_b) else {
                    continue;
                };

                let manifold = find_contact_manifold(
                    shape_a, rot_a, pos_a,
                    shape_b, rot_b, pos_b,
                    &contact,
                );
                contacts.extend(manifold.into_iter().map(|cp| RawContact {
                    a,
                    b,
                    normal: contact.normal,
                    position: cp.position,
                    depth: cp.depth,
                }));
            }
        }
    }

    fn prepare_contact(
        bodies: &RigidBodySet,
        inv_inertia_world: &[Mat3],
        contact: &RawContact,
        dt: Float,
    ) -> Option<ContactConstraint> {
        let (a, b, n) = (contact.a, contact.b, contact.normal);
        let (m_a, m_b) = (bodies.inv_mass[a], bodies.inv_mass[b]);
        let (inv_i_a, inv_i_b) = (inv_inertia_world[a], inv_inertia_world[b]);

        let com_a = bodies.positions[a] + bodies.orientations[a] * bodies.mass_center[a];
        let com_b = bodies.positions[b] + bodies.orientations[b] * bodies.mass_center[b];
        let r_a = contact.position - com_a;
        let r_b = contact.position - com_b;

        let effective_mass = |dir: Vec3| -> Float {
            let ra_d = r_a.cross(dir);
            let rb_d = r_b.cross(dir);
            let k = m_a + m_b + (inv_i_a * ra_d).dot(ra_d) + (inv_i_b * rb_d).dot(rb_d);
            if k > 1e-12 { 1.0 / k } else { 0.0 }
        };

        let normal_mass = effective_mass(n);
        if normal_mass == 0.0 {
            return None;
        }

        let rel_vel = Self::relative_velocity(bodies, a, b, r_a, r_b);
        let vn = rel_vel.dot(n);

        // Friction directions: along the current sliding direction when there is one.
        let sliding = rel_vel - n * vn;
        let t1 = if sliding.length_squared() > 1e-12 {
            sliding.normalize()
        } else {
            n.any_orthonormal_vector()
        };
        let t2 = n.cross(t1);

        let target_velocity = if contact.depth < 0.0 {
            // Not touching yet: allow it to close the gap this step, but no further.
            contact.depth / dt
        } else {
            let e = bodies.restitution[a].min(bodies.restitution[b]);
            if vn < -RESTITUTION_THRESHOLD { -e * vn } else { 0.0 }
        };

        Some(ContactConstraint {
            a,
            b,
            normal: n,
            tangents: [t1, t2],
            r_a,
            r_b,
            depth: contact.depth,
            normal_mass,
            tangent_mass: [effective_mass(t1), effective_mass(t2)],
            target_velocity,
            friction: (bodies.friction[a] + bodies.friction[b]) * 0.5,
            normal_impulse: 0.0,
            tangent_impulse: [0.0, 0.0],
        })
    }

    #[inline]
    fn relative_velocity(bodies: &RigidBodySet, a: usize, b: usize, r_a: Vec3, r_b: Vec3) -> Vec3 {
        let vel_a = bodies.velocities[a] + bodies.omega[a].cross(r_a);
        let vel_b = bodies.velocities[b] + bodies.omega[b].cross(r_b);
        vel_b - vel_a
    }

    #[inline]
    fn apply_impulse(bodies: &mut RigidBodySet, inv_inertia_world: &[Mat3], c: &ContactConstraint, impulse: Vec3) {
        bodies.velocities[c.a] -= impulse * bodies.inv_mass[c.a];
        bodies.velocities[c.b] += impulse * bodies.inv_mass[c.b];
        bodies.omega[c.a] -= inv_inertia_world[c.a] * c.r_a.cross(impulse);
        bodies.omega[c.b] += inv_inertia_world[c.b] * c.r_b.cross(impulse);
    }

    fn solve_contact_velocity(bodies: &mut RigidBodySet, inv_inertia_world: &[Mat3], c: &mut ContactConstraint) {
        // --- Friction (clamped to the friction cone of the current normal impulse) ---
        let rel_vel = Self::relative_velocity(bodies, c.a, c.b, c.r_a, c.r_b);
        let old = c.tangent_impulse;
        let mut new = [
            old[0] - rel_vel.dot(c.tangents[0]) * c.tangent_mass[0],
            old[1] - rel_vel.dot(c.tangents[1]) * c.tangent_mass[1],
        ];
        let max_friction = c.friction * c.normal_impulse;
        let len = (new[0] * new[0] + new[1] * new[1]).sqrt();
        if len > max_friction {
            let scale = if len > 0.0 { max_friction / len } else { 0.0 };
            new = [new[0] * scale, new[1] * scale];
        }
        c.tangent_impulse = new;
        let friction_impulse = c.tangents[0] * (new[0] - old[0]) + c.tangents[1] * (new[1] - old[1]);
        Self::apply_impulse(bodies, inv_inertia_world, c, friction_impulse);

        // --- Normal (accumulated impulse may only ever push) ---
        let rel_vel = Self::relative_velocity(bodies, c.a, c.b, c.r_a, c.r_b);
        let vn = rel_vel.dot(c.normal);
        let lambda = (c.target_velocity - vn) * c.normal_mass;
        let new_impulse = (c.normal_impulse + lambda).max(0.0);
        let delta = new_impulse - c.normal_impulse;
        c.normal_impulse = new_impulse;
        Self::apply_impulse(bodies, inv_inertia_world, c, c.normal * delta);
    }

    /// Push overlapping bodies apart. Contacts are processed iteratively and see
    /// the corrections already applied, so a pair with many contact points is
    /// not pushed apart once per point.
    fn correct_positions(bodies: &mut RigidBodySet, constraints: &[ContactConstraint]) {
        let mut shift = vec![Vec3::ZERO; bodies.len()];

        for _ in 0..POSITION_ITERATIONS {
            for c in constraints {
                let (m_a, m_b) = (bodies.inv_mass[c.a], bodies.inv_mass[c.b]);
                let total = m_a + m_b;
                if total <= 0.0 {
                    continue;
                }
                let depth = c.depth - (shift[c.b] - shift[c.a]).dot(c.normal);
                let error = depth - SLOP;
                if error <= 0.0 {
                    continue;
                }
                let correction = c.normal * (BAUMGARTE * error / total);
                shift[c.a] -= correction * m_a;
                shift[c.b] += correction * m_b;
            }
        }

        for (pos, s) in bodies.positions.iter_mut().zip(shift) {
            *pos += s;
        }
    }
}
