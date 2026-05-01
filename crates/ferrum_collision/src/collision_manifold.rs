use ferrum_core::math::{Float, Quat, Vec3};
use crate::collision_mesh::{CollisionFace, CollisionSubShape};

#[derive(Clone, Copy, Debug)]
pub struct ContactPoint {
    pub position: Vec3,
    pub depth: Float,
}

fn face_score(face: &CollisionFace, verts: &[Vec3], normal: Vec3) -> Float {
    let mut max_proj = Float::NEG_INFINITY;

    for &i in &face.verts {
        let v = verts[i];
        let proj = v.dot(normal);
        if proj > max_proj {
            max_proj = proj;
        }
    }

    max_proj
}

pub fn find_contact_manifold(
    shape_a: &CollisionSubShape,
    rot_a: Quat,
    pos_a: Vec3,
    shape_b: &CollisionSubShape,
    rot_b: Quat,
    pos_b: Vec3,
    normal: Vec3,
) -> Vec<ContactPoint> {

    let verts_a: Vec<Vec3> = shape_a.verts.iter().map(|&v| rot_a * v + pos_a).collect();
    let verts_b: Vec<Vec3> = shape_b.verts.iter().map(|&v| rot_b * v + pos_b).collect();

    // Reference face (A) — furthest along normal
    let ref_face = shape_a.faces.iter()
        .max_by(|a, b| {
            face_score(a, &verts_a, normal)
                .partial_cmp(&face_score(b, &verts_a, normal))
                .unwrap()
        });

    // Incident face (B) — furthest opposite
    let inc_face = shape_b.faces.iter()
        .min_by(|a, b| {
            face_score(a, &verts_b, normal)
                .partial_cmp(&face_score(b, &verts_b, normal))
                .unwrap()
        });

    let (Some(ref_face), Some(inc_face)) = (ref_face, inc_face) else {
        return vec![];
    };

    let reference: Vec<Vec3> = ref_face.verts.iter().map(|&i| verts_a[i]).collect();
    let mut incident: Vec<Vec3> = inc_face.verts.iter().map(|&i| verts_b[i]).collect();

    if reference.len() < 3 || incident.len() < 3 {
        return vec![];
    }

    let ref_normal = face_normal(&reference);

    // --- Clip against side planes ---
    for i in 0..reference.len() {
        let a = reference[i];
        let b = reference[(i + 1) % reference.len()];

        let edge = b - a;

        let plane_n = ref_normal.cross(edge).normalize_or_zero();

        incident = clip_by_plane(&incident, plane_n, a);

        if incident.is_empty() {
            break; // don't early-return — we want fallback later
        }
    }

    // --- Clip against reference plane ---
    let plane_offset = ref_normal.dot(reference[0]);

    let mut contacts = Vec::new();

    for p in &incident {
        let dist = ref_normal.dot(*p) - plane_offset;

        if dist <= 0.01 {
            contacts.push(ContactPoint {
                position: *p,
                depth: -dist,
            });
        }
    }

    if contacts.is_empty() {

        // --- Find best edges on both shapes ---
        let edge_a = best_edge(ref_face, &verts_a, normal);
        let edge_b = best_edge(inc_face, &verts_b, normal);

        let (pa, pb) = closest_points_on_segments(
            edge_a.0, edge_a.1,
            edge_b.0, edge_b.1
        );

        let contact_point = (pa + pb) * 0.5;

        // Depth from separation along normal
        let depth = (pb - pa).dot(normal).abs().max(0.001);

        return vec![ContactPoint {
            position: contact_point,
            depth,
        }];
    }

    // --- Reduce to max 4 contacts ---
    if contacts.len() > 4 {
        contacts = reduce_contacts(contacts);
    }
    contacts
}

fn face_normal(verts: &[Vec3]) -> Vec3 {
    (verts[1] - verts[0]).cross(verts[2] - verts[0]).normalize_or_zero()
}

fn clip_by_plane(polygon: &[Vec3], plane_normal: Vec3, plane_point: Vec3) -> Vec<Vec3> {
    let mut out = Vec::new();
    let n = polygon.len();

    for i in 0..n {
        let curr = polygon[i];
        let prev = polygon[(i + n - 1) % n];

        let dc = plane_normal.dot(curr - plane_point);
        let dp = plane_normal.dot(prev - plane_point);

        if dc >= 0.0 {
            if dp < 0.0 {
                out.push(intersect_plane(prev, curr, plane_normal, plane_point));
            }
            out.push(curr);
        } else if dp >= 0.0 {
            out.push(intersect_plane(prev, curr, plane_normal, plane_point));
        }
    }

    out
}

fn intersect_plane(a: Vec3, b: Vec3, n: Vec3, p: Vec3) -> Vec3 {
    let ab = b - a;
    let denom = n.dot(ab);

    if denom.abs() < 1e-10 {
        return a;
    }

    let t = n.dot(p - a) / denom;
    a + ab * t.clamp(0.0, 1.0)
}

fn reduce_contacts(mut contacts: Vec<ContactPoint>) -> Vec<ContactPoint> {
    contacts.sort_by(|a, b| b.depth.partial_cmp(&a.depth).unwrap());
    contacts.truncate(4);
    contacts
}

fn closest_points_on_segments(a0: Vec3, a1: Vec3, b0: Vec3, b1: Vec3) -> (Vec3, Vec3) {
    let d1 = a1 - a0;
    let d2 = b1 - b0;
    let r = a0 - b0;

    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);

    let mut s;
    let mut t;

    if a <= 1e-8 && e <= 1e-8 {
        return (a0, b0);
    }

    if a <= 1e-8 {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = d1.dot(r);

        if e <= 1e-8 {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = d1.dot(d2);
            let denom = a * e - b * b;

            if denom != 0.0 {
                s = ((b * f - c * e) / denom).clamp(0.0, 1.0);
            } else {
                s = 0.0;
            }

            t = (b * s + f) / e;

            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }
        }
    }

    let p_a = a0 + d1 * s;
    let p_b = b0 + d2 * t;

    (p_a, p_b)
}

fn best_edge(face: &CollisionFace, verts: &[Vec3], normal: Vec3) -> (Vec3, Vec3) {
    let mut best_dot = Float::NEG_INFINITY;
    let mut best_edge = (verts[face.verts[0]], verts[face.verts[1]]);

    let n = face.verts.len();

    for i in 0..n {
        let a = verts[face.verts[i]];
        let b = verts[face.verts[(i + 1) % n]];

        let edge_dir = (b - a).normalize_or_zero();
        let dot = edge_dir.dot(normal).abs();

        if dot > best_dot {
            best_dot = dot;
            best_edge = (a, b);
        }
    }

    best_edge
}