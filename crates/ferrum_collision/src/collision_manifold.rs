use ferrum_core::math::{Float, Quat, Vec3};
use crate::collision_mesh::{CollisionFace, CollisionSubShape};
use crate::epa::Contact;

#[derive(Clone, Copy, Debug)]
pub struct ContactPoint {
    pub position: Vec3,
    /// Penetration depth along the contact normal. Slightly negative for points
    /// that are within `CONTACT_MARGIN` of touching but not yet overlapping.
    pub depth: Float,
}

/// A face whose normal is within ~10° of the contact normal is treated as a
/// face contact and clipped. Anything else (edge-edge, vertex-edge) uses the
/// single deepest point reported by EPA.
const FACE_CONTACT_TOLERANCE: Float = 0.985;
/// Clipped points this close to the reference face are kept as contacts so a
/// resting face doesn't flicker between 1 and 4 contact points.
const CONTACT_MARGIN: Float = 0.01;
const MAX_CONTACTS: usize = 4;

/// Face of `shape` whose (local) normal is most aligned with `local_dir`.
fn most_aligned_face(shape: &CollisionSubShape, local_dir: Vec3) -> Option<(&CollisionFace, Float)> {
    shape
        .faces
        .iter()
        .map(|f| (f, f.normal.dot(local_dir)))
        .max_by(|a, b| a.1.total_cmp(&b.1))
}

pub fn find_contact_manifold(
    shape_a: &CollisionSubShape,
    rot_a: Quat,
    pos_a: Vec3,
    shape_b: &CollisionSubShape,
    rot_b: Quat,
    pos_b: Vec3,
    contact: &Contact,
) -> Vec<ContactPoint> {
    let normal = contact.normal;
    let deepest = ContactPoint {
        position: (contact.point_a + contact.point_b) * 0.5,
        depth: contact.depth,
    };

    // The normal points from A to B, so A touches with the face facing +normal
    // and B with the face facing -normal.
    let face_a = most_aligned_face(shape_a, rot_a.conjugate() * normal);
    let face_b = most_aligned_face(shape_b, rot_b.conjugate() * -normal);

    let (Some((face_a, align_a)), Some((face_b, align_b))) = (face_a, face_b) else {
        return vec![deepest];
    };
    if align_a.max(align_b) < FACE_CONTACT_TOLERANCE {
        return vec![deepest];
    }

    // The better aligned face becomes the reference face; the other shape
    // supplies the incident face that gets clipped against it.
    let (ref_face, ref_verts, ref_rot, ref_pos, inc_shape, inc_rot, inc_pos) = if align_b > align_a + 1e-3 {
        (face_b, &shape_b.verts, rot_b, pos_b, shape_a, rot_a, pos_a)
    } else {
        (face_a, &shape_a.verts, rot_a, pos_a, shape_b, rot_b, pos_b)
    };

    let ref_normal = ref_rot * ref_face.normal;
    let reference: Vec<Vec3> = ref_face.verts.iter().map(|&i| ref_rot * ref_verts[i] + ref_pos).collect();

    // Incident face: the face of the other shape most anti-parallel to the reference normal.
    let Some((inc_face, _)) = most_aligned_face(inc_shape, inc_rot.conjugate() * -ref_normal) else {
        return vec![deepest];
    };
    let mut incident: Vec<Vec3> = inc_face.verts.iter().map(|&i| inc_rot * inc_shape.verts[i] + inc_pos).collect();

    // --- Clip against the side planes of the reference face ---
    let mut scratch = Vec::with_capacity(incident.len() + reference.len());
    for i in 0..reference.len() {
        let a = reference[i];
        let b = reference[(i + 1) % reference.len()];
        let edge = b - a;
        if edge.length_squared() < 1e-20 {
            continue;
        }
        // Faces are wound counter-clockwise, so this points into the face.
        let inward = ref_normal.cross(edge);
        clip_by_plane(&incident, inward, a, &mut scratch);
        std::mem::swap(&mut incident, &mut scratch);
        if incident.is_empty() {
            break;
        }
    }

    // --- Keep the points below (or very near) the reference plane ---
    let plane_offset = ref_normal.dot(reference[0]);
    let mut contacts: Vec<ContactPoint> = incident
        .iter()
        .filter_map(|&p| {
            let separation = ref_normal.dot(p) - plane_offset;
            (separation <= CONTACT_MARGIN).then(|| ContactPoint {
                // Halfway between the incident point and the reference face.
                position: p - ref_normal * (separation * 0.5),
                depth: -separation,
            })
        })
        .collect();

    if contacts.iter().all(|c| c.depth <= 0.0) {
        return vec![deepest];
    }

    if contacts.len() > MAX_CONTACTS {
        contacts = reduce_contacts(&contacts, ref_normal);
    }
    contacts
}

/// Sutherland–Hodgman: keep the part of `polygon` on the side `plane_normal` points to.
fn clip_by_plane(polygon: &[Vec3], plane_normal: Vec3, plane_point: Vec3, out: &mut Vec<Vec3>) {
    out.clear();
    let n = polygon.len();

    for i in 0..n {
        let curr = polygon[i];
        let prev = polygon[(i + n - 1) % n];

        let dc = plane_normal.dot(curr - plane_point);
        let dp = plane_normal.dot(prev - plane_point);

        if dc >= 0.0 {
            if dp < 0.0 {
                out.push(prev + (curr - prev) * (dp / (dp - dc)));
            }
            out.push(curr);
        } else if dp >= 0.0 {
            out.push(prev + (curr - prev) * (dp / (dp - dc)));
        }
    }
}

/// Keep the deepest point plus the three points that span the largest area,
/// so the remaining contacts still support the whole face.
fn reduce_contacts(contacts: &[ContactPoint], normal: Vec3) -> Vec<ContactPoint> {
    let pick = |score: &dyn Fn(&ContactPoint) -> Float| -> usize {
        (0..contacts.len())
            .max_by(|&i, &j| score(&contacts[i]).total_cmp(&score(&contacts[j])))
            .unwrap()
    };

    let i0 = pick(&|c| c.depth);
    let p0 = contacts[i0].position;

    let i1 = pick(&|c| (c.position - p0).length_squared());
    let p1 = contacts[i1].position;

    // Largest triangle (signed, either winding) with the first two points.
    let signed_area = |a: Vec3, b: Vec3, c: Vec3| (b - a).cross(c - a).dot(normal);
    let i2 = pick(&|c| signed_area(p0, p1, c.position).abs());
    let p2 = contacts[i2].position;

    // Fourth point: the one furthest outside the triangle, on the opposite side of p0-p1 from p2.
    let flip = if signed_area(p0, p1, p2) < 0.0 { -1.0 } else { 1.0 };
    let i3 = pick(&|c| {
        let q = c.position;
        -(signed_area(p0, p1, q) * flip)
            .min(signed_area(p1, p2, q) * flip)
            .min(signed_area(p2, p0, q) * flip)
    });

    let mut out = vec![contacts[i0]];
    for i in [i1, i2, i3] {
        if !out.iter().any(|c| (c.position - contacts[i].position).length_squared() < 1e-12) {
            out.push(contacts[i]);
        }
    }
    out
}
