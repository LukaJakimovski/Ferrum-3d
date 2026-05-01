use ferrum_core::math::{Float, Quat, Vec3};
use crate::collision_mesh::{CollisionFace, CollisionSubShape};

#[derive(Clone, Copy, Debug)]
pub struct ContactPoint {
    pub position: Vec3,
    pub depth: Float,
}

pub fn polygon_area(verts: &[Vec3]) -> Float {
    if verts.len() < 3 { return 0.0; }
    let mut area = Vec3::ZERO;
    for i in 1..verts.len() - 1 {
        area += (verts[i] - verts[0]).cross(verts[i + 1] - verts[0]);
    }
    area.length() * 0.5
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
    // Transform verts into world space
    let verts_a: Vec<Vec3> = shape_a.verts.iter().map(|&v| rot_a * v + pos_a).collect();
    let verts_b: Vec<Vec3> = shape_b.verts.iter().map(|&v| rot_b * v + pos_b).collect();

    // Find reference face on A: face whose normal is most aligned with contact normal
    let ref_face = best_face_indexed(&shape_a.faces, &verts_a, normal).unwrap();

    // Find incident face on B: face whose normal is most anti-aligned
    let inc_face = best_face_indexed(&shape_b.faces, &verts_b, -normal).unwrap();

    let ref_verts: Vec<Vec3> = ref_face.verts.iter().map(|&i| verts_a[i]).collect();
    let inc_verts: Vec<Vec3> = inc_face.verts.iter().map(|&i| verts_b[i]).collect();

    // Clip incident face against side planes of reference face
    let clipped = sutherland_hodgman(&inc_verts, &ref_verts, normal);

    // Reference plane depth
    let ref_d = ref_verts[0].dot(normal);

    let mut contacts: Vec<ContactPoint> = clipped
        .into_iter()
        .filter_map(|p| {
            let depth = ref_d - p.dot(normal);
            if depth >= -1e-4 {
                Some(ContactPoint {
                    position: p,
                    depth: depth.max(0.0),
                })
            } else {
                None
            }
        })
        .collect();

    // Reduce to 4 maximally spread points
    if contacts.len() > 4 {
        contacts = reduce_manifold(contacts, 4);
    }

    contacts
}

fn best_face_indexed<'a>(
    faces: &'a [CollisionFace],
    verts: &[Vec3],
    dir: Vec3,
) -> Option<&'a CollisionFace> {
    faces.iter().max_by(|a, b| {
        // Use the face normal transformed to world space to find best alignment.
        // Since verts are already in world space, recompute normal from verts
        // to avoid needing to rotate stored normals separately.
        let normal_a = face_normal_from_verts(a, verts);
        let normal_b = face_normal_from_verts(b, verts);
        normal_a.dot(dir)
            .partial_cmp(&normal_b.dot(dir))
            .unwrap_or(std::cmp::Ordering::Equal)
    })
}

fn face_normal_from_verts(face: &CollisionFace, verts: &[Vec3]) -> Vec3 {
    if face.verts.len() < 3 {
        return Vec3::ZERO;
    }
    let a = verts[face.verts[0]];
    let b = verts[face.verts[1]];
    let c = verts[face.verts[2]];
    (b - a).cross(c - a).normalize_or_zero()
}

fn reduce_manifold(mut points: Vec<ContactPoint>, max_points: usize) -> Vec<ContactPoint> {
    if points.len() <= max_points {
        return points;
    }

    let centroid = points.iter().fold(Vec3::ZERO, |a, p| a + p.position)
        / points.len() as Float;

    let mut result = Vec::with_capacity(max_points);

    let first = points
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| {
            (a.position - centroid).length_squared()
                .partial_cmp(&(b.position - centroid).length_squared())
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|(i, _)| i)
        .unwrap();
    result.push(points.remove(first));

    while result.len() < max_points && !points.is_empty() {
        let next = points
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| {
                let dist_a = result.iter()
                    .map(|r| (a.position - r.position).length_squared())
                    .fold(Float::INFINITY, Float::min);
                let dist_b = result.iter()
                    .map(|r| (b.position - r.position).length_squared())
                    .fold(Float::INFINITY, Float::min);
                dist_a.partial_cmp(&dist_b).unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(i, _)| i)
            .unwrap();
        result.push(points.remove(next));
    }

    result
}

/// Sutherland-Hodgman polygon clipping.
///
/// Clips `subject` (the incident face polygon) against each side plane of
/// `clip_face` (the reference face polygon).  The side planes are the planes
/// that contain each edge of the reference face and are perpendicular to the
/// reference face's plane (whose normal is `face_normal`).
fn sutherland_hodgman(subject: &[Vec3], clip_face: &[Vec3], face_normal: Vec3) -> Vec<Vec3> {
    if clip_face.len() < 2 || subject.is_empty() {
        return subject.to_vec();
    }

    let mut output = subject.to_vec();

    let n = clip_face.len();
    for i in 0..n {
        if output.is_empty() {
            break;
        }
        let edge_start = clip_face[i];
        let edge_end = clip_face[(i + 1) % n];

        // Side-plane normal: perpendicular to the edge, lying in the reference
        // face plane, pointing inward.
        let edge_dir = edge_end - edge_start;
        let plane_normal = face_normal.cross(edge_dir).normalize_or_zero();

        let input = output.clone();
        output.clear();

        for j in 0..input.len() {
            let current = input[j];
            let previous = input[(j + n - 1) % input.len()];

            let d_current = plane_normal.dot(current - edge_start);
            let d_previous = plane_normal.dot(previous - edge_start);

            if d_current >= 0.0 {
                // Current vertex is inside.
                if d_previous < 0.0 {
                    // Previous was outside — add intersection.
                    output.push(line_plane_intersect(previous, current, plane_normal, edge_start));
                }
                output.push(current);
            } else if d_previous >= 0.0 {
                // Current outside, previous inside — add intersection.
                output.push(line_plane_intersect(previous, current, plane_normal, edge_start));
            }
        }
    }

    output
}

/// Finds the intersection of the line segment [a, b] with the plane defined by
/// `plane_normal` and a point `plane_point` on the plane.
fn line_plane_intersect(a: Vec3, b: Vec3, plane_normal: Vec3, plane_point: Vec3) -> Vec3 {
    let ab = b - a;
    let denom = plane_normal.dot(ab);
    if denom.abs() < 1e-10 {
        return a;
    }
    let t = plane_normal.dot(plane_point - a) / denom;
    a + ab * t.clamp(0.0, 1.0)
}