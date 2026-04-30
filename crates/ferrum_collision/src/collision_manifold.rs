use ferrum_core::math::{Float, Vec3};

#[derive(Clone, Copy, Debug)]
pub struct ContactPoint {
    pub position: Vec3,
    pub depth: Float,
}

/// Finds contact points for two convex shapes that are known to be penetrating.
///
/// Strategy: clip each shape's most-anti-parallel face (the "reference" and
/// "incident" faces) against the side-planes of the reference face, then keep
/// the clipped incident vertices that are below the reference plane.  This is
/// the standard Sutherland-Hodgman manifold reduction used in most narrow-phase
/// solvers.
///
/// `normal`  – contact normal pointing from B toward A (world space).
/// `pos_a/b` – world-space origins of the two bodies.
pub fn find_contact_manifold(
    shape_a: &[Vec3],
    rot_a: ferrum_core::math::Quat,
    pos_a: Vec3,
    shape_b: &[Vec3],
    rot_b: ferrum_core::math::Quat,
    pos_b: Vec3,
    normal: Vec3,
) -> Vec<ContactPoint> {
    // Rotate into world space.
    let verts_a: Vec<Vec3> = shape_a.iter().map(|&v| rot_a * v + pos_a).collect();
    let verts_b: Vec<Vec3> = shape_b.iter().map(|&v| rot_b * v + pos_b).collect();

    // Find the reference face on A (most aligned with normal)
    let ref_face_a = best_face(&verts_a, normal);

    // Find the incident face on B (most anti-aligned with normal)
    let inc_face_b = best_face(&verts_b, -normal);

    // Clip the incident face against the side planes of the reference
    let clipped = sutherland_hodgman(&inc_face_b, &ref_face_a, normal);

    // Keep only vertices that are on or below the reference plane
    // Reference plane: dot(p, normal) = dot(ref_face_a[0], normal)
    let ref_d = ref_face_a[0].dot(normal);

    clipped
        .into_iter()
        .filter_map(|p| {
            let depth = ref_d - p.dot(normal);
            if depth >= -1e-4 {
                // Contact point halfway between the two surfaces.
                Some(ContactPoint {
                    position: p + normal * (depth * 0.5),
                    depth: depth.max(0.0),
                })
            } else {
                None
            }
        })
        .collect()
}

/// Returns the face of the convex hull
/// whose outward normal is most aligned with `dir`.
///
/// For a convex polyhedron stored as a flat vertex soup we approximate the
/// "face" by finding the support vertex and then collecting all vertices whose
/// projection onto `dir` is within a small epsilon of the maximum projection.
/// This works for box-like and sphere-approximate shapes; for exact face
/// topology you would need an **index buffer**.
fn best_face(verts: &[Vec3], dir: Vec3) -> Vec<Vec3> {
    let max_proj = verts
        .iter()
        .map(|v| v.dot(dir))
        .fold(Float::NEG_INFINITY, Float::max);

    const FACE_EPS: Float = 1e-3;
    let face: Vec<Vec3> = verts
        .iter()
        .copied()
        .filter(|v| v.dot(dir) >= max_proj - FACE_EPS)
        .collect();

    if face.is_empty() {
        // Fallback: just the single support vertex.
        vec![*verts
            .iter()
            .max_by(|a, b| {
                a.dot(dir)
                    .partial_cmp(&b.dot(dir))
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap()]
    } else {
        face
    }
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