use ferrum_core::math::{Float, Vec3};
use crate::gjk::{do_simplex, minkowski_support, ConvexShape, Simplex, SupportPoint};

#[derive(Debug, Clone, Copy)]
pub struct Contact {
    /// Unit normal pointing from shape A toward shape B.
    /// Moving B by `normal * depth` (or A by the opposite) separates the shapes.
    pub normal: Vec3,
    pub depth: Float,
    /// Deepest point of A inside B (world space).
    pub point_a: Vec3,
    /// Deepest point of B inside A (world space).
    pub point_b: Vec3,
}

#[derive(Clone, Copy)]
struct EpaFace {
    verts: [usize; 3],
    normal: Vec3,
    dist: Float,
}

impl EpaFace {
    /// Faces are wound counter-clockwise when seen from outside the polytope,
    /// so the normal always points outward without having to look at the origin.
    fn new(verts: [usize; 3], points: &[SupportPoint]) -> Option<Self> {
        let a = points[verts[0]].p;
        let b = points[verts[1]].p;
        let c = points[verts[2]].p;

        let cross = (b - a).cross(c - a);
        let len = cross.length();

        // Reject degenerate (zero-area) faces
        if len < 1e-12 {
            return None;
        }

        let normal = cross / len;
        Some(Self { verts, normal, dist: normal.dot(a) })
    }
}

/// Grow a GJK simplex that ended with fewer than four points (the origin lies on
/// its boundary) into a tetrahedron so that EPA has a volume to expand.
fn blow_up_simplex(simplex: &Simplex, shape_a: &ConvexShape, shape_b: &ConvexShape) -> Option<[SupportPoint; 4]> {
    let mut points: Vec<SupportPoint> = simplex.points[..simplex.size].to_vec();

    const EPS: Float = 1e-10;

    if points.len() == 1 {
        let candidate = [Vec3::X, -Vec3::X, Vec3::Y, -Vec3::Y, Vec3::Z, -Vec3::Z]
            .into_iter()
            .map(|d| minkowski_support(shape_a, shape_b, d))
            .find(|s| (s.p - points[0].p).length_squared() > EPS)?;
        points.push(candidate);
    }

    if points.len() == 2 {
        let line = (points[1].p - points[0].p).normalize();
        let axis = if line.x.abs() < 0.57 { Vec3::X } else if line.y.abs() < 0.57 { Vec3::Y } else { Vec3::Z };
        let u = line.cross(axis).normalize();
        let v = line.cross(u);
        let candidate = (0..6)
            .map(|k| {
                let angle = k as Float * std::f64::consts::FRAC_PI_3 as Float;
                minkowski_support(shape_a, shape_b, u * angle.cos() + v * angle.sin())
            })
            .find(|s| (s.p - points[0].p).cross(line).length_squared() > EPS)?;
        points.push(candidate);
    }

    if points.len() == 3 {
        let n = (points[1].p - points[0].p).cross(points[2].p - points[0].p).normalize_or_zero();
        if n == Vec3::ZERO {
            return None;
        }
        let candidate = [n, -n]
            .into_iter()
            .map(|d| minkowski_support(shape_a, shape_b, d))
            .find(|s| (s.p - points[0].p).dot(n).abs() > 1e-8)?;
        points.push(candidate);
    }

    Some([points[0], points[1], points[2], points[3]])
}

fn build_initial_polytope(mut tetra: [SupportPoint; 4]) -> Option<Vec<EpaFace>> {
    // Orient the tetrahedron so face (0, 1, 2) points away from point 3.
    let orientation = (tetra[1].p - tetra[0].p)
        .cross(tetra[2].p - tetra[0].p)
        .dot(tetra[3].p - tetra[0].p);
    if orientation.abs() < 1e-12 {
        return None;
    }
    if orientation > 0.0 {
        tetra.swap(1, 2);
    }

    let mut faces = Vec::with_capacity(4);
    for tri in [[0, 1, 2], [0, 3, 1], [0, 2, 3], [1, 3, 2]] {
        let face = EpaFace::new(tri, &tetra)?;
        // The origin must be inside (or on) the polytope for EPA to be valid.
        if face.dist < -1e-8 {
            return None;
        }
        faces.push(face);
    }
    Some(faces)
}

/// Closest point to the origin on triangle `abc`, as barycentric weights.
fn barycentric_of_origin_projection(a: Vec3, b: Vec3, c: Vec3, normal: Vec3) -> (Float, Float, Float) {
    let p = normal * normal.dot(a);
    let v0 = b - a;
    let v1 = c - a;
    let v2 = p - a;
    let d00 = v0.dot(v0);
    let d01 = v0.dot(v1);
    let d11 = v1.dot(v1);
    let d20 = v2.dot(v0);
    let d21 = v2.dot(v1);
    let denom = d00 * d11 - d01 * d01;
    if denom.abs() < 1e-20 {
        return (1.0, 0.0, 0.0);
    }
    let v = (d11 * d20 - d01 * d21) / denom;
    let w = (d00 * d21 - d01 * d20) / denom;
    (1.0 - v - w, v, w)
}

fn face_to_contact(face: &EpaFace, points: &[SupportPoint]) -> Contact {
    let [i, j, k] = face.verts;
    let (u, v, w) = barycentric_of_origin_projection(points[i].p, points[j].p, points[k].p, face.normal);
    Contact {
        normal: face.normal,
        depth: face.dist.max(0.0),
        point_a: points[i].a * u + points[j].a * v + points[k].a * w,
        point_b: points[i].b * u + points[j].b * v + points[k].b * w,
    }
}

// EPA
const EPA_MAX_ITER: usize = 64;
const EPA_TOLERANCE: Float = 1e-6;

/// `simplex` must be the **final GJK simplex** that contains the origin
/// Returns the penetration `Contact` (normal + depth + witness points)
pub fn epa(simplex: &Simplex, shape_a: &ConvexShape, shape_b: &ConvexShape) -> Option<Contact> {
    let tetra = if simplex.size == 4 {
        [simplex.points[0], simplex.points[1], simplex.points[2], simplex.points[3]]
    } else {
        blow_up_simplex(simplex, shape_a, shape_b)?
    };
    let mut faces = build_initial_polytope(tetra)?;
    let mut points: Vec<SupportPoint> = tetra.to_vec();
    let mut horizon: Vec<(usize, usize)> = Vec::with_capacity(16);

    for _ in 0..EPA_MAX_ITER {
        // Find the face closest to the origin
        let closest = *faces.iter().min_by(|a, b| a.dist.total_cmp(&b.dist))?;

        // Find the support point along the closest face's normal
        let support = minkowski_support(shape_a, shape_b, closest.normal);
        let new_dist = support.p.dot(closest.normal);

        // Converged: the new support point is not meaningfully further than the face
        if new_dist - closest.dist < EPA_TOLERANCE * closest.dist.max(1.0) {
            return Some(face_to_contact(&closest, &points));
        }

        let support_idx = points.len();
        points.push(support);

        // Remove every face visible from the support point and collect the
        // boundary (horizon) of the hole. An edge shared by two removed faces
        // shows up once in each direction and cancels out.
        horizon.clear();
        faces.retain(|face| {
            let visible = face.normal.dot(support.p - points[face.verts[0]].p) > 0.0;
            if visible {
                for (a, b) in [
                    (face.verts[0], face.verts[1]),
                    (face.verts[1], face.verts[2]),
                    (face.verts[2], face.verts[0]),
                ] {
                    if let Some(pos) = horizon.iter().position(|&e| e == (b, a)) {
                        horizon.swap_remove(pos);
                    } else {
                        horizon.push((a, b));
                    }
                }
            }
            !visible
        });

        // Stitch new faces from the horizon edges to the support point. They keep
        // the winding of the removed faces, so their normals point outward.
        for &(a, b) in &horizon {
            match EpaFace::new([a, b, support_idx], &points) {
                Some(face) => faces.push(face),
                // A sliver face would leave a hole in the polytope; the closest
                // face found so far is as good as this is going to get.
                None => return Some(face_to_contact(&closest, &points)),
            }
        }
    }

    // Iteration limit hit — return the best face found so far
    let closest = faces.iter().min_by(|a, b| a.dist.total_cmp(&b.dist))?;
    Some(face_to_contact(closest, &points))
}

pub enum GjkResult {
    Separated,
    Intersecting(Contact),
}

pub fn gjk_epa(shape_a: &ConvexShape, shape_b: &ConvexShape) -> GjkResult {
    debug_assert!(!shape_a.verts.is_empty());
    debug_assert!(!shape_b.verts.is_empty());

    let mut dir = shape_b.pos - shape_a.pos;
    if dir.length_squared() < 1e-10 {
        dir = Vec3::X;
    }

    let first = minkowski_support(shape_a, shape_b, dir);
    let mut simplex = Simplex::new(first);
    dir = -first.p;

    const MAX_ITER: usize = 64;
    for _ in 0..MAX_ITER {
        if dir.length_squared() < 1e-20 {
            // The origin lies on the simplex: the shapes are just touching.
            break;
        }

        let new_point = minkowski_support(shape_a, shape_b, dir);
        if new_point.p.dot(dir) < 0.0 {
            return GjkResult::Separated;
        }

        simplex.push(new_point);

        match do_simplex(&mut simplex) {
            None => break,
            Some(new_dir) => dir = new_dir,
        }
    }

    match epa(&simplex, shape_a, shape_b) {
        Some(contact) => GjkResult::Intersecting(contact),
        // Degenerate (touching with no measurable depth): nothing to resolve.
        None => GjkResult::Separated,
    }
}
