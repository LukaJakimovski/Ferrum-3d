use ferrum_core::math::{Float, Quat, Vec3};
use crate::gjk::{do_simplex, minkowski_support_rotated, Simplex};

#[derive(Debug, Clone, Copy)]
pub struct Contact {
    pub normal: Vec3,
    pub depth: Float,
}
#[derive(Clone)]
struct EpaFace {
    verts: [usize; 3],
    normal: Vec3,
    dist: Float,
}

impl EpaFace {
    fn new(verts: [usize; 3], points: &[Vec3]) -> Option<Self> {
        let [a, b, c] = verts.map(|i| points[i]);
        let ab = b - a;
        let ac = c - a;
        let cross = ab.cross(ac);
        let len = cross.length();
        if len < 1e-10 {
            return None; 
        }
        let normal = cross / len;
        let dist = normal.dot(a);
        if dist < 0.0 {
            return Some(Self {
                verts: [verts[0], verts[2], verts[1]],
                normal: -normal,
                dist: -dist,
            });
        }
        Some(Self { verts, normal, dist })
    }
}
#[derive(Clone, Copy, PartialEq, Eq)]
struct Edge(usize, usize);

impl Edge {
    fn reversed(self) -> Self {
        Edge(self.1, self.0)
    }
}


fn build_initial_polytope(
    simplex: &Simplex,
    shape_a: &[Vec3],
    rot_a: Quat,
    shape_b: &[Vec3],
    rot_b: Quat,
    offset: Vec3,
) -> Option<(Vec<Vec3>, Vec<EpaFace>)> {
    let mut points: Vec<Vec3> = simplex.points.to_vec();

    while points.len() < 4 {
        let candidate = [
            Vec3::X, Vec3::Y, Vec3::Z,
            -Vec3::X, -Vec3::Y, -Vec3::Z,
        ]
            .into_iter()
            .map(|d| minkowski_support_rotated(shape_a, rot_a, shape_b, rot_b, offset, d))
            .find(|&p| !points.iter().any(|&q| (p - q).length_squared() < 1e-10))?;

        points.push(candidate);
    }

    let face_indices = [
        [0, 1, 2],
        [0, 3, 1],
        [0, 2, 3],
        [1, 3, 2],
    ];

    let faces: Vec<EpaFace> = face_indices
        .into_iter()
        .filter_map(|tri| EpaFace::new(tri, &points))
        .collect();

    if faces.is_empty() {
        return None;
    }

    Some((points, faces))
}

// EPA
const EPA_MAX_ITER: usize = 64;
const EPA_TOLERANCE: Float = 1e-4;

/// `simplex` must be the **final GJK simplex** that contains the origin
/// Returns the penetration `Contact` (normal + depth)
pub fn epa(
    simplex: &Simplex,
    shape_a: &[Vec3],
    rot_a: Quat,
    shape_b: &[Vec3],
    rot_b: Quat,
    offset: Vec3,
) -> Option<Contact> {
    let (mut points, mut faces) =
        build_initial_polytope(simplex, shape_a, rot_a, shape_b, rot_b, offset)?;

    for _ in 0..EPA_MAX_ITER {
        // Find the face closest to the origin
        let (_closest_idx, closest) = faces
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                a.dist.partial_cmp(&b.dist).unwrap_or(std::cmp::Ordering::Equal)
            })?;

        let normal = closest.normal;
        let dist = closest.dist;

        // Find the support point in the closest face's normal direction
        let support = minkowski_support_rotated(shape_a, rot_a, shape_b, rot_b, offset, normal);

        // Check convergence
        let new_dist = support.dot(normal);
        if new_dist - dist < EPA_TOLERANCE {
            return Some(Contact { normal, depth: dist });
        }

        // Expand the polytope
        let support_idx = points.len();
        points.push(support);

        let mut horizon: Vec<Edge> = Vec::new();
        let mut new_faces: Vec<EpaFace> = Vec::new();

        let mut kept_faces: Vec<EpaFace> = Vec::new();
        let mut visible_edges: Vec<Edge> = Vec::new();

        for face in faces.drain(..) {
            let dot = face.normal.dot(support - points[face.verts[0]]);
            if dot > 0.0 {
                visible_edges.push(Edge(face.verts[0], face.verts[1]));
                visible_edges.push(Edge(face.verts[1], face.verts[2]));
                visible_edges.push(Edge(face.verts[2], face.verts[0]));
            } else {
                kept_faces.push(face);
            }
        }

        for &edge in &visible_edges {
            if !visible_edges.contains(&edge.reversed()) {
                horizon.push(edge);
            }
        }

        for edge in &horizon {
            if let Some(face) = EpaFace::new([edge.0, edge.1, support_idx], &points) {
                new_faces.push(face);
            }
        }

        if new_faces.is_empty() {
            return Some(Contact { normal, depth: dist });
        }

        faces = kept_faces;
        faces.extend(new_faces);
    }

    let closest = faces
        .iter()
        .min_by(|a, b| a.dist.partial_cmp(&b.dist).unwrap_or(std::cmp::Ordering::Equal))?;

    Some(Contact {
        normal: closest.normal,
        depth: closest.dist,
    })
}

pub enum GjkResult {
    Separated,
    Intersecting(Contact),
}

pub fn gjk_epa(
    shape_a: &[Vec3],
    rot_a: Quat,
    shape_b: &[Vec3],
    rot_b: Quat,
    offset: Vec3,
) -> GjkResult {
    debug_assert!(!shape_a.is_empty());
    debug_assert!(!shape_b.is_empty());

    let centroid_a: Vec3 =
        shape_a.iter().map(|&v| rot_a * v).sum::<Vec3>() / shape_a.len() as Float;
    let centroid_b: Vec3 =
        shape_b.iter().map(|&v| rot_b * v + offset).sum::<Vec3>() / shape_b.len() as Float;

    let mut dir = centroid_a - centroid_b;
    if dir.length_squared() < 1e-10 {
        dir = Vec3::X;
    }

    let first = minkowski_support_rotated(shape_a, rot_a, shape_b, rot_b, offset, dir);
    let mut simplex = Simplex::new(first);
    dir = -first;

    const MAX_ITER: usize = 64;
    for _ in 0..MAX_ITER {
        let new_point = minkowski_support_rotated(shape_a, rot_a, shape_b, rot_b, offset, dir);

        if new_point.dot(dir) < 0.0 {
            return GjkResult::Separated;
        }

        simplex.push(new_point);

        match do_simplex(&mut simplex) {
            None => {
                let contact = epa(
                    &simplex,
                    shape_a, rot_a,
                    shape_b, rot_b,
                    offset,
                );
                return match contact {
                    Some(c) => GjkResult::Intersecting(c),
                    // Degenerate EPA
                    None => GjkResult::Intersecting(Contact {
                        normal: Vec3::Y,
                        depth: 0.0,
                    }),
                };
            }
            Some(new_dir) => {
                if new_dir.length_squared() < 1e-10 {
                    let contact = epa(
                        &simplex,
                        shape_a, rot_a,
                        shape_b, rot_b,
                        offset,
                    );
                    return match contact {
                        Some(c) => GjkResult::Intersecting(c),
                        None => GjkResult::Intersecting(Contact {
                            normal: Vec3::Y,
                            depth: 0.0,
                        }),
                    };
                }
                dir = new_dir;
            }
        }
    }
    GjkResult::Separated
}