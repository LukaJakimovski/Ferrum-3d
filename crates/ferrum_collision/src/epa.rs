use std::collections::HashMap;
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
        let a = points[verts[0]];
        let b = points[verts[1]];
        let c = points[verts[2]];

        let cross = (b - a).cross(c - a);
        let len = cross.length();

        // Reject degenerate (zero-area) faces
        if len < 1e-10 {
            return None;
        }

        let normal = cross / len;

        // Ensure the normal points away from the origin.
        // All EPA faces must satisfy this invariant or the visibility
        // test in the main loop produces wrong results over time.
        let (normal, dist) = if normal.dot(a) >= 0.0 {
            (normal, normal.dot(a))
        } else {
            (-normal, (-normal).dot(a))
        };

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
        let closest = faces
            .iter()
            .min_by(|a, b| a.dist.partial_cmp(&b.dist).unwrap_or(std::cmp::Ordering::Less))?
            .clone();

        // Find the support point along the closest face's normal
        let support =
            minkowski_support_rotated(shape_a, rot_a, shape_b, rot_b, offset, closest.normal);
        let new_dist = support.dot(closest.normal);

        // Converged: the new support point is not meaningfully further than the face
        if new_dist - closest.dist < EPA_TOLERANCE {
            return Some(Contact {
                normal: closest.normal,
                depth: closest.dist,
            });
        }

        // Partition faces into those visible from the support point (to be removed)
        // and those that are not (to be kept).
        let support_idx = points.len();
        points.push(support);

        let mut visible_edges: Vec<Edge> = Vec::new();
        let mut kept_faces: Vec<EpaFace> = Vec::new();

        for face in faces.drain(..) {
            let to_support = support - points[face.verts[0]];
            if face.normal.dot(to_support) > 0.0 {
                // Face is visible from support point — collect its edges
                visible_edges.push(Edge(face.verts[0], face.verts[1]));
                visible_edges.push(Edge(face.verts[1], face.verts[2]));
                visible_edges.push(Edge(face.verts[2], face.verts[0]));
            } else {
                kept_faces.push(face);
            }
        }

        if visible_edges.is_empty() {
            // Support point didn't expand any face — polytope is degenerate
            return None;
        }

        // Build the horizon: edges that appear exactly once across all visible
        // faces. Edges shared by two visible faces are interior and are removed.
        // Uses a canonical key (lo, hi) so (a,b) and (b,a) map to the same slot.
        let mut edge_counts: HashMap<(usize, usize), usize> = HashMap::new();
        for &Edge(a, b) in &visible_edges {
            let key = if a < b { (a, b) } else { (b, a) };
            *edge_counts.entry(key).or_insert(0) += 1;
        }
        let horizon: Vec<Edge> = visible_edges
            .iter()
            .copied()
            .filter(|&Edge(a, b)| {
                let key = if a < b { (a, b) } else { (b, a) };
                edge_counts[&key] == 1
            })
            .collect();

        if horizon.is_empty() {
            // No boundary edges found — polytope has become degenerate
            return None;
        }

        // Stitch new faces from the horizon edges to the support point
        let new_faces: Vec<EpaFace> = horizon
            .iter()
            .filter_map(|edge| EpaFace::new([edge.0, edge.1, support_idx], &points))
            .collect();

        if new_faces.is_empty() {
            // All candidate faces were degenerate (zero area) — bail out
            return None;
        }

        faces = kept_faces;
        faces.extend(new_faces);
    }

    // Iteration limit hit — return the best face found so far
    let closest = faces
        .iter()
        .min_by(|a, b| a.dist.partial_cmp(&b.dist).unwrap_or(std::cmp::Ordering::Less))?;
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