use ferrum_core::math::{Float, Vec3};
use crate::aabb::Aabb;

#[derive(Clone, Default, PartialEq)]
#[derive(Debug)]
pub struct CollisionFace {
    /// Outward facing unit normal (local space).
    pub normal: Vec3,
    /// Indices into CollisionSubShape::verts, wound counter-clockwise around `normal`.
    pub verts: Vec<usize>,
}

#[derive(Default, Clone, PartialEq)]
pub struct CollisionSubShape {
    pub verts: Vec<Vec3>,
    pub faces: Vec<CollisionFace>,
    /// Local space bounds of `verts`.
    pub aabb: Aabb,
    /// Hull edges: `neighbors[i]` lists the vertices connected to vertex `i`.
    /// Used to find support points by hill climbing instead of scanning every vertex.
    pub neighbors: Vec<Vec<usize>>,
}

impl CollisionSubShape {
    /// Build a convex sub-shape from its hull vertices and (possibly triangulated) faces.
    ///
    /// Coplanar faces are merged into a single polygon and every face normal is oriented
    /// away from the hull's interior, which is what the contact manifold clipping relies on.
    pub fn new(verts: Vec<Vec3>, faces: Vec<CollisionFace>) -> Self {
        let aabb = Aabb::from_shapes(&verts);
        let polygons = build_polygon_faces(&verts, &faces);
        let neighbors = build_neighbors(&verts, faces.iter().chain(&polygons));
        Self { verts, faces: polygons, aabb, neighbors }
    }
}

/// Index of the vertex furthest along `dir`, found by walking hull edges from `start`.
///
/// On a convex hull a vertex that none of its neighbours improves on is the global maximum.
#[inline]
pub(crate) fn hill_climb_support(verts: &[Vec3], neighbors: &[Vec<usize>], start: usize, dir: Vec3) -> usize {
    let mut best = start;
    let mut best_dot = verts[best].dot(dir);
    loop {
        let mut improved = false;
        for &n in &neighbors[best] {
            let d = verts[n].dot(dir);
            if d > best_dot {
                best_dot = d;
                best = n;
                improved = true;
            }
        }
        if !improved {
            return best;
        }
    }
}

/// Vertex adjacency for hill climbing support queries. Returns an empty list
/// (meaning: scan every vertex) when hill climbing can't be trusted on this
/// hull, e.g. because the source mesh isn't a clean closed convex surface.
fn build_neighbors<'a>(verts: &[Vec3], faces: impl Iterator<Item = &'a CollisionFace>) -> Vec<Vec<usize>> {
    let mut neighbors = vec![Vec::new(); verts.len()];
    for face in faces {
        if face.verts.iter().any(|&i| i >= verts.len()) {
            continue;
        }
        for (k, &a) in face.verts.iter().enumerate() {
            let b = face.verts[(k + 1) % face.verts.len()];
            if a != b && !neighbors[a].contains(&b) {
                neighbors[a].push(b);
                neighbors[b].push(a);
            }
        }
    }

    // Check against a brute force search over directions spread evenly over the
    // sphere, starting from varying vertices like the warm-started queries do.
    let connected: Vec<usize> = (0..verts.len()).filter(|&i| !neighbors[i].is_empty()).collect();
    if connected.is_empty() {
        return vec![];
    }
    let mut start = connected[0];
    const SAMPLES: usize = 512;
    let golden_angle = std::f64::consts::PI as Float * (3.0 - (5.0 as Float).sqrt());
    for k in 0..SAMPLES {
        let y = 1.0 - 2.0 * (k as Float + 0.5) / SAMPLES as Float;
        let r = (1.0 - y * y).sqrt();
        let theta = golden_angle * k as Float;
        let dir = Vec3::new(r * theta.cos(), y, r * theta.sin());

        let best = verts.iter().map(|v| v.dot(dir)).fold(Float::NEG_INFINITY, Float::max);
        let climbed = hill_climb_support(verts, &neighbors, start, dir);
        if verts[climbed].dot(dir) < best - 1e-9 {
            return vec![];
        }
        start = if k % 2 == 0 { climbed } else { connected[(k * 7919) % connected.len()] };
    }
    neighbors
}

pub struct CollisionMesh {
    pub shapes: Vec<CollisionSubShape>,
    /// Local space bounds of every sub-shape.
    pub aabb: Aabb,
}

impl CollisionMesh {
    pub fn new(shapes: Vec<CollisionSubShape>) -> Self {
        let aabb = shapes
            .iter()
            .map(|s| s.aabb)
            .reduce(|a, b| a.union(&b))
            .unwrap_or_default();
        Self { shapes, aabb }
    }
}

/// Group the input faces by plane and turn each plane into one convex polygon.
fn build_polygon_faces(verts: &[Vec3], faces: &[CollisionFace]) -> Vec<CollisionFace> {
    if verts.len() < 4 {
        return vec![];
    }

    let centroid = verts.iter().copied().sum::<Vec3>() / verts.len() as Float;
    let extent = verts.iter().map(|v| (*v - centroid).length()).fold(0.0, Float::max);
    let plane_eps = (extent * 1e-4).max(1e-9);
    const NORMAL_EPS: Float = 1e-5;

    // Outward plane of every valid input face.
    let mut planes: Vec<(Vec3, Float, &[usize])> = Vec::with_capacity(faces.len());
    for face in faces {
        if face.verts.len() < 3 || face.verts.iter().any(|&i| i >= verts.len()) {
            continue;
        }
        let a = verts[face.verts[0]];
        let b = verts[face.verts[1]];
        let c = verts[face.verts[2]];
        let mut normal = (b - a).cross(c - a).normalize_or_zero();
        if normal == Vec3::ZERO {
            continue;
        }
        if normal.dot(a - centroid) < 0.0 {
            normal = -normal;
        }
        planes.push((normal, normal.dot(a), &face.verts));
    }

    // Group coplanar faces. Sorting by normal.x means only a short run of
    // neighbours needs comparing for each face.
    let mut order: Vec<usize> = (0..planes.len()).collect();
    order.sort_by(|&i, &j| planes[i].0.x.total_cmp(&planes[j].0.x));
    let mut group = vec![usize::MAX; planes.len()];
    let mut groups: Vec<Vec<usize>> = Vec::new();
    for (k, &i) in order.iter().enumerate() {
        if group[i] != usize::MAX {
            continue;
        }
        group[i] = groups.len();
        let mut members = vec![i];
        for &j in &order[k + 1..] {
            if planes[j].0.x - planes[i].0.x > NORMAL_EPS {
                break;
            }
            if group[j] == usize::MAX
                && planes[i].0.dot(planes[j].0) > 1.0 - NORMAL_EPS
                && (planes[i].1 - planes[j].1).abs() < plane_eps
            {
                group[j] = groups.len();
                members.push(j);
            }
        }
        groups.push(members);
    }

    groups
        .into_iter()
        .filter_map(|members| {
            let normal = members.iter().map(|&i| planes[i].0).sum::<Vec3>().normalize();

            // Every vertex of the merged faces, skipping duplicate positions.
            let mut candidates: Vec<usize> = Vec::new();
            for &i in members.iter().flat_map(|&m| planes[m].2) {
                if !candidates.iter().any(|&j| (verts[j] - verts[i]).length_squared() < plane_eps * plane_eps) {
                    candidates.push(i);
                }
            }

            let polygon = convex_polygon(verts, &candidates, normal, plane_eps);
            if polygon.len() < 3 {
                return None;
            }

            Some(CollisionFace { normal, verts: polygon })
        })
        .collect()
}

/// Counter-clockwise (around `normal`) convex hull of coplanar points, using
/// Andrew's monotone chain in the plane. Interior and collinear points are dropped.
fn convex_polygon(verts: &[Vec3], points: &[usize], normal: Vec3, eps: Float) -> Vec<usize> {
    if points.len() < 3 {
        return vec![];
    }
    let u = normal.any_orthonormal_vector();
    let w = normal.cross(u);
    let mut pts: Vec<(Float, Float, usize)> = points
        .iter()
        .map(|&i| (verts[i].dot(u), verts[i].dot(w), i))
        .collect();
    pts.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));

    // > 0 when o -> a -> b turns counter-clockwise.
    let cross = |o: &(Float, Float, usize), a: &(Float, Float, usize), b: &(Float, Float, usize)| {
        (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0)
    };
    let area_eps = eps * eps;

    let mut hull: Vec<(Float, Float, usize)> = Vec::with_capacity(pts.len() + 1);
    for pass in 0..2 {
        let start = hull.len();
        let iter: Box<dyn Iterator<Item = &(Float, Float, usize)>> =
            if pass == 0 { Box::new(pts.iter()) } else { Box::new(pts.iter().rev()) };
        for p in iter {
            while hull.len() >= start + 2 && cross(&hull[hull.len() - 2], &hull[hull.len() - 1], p) <= area_eps {
                hull.pop();
            }
            hull.push(*p);
        }
        // The last point of each chain is the first point of the next one.
        hull.pop();
    }
    hull.into_iter().map(|p| p.2).collect()
}
