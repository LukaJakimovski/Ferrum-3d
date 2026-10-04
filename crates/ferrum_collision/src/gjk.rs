use std::cell::Cell;
use ferrum_core::math::{Quat, Vec3};
use crate::collision_mesh::{hill_climb_support, CollisionSubShape};

/// Hulls with fewer vertices than this are scanned linearly, which beats
/// hill climbing when everything fits in a few cache lines.
const HILL_CLIMB_MIN_VERTS: usize = 32;

/// A convex vertex cloud placed in world space.
pub struct ConvexShape<'a> {
    pub verts: &'a [Vec3],
    /// Hull adjacency (see `CollisionSubShape::neighbors`); empty for a plain point cloud.
    neighbors: &'a [Vec<usize>],
    pub rot: Quat,
    pub inv_rot: Quat,
    pub pos: Vec3,
    /// Vertex returned by the previous support query. Consecutive GJK/EPA
    /// directions are close, so hill climbing from here takes very few steps.
    last_support: Cell<usize>,
}

impl<'a> ConvexShape<'a> {
    pub fn new(verts: &'a [Vec3], rot: Quat, pos: Vec3) -> Self {
        Self { verts, neighbors: &[], rot, inv_rot: rot.conjugate(), pos, last_support: Cell::new(0) }
    }

    pub fn from_sub_shape(shape: &'a CollisionSubShape, rot: Quat, pos: Vec3) -> Self {
        let mut convex = Self::new(&shape.verts, rot, pos);
        if shape.verts.len() >= HILL_CLIMB_MIN_VERTS && shape.neighbors.len() == shape.verts.len() {
            if let Some(start) = shape.neighbors.iter().position(|n| !n.is_empty()) {
                convex.neighbors = &shape.neighbors;
                convex.last_support.set(start);
            }
        }
        convex
    }

    /// Returns the world space point in the shape that is furthest in direction `dir`.
    ///
    /// The search happens in local space so only the direction and the winning
    /// vertex need to be rotated, not every vertex of the hull.
    #[inline]
    pub fn support(&self, dir: Vec3) -> Vec3 {
        debug_assert!(!self.verts.is_empty());
        let local_dir = self.inv_rot * dir;

        let best = if self.neighbors.is_empty() {
            let mut best = 0;
            let mut best_dot = self.verts[0].dot(local_dir);
            for (i, v) in self.verts.iter().enumerate().skip(1) {
                let d = v.dot(local_dir);
                if d > best_dot {
                    best_dot = d;
                    best = i;
                }
            }
            best
        } else {
            let best = hill_climb_support(self.verts, self.neighbors, self.last_support.get(), local_dir);
            self.last_support.set(best);
            best
        };
        self.rot * self.verts[best] + self.pos
    }
}

/// A point on the Minkowski difference A - B, together with the points on
/// A and B that produced it (needed to recover contact positions).
#[derive(Clone, Copy, Debug, Default)]
pub struct SupportPoint {
    pub p: Vec3,
    pub a: Vec3,
    pub b: Vec3,
}

/// Minkowski difference support: furthest point of (A – B) in direction `dir`.
#[inline]
pub fn minkowski_support(shape_a: &ConvexShape, shape_b: &ConvexShape, dir: Vec3) -> SupportPoint {
    let a = shape_a.support(dir);
    let b = shape_b.support(-dir);
    SupportPoint { p: a - b, a, b }
}

#[derive(Clone, Debug)]
pub struct Simplex {
    pub points: [SupportPoint; 4],
    pub size: usize,
}
impl Simplex {
    pub(crate) fn new(initial: SupportPoint) -> Self {
        Self {
            points: [initial; 4],
            size: 1,
        }
    }

    pub(crate) fn push(&mut self, p: SupportPoint) {
        // Shift existing points up and put the newest point at index 0
        // (index 0 is always the point added most recently)
        self.points[3] = self.points[2];
        self.points[2] = self.points[1];
        self.points[1] = self.points[0];
        self.points[0] = p;
        self.size = (self.size + 1).min(4);
    }

    fn a(&self) -> Vec3 { self.points[0].p }
    fn b(&self) -> Vec3 { self.points[1].p }
    fn c(&self) -> Vec3 { self.points[2].p }
    fn d(&self) -> Vec3 { self.points[3].p }
}


/// Updates the simplex so that it is the sub-simplex closest to the origin,
/// and returns the next search direction.
/// Returns `None` when the simplex already contains the origin.
pub fn do_simplex(simplex: &mut Simplex) -> Option<Vec3> {
    match simplex.size {
        2 => line_case(simplex),
        3 => triangle_case(simplex),
        4 => tetrahedron_case(simplex),
        _ => unreachable!(),
    }
}

/// Line simplex: A is the newest point, B is the older one.
fn line_case(simplex: &mut Simplex) -> Option<Vec3> {
    let a = simplex.a();
    let b = simplex.b();

    let ab = b - a;
    let ao = -a; // direction from A toward the origin

    if ab.dot(ao) > 0.0 {
        // Origin is between A and B – keep both, search perpendicular to AB
        Some(ab.cross(ao).cross(ab))
    } else {
        // Origin is past A – reduce to just A
        simplex.size = 1;
        Some(ao)
    }
}

/// Triangle simplex: A newest, then B, then C.
fn triangle_case(simplex: &mut Simplex) -> Option<Vec3> {
    let a = simplex.a();
    let b = simplex.b();
    let c = simplex.c();

    let ab = b - a;
    let ac = c - a;
    let ao = -a;

    let abc = ab.cross(ac);

    // Edge AC region
    if abc.cross(ac).dot(ao) > 0.0 {
        if ac.dot(ao) > 0.0 {
            // Keep A, C
            simplex.points[1] = simplex.points[2];
            simplex.size = 2;
            return Some(ac.cross(ao).cross(ac));
        }
        // Fall through to star test
        return line_case_ab(simplex, a, b, ao);
    }

    // Edge AB region
    if ab.cross(abc).dot(ao) > 0.0 {
        return line_case_ab(simplex, a, b, ao);
    }

    // Inside the triangle – determine which face side
    if abc.dot(ao) > 0.0 {
        // Above the triangle; keep winding order A, B, C
        Some(abc)
    } else {
        // Below the triangle; swap B and C to flip normal
        simplex.points.swap(1, 2);
        Some(-abc)
    }
}

/// Shared helper: reduce to line AB and return search direction.
fn line_case_ab(simplex: &mut Simplex, a: Vec3, b: Vec3, ao: Vec3) -> Option<Vec3> {
    let ab = b - a;
    if ab.dot(ao) > 0.0 {
        simplex.size = 2;
        Some(ab.cross(ao).cross(ab))
    } else {
        simplex.size = 1;
        Some(ao)
    }
}

/// Tetrahedron simplex: A newest, then B, C, D.
fn tetrahedron_case(simplex: &mut Simplex) -> Option<Vec3> {
    let a = simplex.a();
    let b = simplex.b();
    let c = simplex.c();
    let d = simplex.d();

    let ab = b - a;
    let ac = c - a;
    let ad = d - a;
    let ao = -a;

    let abc = ab.cross(ac);
    let acd = ac.cross(ad);
    let adb = ad.cross(ab);

    let [pa, pb, pc, pd] = simplex.points;

    // Check which face the origin is "above" and reduce accordingly
    if abc.dot(ao) > 0.0 {
        // Origin above face ABC – discard D, recurse as triangle ABC
        simplex.size = 3;
        return triangle_case(simplex);
    }
    if acd.dot(ao) > 0.0 {
        // Origin above face ACD – discard B, recurse as triangle ACD
        simplex.points = [pa, pc, pd, pd];
        simplex.size = 3;
        return triangle_case(simplex);
    }
    if adb.dot(ao) > 0.0 {
        // Origin above face ADB – discard C, recurse as triangle ADB
        simplex.points = [pa, pd, pb, pb];
        simplex.size = 3;
        return triangle_case(simplex);
    }

    // Origin is inside the tetrahedron – intersection!
    None
}
