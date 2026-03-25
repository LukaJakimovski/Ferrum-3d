
use ferrum_core::math::{Float, Vec3};

/// Returns the point in `shape` that is furthest in direction `dir`.
fn support(shape: &[Vec3], dir: Vec3) -> Vec3 {
    shape
        .iter()
        .copied()
        .max_by(|a, b| {
            a.dot(dir)
                .partial_cmp(&b.dot(dir))
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .expect("shape must not be empty")
}

/// Minkowski difference support: furthest point of (A – B) in direction `dir`.
fn minkowski_support(a: &[Vec3], b: &[Vec3], dir: Vec3) -> Vec3 {
    support(a, dir) - support(b, -dir)
}


#[derive(Clone, Debug)]
struct Simplex {
    points: [Vec3; 4],
    size: usize,
}

impl Simplex {
    fn new(initial: Vec3) -> Self {
        Self {
            points: [initial, Vec3::ZERO, Vec3::ZERO, Vec3::ZERO],
            size: 1,
        }
    }

    fn push(&mut self, p: Vec3) {
        // Shift existing points up and put the newest point at index 0
        // (index 0 is always the point added most recently)
        self.points[3] = self.points[2];
        self.points[2] = self.points[1];
        self.points[1] = self.points[0];
        self.points[0] = p;
        self.size = (self.size + 1).min(4);
    }

    fn a(&self) -> Vec3 { self.points[0] }
    fn b(&self) -> Vec3 { self.points[1] }
    fn c(&self) -> Vec3 { self.points[2] }
    fn d(&self) -> Vec3 { self.points[3] }
}


/// Updates the simplex so that it is the sub-simplex closest to the origin,
/// and returns the next search direction.
/// Returns `None` when the simplex already contains the origin.
fn do_simplex(simplex: &mut Simplex) -> Option<Vec3> {
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
            simplex.points[1] = c;
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
        simplex.points[1] = c;
        simplex.points[2] = b;
        Some(-abc)
    }
}

/// Shared helper: reduce to line AB and return search direction.
fn line_case_ab(simplex: &mut Simplex, a: Vec3, b: Vec3, ao: Vec3) -> Option<Vec3> {
    if ab_toward_origin(a, b, ao) {
        simplex.points[1] = b;
        simplex.size = 2;
        let ab = b - a;
        Some(ab.cross(ao).cross(ab))
    } else {
        simplex.size = 1;
        Some(ao)
    }
}

fn ab_toward_origin(a: Vec3, b: Vec3, ao: Vec3) -> bool {
    (b - a).dot(ao) > 0.0
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

    // Check which face the origin is "above" and reduce accordingly
    if abc.dot(ao) > 0.0 {
        // Origin above face ABC – discard D, recurse as triangle ABC
        simplex.points[3] = Vec3::ZERO;
        simplex.size = 3;
        return triangle_case(simplex);
    }
    if acd.dot(ao) > 0.0 {
        // Origin above face ACD – discard B, recurse as triangle ACD
        simplex.points[1] = c;
        simplex.points[2] = d;
        simplex.points[3] = Vec3::ZERO;
        simplex.size = 3;
        return triangle_case(simplex);
    }
    if adb.dot(ao) > 0.0 {
        // Origin above face ADB – discard C, recurse as triangle ADB
        simplex.points[1] = d;
        simplex.points[2] = b;
        simplex.points[3] = Vec3::ZERO;
        simplex.size = 3;
        return triangle_case(simplex);
    }

    // Origin is inside the tetrahedron – intersection!
    None
}


/// Returns `true` if the two convex polygons (given as point clouds) intersect.
pub fn gjk_intersects(shape_a: &[Vec3], shape_b: &[Vec3], offset: Vec3) -> bool {
    assert!(!shape_a.is_empty(), "shape_a must not be empty");
    assert!(!shape_b.is_empty(), "shape_b must not be empty");

    let shape_b: Vec<Vec3> = shape_b.iter()
        .map(|x| x + offset)
        .collect();
    let shape_b = &*shape_b;

    // Initial search direction: difference of centroids
    let centroid_a: Vec3 = shape_a.iter().copied().sum::<Vec3>() / shape_a.len() as Float;
    let centroid_b: Vec3 = shape_b.iter().copied().sum::<Vec3>() / shape_b.len() as Float;
    let mut dir = centroid_a - centroid_b;

    if dir.length_squared() < 1e-10 {
        dir = Vec3::X;
    }

    let first = minkowski_support(shape_a, shape_b, dir);
    let mut simplex = Simplex::new(first);
    dir = -first; // search toward the origin

    const MAX_ITER: usize = 64;
    for _ in 0..MAX_ITER {
        let new_point = minkowski_support(shape_a, shape_b, dir);

        // If the new point did not pass the origin, no intersection
        if new_point.dot(dir) < 0.0 {
            return false;
        }

        simplex.push(new_point);

        match do_simplex(&mut simplex) {
            None => return true, // simplex contains origin
            Some(new_dir) => {
                if new_dir.length_squared() < 1e-10 {
                    // Direction collapsed – origin is on the boundary
                    return true;
                }
                dir = new_dir;
            }
        }
    }
    false
}