use ferrum_core::math::{Float, Quat, Vec3};
use crate::aabb::Aabb;
use crate::collision_manifold::{find_contact_manifold, ContactPoint};
use crate::collision_mesh::{CollisionFace, CollisionSubShape};
use crate::epa::{gjk_epa, Contact, GjkResult};
use crate::gjk::ConvexShape;

const EPS: Float = 1e-6;

fn triangulated(verts: Vec<Vec3>, quads_or_tris: &[&[usize]]) -> CollisionSubShape {
    let mut faces = Vec::new();
    for poly in quads_or_tris {
        for i in 1..poly.len() - 1 {
            faces.push(CollisionFace { normal: Vec3::ZERO, verts: vec![poly[0], poly[i], poly[i + 1]] });
        }
    }
    CollisionSubShape::new(verts, faces)
}

/// Box with the given half extents, every quad split into two triangles.
fn cuboid(half: Vec3) -> CollisionSubShape {
    let verts = (0..8)
        .map(|i| Vec3::new(
            if i & 1 == 0 { -half.x } else { half.x },
            if i & 2 == 0 { -half.y } else { half.y },
            if i & 4 == 0 { -half.z } else { half.z },
        ))
        .collect();
    triangulated(verts, &[
        &[0, 2, 3, 1], &[4, 5, 7, 6], &[0, 1, 5, 4],
        &[2, 6, 7, 3], &[0, 4, 6, 2], &[1, 3, 7, 5],
    ])
}

/// Latitude/longitude sphere, a hull big enough to use hill climbing.
fn uv_sphere(rings: usize, segments: usize) -> CollisionSubShape {
    let mut verts = vec![Vec3::Y, -Vec3::Y];
    for r in 1..rings {
        let phi = std::f64::consts::PI as Float * r as Float / rings as Float;
        for s in 0..segments {
            let theta = 2.0 * std::f64::consts::PI as Float * s as Float / segments as Float;
            verts.push(Vec3::new(phi.sin() * theta.cos(), phi.cos(), phi.sin() * theta.sin()));
        }
    }
    let ring = |r: usize, s: usize| 2 + (r - 1) * segments + s % segments;
    let mut tris: Vec<Vec<usize>> = Vec::new();
    for s in 0..segments {
        tris.push(vec![0, ring(1, s + 1), ring(1, s)]);
        tris.push(vec![1, ring(rings - 1, s), ring(rings - 1, s + 1)]);
        for r in 1..rings - 1 {
            tris.push(vec![ring(r, s), ring(r, s + 1), ring(r + 1, s + 1), ring(r + 1, s)]);
        }
    }
    let refs: Vec<&[usize]> = tris.iter().map(|t| t.as_slice()).collect();
    triangulated(verts, &refs)
}

fn collide(a: &CollisionSubShape, rot_a: Quat, pos_a: Vec3, b: &CollisionSubShape, rot_b: Quat, pos_b: Vec3)
    -> Option<(Contact, Vec<ContactPoint>)> {
    let ca = ConvexShape::from_sub_shape(a, rot_a, pos_a);
    let cb = ConvexShape::from_sub_shape(b, rot_b, pos_b);
    match gjk_epa(&ca, &cb) {
        GjkResult::Separated => None,
        GjkResult::Intersecting(c) => {
            let manifold = find_contact_manifold(a, rot_a, pos_a, b, rot_b, pos_b, &c);
            Some((c, manifold))
        }
    }
}

#[test]
fn coplanar_triangles_merge_into_outward_quads() {
    let cube = cuboid(Vec3::ONE);
    assert_eq!(cube.faces.len(), 6);
    for face in &cube.faces {
        assert_eq!(face.verts.len(), 4);
        let center = face.verts.iter().map(|&i| cube.verts[i]).sum::<Vec3>() / 4.0;
        assert!((face.normal - center).length() < EPS, "normal {:?} for face at {:?}", face.normal, center);
        // Counter-clockwise around the outward normal.
        let v: Vec<Vec3> = face.verts.iter().map(|&i| cube.verts[i]).collect();
        assert!((v[1] - v[0]).cross(v[2] - v[0]).dot(face.normal) > 0.0);
    }
}

#[test]
fn rotated_aabb_matches_transformed_points() {
    let shape = cuboid(Vec3::new(3.0, 0.5, 1.0));
    let rot = Quat::from_axis_angle(Vec3::new(1.0, 2.0, 0.5).normalize(), 0.9);
    let pos = Vec3::new(4.0, -2.0, 7.0);
    let expected = Aabb::from_shapes(&shape.verts.iter().map(|&v| rot * v + pos).collect::<Vec<_>>());
    let actual = shape.aabb.transformed(rot, pos);
    assert!((expected.min - actual.min).length() < EPS && (expected.max - actual.max).length() < EPS,
            "expected {expected:?}, got {actual:?}");
}

#[test]
fn hill_climbing_support_matches_brute_force() {
    let sphere = uv_sphere(12, 24);
    assert!(!sphere.neighbors.is_empty());
    let rot = Quat::from_axis_angle(Vec3::new(0.3, 1.0, -0.2).normalize(), 2.0);
    let climbing = ConvexShape::from_sub_shape(&sphere, rot, Vec3::ONE);
    let scanning = ConvexShape::new(&sphere.verts, rot, Vec3::ONE);
    for k in 0..500 {
        let t = k as Float * 0.37;
        let dir = Vec3::new(t.sin() * (t * 1.3).cos(), (t * 0.7).cos(), (t * 1.9).sin());
        let a = climbing.support(dir).dot(dir);
        let b = scanning.support(dir).dot(dir);
        assert!((a - b).abs() < EPS, "direction {dir:?}: climbed {a}, best {b}");
    }
}

#[test]
fn separated_boxes_do_not_collide() {
    let cube = cuboid(Vec3::ONE);
    assert!(collide(&cube, Quat::IDENTITY, Vec3::ZERO, &cube, Quat::IDENTITY, Vec3::new(0.0, 2.05, 0.0)).is_none());
    assert!(collide(&cube, Quat::IDENTITY, Vec3::ZERO, &cube, Quat::IDENTITY, Vec3::new(1.5, 1.5, 1.5) * 1.4).is_none());
}

#[test]
fn stacked_boxes_give_four_point_face_contact() {
    let cube = cuboid(Vec3::ONE);
    let (contact, manifold) =
        collide(&cube, Quat::IDENTITY, Vec3::ZERO, &cube, Quat::IDENTITY, Vec3::new(0.3, 1.9, -0.2)).unwrap();
    assert!((contact.normal - Vec3::Y).length() < 1e-4, "normal {:?}", contact.normal);
    assert!((contact.depth - 0.1).abs() < 1e-4, "depth {}", contact.depth);
    assert_eq!(manifold.len(), 4);
    for cp in &manifold {
        assert!((cp.depth - 0.1).abs() < 1e-4);
        assert!((cp.position.y - 0.95).abs() < 1e-4);
        // Inside the overlap of the two top/bottom faces.
        assert!(cp.position.x >= -0.7 - EPS && cp.position.x <= 1.0 + EPS);
        assert!(cp.position.z >= -1.0 - EPS && cp.position.z <= 0.8 + EPS);
    }
}

#[test]
fn twisted_box_contact_is_reduced_to_four_spread_points() {
    let cube = cuboid(Vec3::ONE);
    let twist = Quat::from_rotation_y(0.785);
    let (_, manifold) = collide(&cube, Quat::IDENTITY, Vec3::ZERO, &cube, twist, Vec3::new(0.0, 1.9, 0.0)).unwrap();
    assert_eq!(manifold.len(), 4);
    // The points should surround the centre of the overlap rather than cluster.
    let center = manifold.iter().map(|c| c.position).sum::<Vec3>() / 4.0;
    assert!(Vec3::new(center.x, 0.0, center.z).length() < 0.3, "center {center:?}");
}

#[test]
fn box_edge_on_box_face_gives_edge_contact() {
    let cube = cuboid(Vec3::ONE);
    // Rest B on one of its edges, sunk 0.1 into A's top face.
    let rot = Quat::from_rotation_z(std::f64::consts::FRAC_PI_4 as Float);
    let pos = Vec3::new(0.0, 1.0 + (2.0 as Float).sqrt() - 0.1, 0.0);
    let (contact, manifold) = collide(&cube, Quat::IDENTITY, Vec3::ZERO, &cube, rot, pos).unwrap();
    assert!((contact.normal - Vec3::Y).length() < 1e-4, "normal {:?}", contact.normal);
    assert!((contact.depth - 0.1).abs() < 1e-4, "depth {}", contact.depth);
    assert_eq!(manifold.len(), 2, "{manifold:?}");
    for cp in &manifold {
        assert!((cp.depth - 0.1).abs() < 1e-4);
        assert!(cp.position.x.abs() < 1e-4);
        assert!((cp.position.z.abs() - 1.0).abs() < 1e-4);
    }
}

#[test]
fn normal_points_from_a_to_b() {
    let cube = cuboid(Vec3::ONE);
    let sphere = uv_sphere(8, 16);
    for dir in [Vec3::X, -Vec3::Y, Vec3::new(0.0, 0.0, 1.0)] {
        let (contact, manifold) =
            collide(&cube, Quat::IDENTITY, Vec3::ZERO, &sphere, Quat::IDENTITY, dir * 1.8).unwrap();
        assert!(contact.normal.dot(dir) > 0.99, "dir {dir:?}, normal {:?}", contact.normal);
        assert!(!manifold.is_empty());
    }
}
