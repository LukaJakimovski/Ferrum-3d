use cgmath::{Matrix3, Quaternion, Vector3};

pub struct RigidBodySet {
    positions:     Vec<Vector3<f64>>,    // hot - read every frame
    velocities:    Vec<Vector3<f64>>,    // hot - read every frame
    orientations:  Vec<Quaternion<f64>>,    // hot - read every frame
    forces:        Vec<Vector3<f64>>,    // hot - written every frame
    inv_mass:      Vec<f64>,     // warm
    inertia:       Vec<Matrix3<f64>>,    // warm
    restitution:   Vec<f64>,     // cold - only on collision
    is_sleeping:   Vec<bool>,    // cold
}

impl RigidBodySet {
    fn translate(&mut self, translation: Vector3<f64>, body_id: usize) {
        self.positions[body_id] += translation;
    }

    fn rotate(&mut self, rotation: Quaternion<f64>, body_id: usize) {
        self.orientations[body_id] = self.orientations[body_id] * rotation;
    }
}