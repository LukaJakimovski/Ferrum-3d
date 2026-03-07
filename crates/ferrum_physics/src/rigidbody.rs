use ferrum_core::math::{Vec3, Quat, Float};

pub struct RigidBodySet {
    positions:     Vec<Vec3>,    // hot - read every frame
    velocities:    Vec<Vec3>,    // hot - read every frame
    orientations:  Vec<Quat>,    // hot - read every frame
    _forces:        Vec<Vec3>,    // hot - written every frame
    _inv_mass:      Vec<Float>,     // warm
    _inertia:       Vec<Vec3>,    // warm
    _restitution:   Vec<Float>,     // cold - only on collision
    _is_sleeping:   Vec<bool>,    // cold
}

impl RigidBodySet {
    pub fn translate(&mut self, translation: Vec3, body_id: usize) {
        self.positions[body_id] += translation;
    }

    pub fn rotate(&mut self, rotation: Quat, body_id: usize) {
        self.orientations[body_id] = self.orientations[body_id] * rotation;
    }

    pub fn get_position(&self, body_id: usize) -> Vec3 {
        self.positions[body_id]
    }
    pub fn get_velocity(&self, body_id: usize) -> Vec3 {
        self.velocities[body_id]
    }
    pub fn get_orientation(&self, body_id: usize) -> Quat {
        self.orientations[body_id]
    }
    pub fn new(num_bodies: usize) -> RigidBodySet {
        RigidBodySet {
            positions:     vec![Vec3::ZERO; num_bodies],
            velocities:    vec![Vec3::ZERO; num_bodies],
            orientations:  vec![Quat::IDENTITY; num_bodies],
            _forces:        vec![Vec3::ZERO; num_bodies],
            _inv_mass:      vec![0.0; num_bodies],
            _inertia:       vec![Vec3::ZERO; num_bodies],
            _restitution:   vec![0.0; num_bodies],
            _is_sleeping:   vec![false; num_bodies],
        }
    }
}