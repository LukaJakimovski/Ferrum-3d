use glam::{Quat, Vec3};
use crate::instance::Instance;

struct Arrow {
    transform: Instance,
    index: usize,
}

impl Arrow {
    pub fn rotation_from_vec3(&mut self, vec: Vec3){
        let norm = vec.normalize();
        let rotation = if norm.dot(Vec3::NEG_X).abs() > 0.9999 {
            if norm.dot(Vec3::NEG_X) > 0.0 {
                // Already pointing -X, no rotation needed
                Quat::IDENTITY
            } else {
                // Pointing +X, rotate 180° around Y
                Quat::from_axis_angle(Vec3::Y, std::f32::consts::PI)
            }
        } else {
            let axis = norm.cross(Vec3::NEG_X).normalize();
            let angle = -norm.dot(Vec3::NEG_X).acos();
            Quat::from_axis_angle(axis, angle)
        };

        self.transform.rotation = rotation;
    }
}