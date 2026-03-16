use glam::{Quat, Vec3};
use ferrum_core::math;
use ferrum_core::math::ToGlamVec3;
use crate::instance::Instance;

pub struct Arrow {
    pub(crate) transform: Option<*mut Instance>,
    pub(crate) vec: Option<*const math::Vec3>,
}

impl Arrow {
    pub fn update_orientation(&mut self) {
        if self.transform.is_none() || self.vec.is_none() {
            println!("Not pointing to object");
            return;
        }

        let vec = unsafe { &*self.vec.unwrap() }.to_glam_vec3();
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

        let transform = unsafe { &mut *self.transform.unwrap() };
        transform.rotation = rotation;
    }
}