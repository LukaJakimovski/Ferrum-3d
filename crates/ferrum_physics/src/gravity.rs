use ferrum_core::math::{Float, Vec3};
use crate::Physics;
use crate::rigidbody_set::RigidBodySet;

impl Physics {
    #[inline]
    pub fn newtonian_gravity(dt_offset: Float, my_pos: Vec3, _my_vel: Vec3, g_const: Float, snap_positions: &Vec<Vec3>, body_id: usize, snap_velocities: &Vec<Vec3>, snap_mass: &Vec<Float>, r: &RigidBodySet) -> Vec3{
        let mut accel = Vec3::ZERO;
        for j in 0..snap_positions.len() {
            if body_id == j { continue; }

            let other_pos_at_t = snap_positions[j] + snap_velocities[j] * dt_offset;

            let diff = other_pos_at_t - my_pos;
            let dist_sq = diff.length_squared() + 1e-14;
            accel += g_const * diff * (snap_mass[j] * r.mass[body_id] * r.mass[body_id]) / (dist_sq * dist_sq.sqrt());
        }
        accel
    }
}
