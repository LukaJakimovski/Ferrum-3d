use ferrum_core::math::{Float, Quat, Vec3};

/// Axis-aligned bounding box for broadphase culling.
#[derive(Clone, Copy)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    /// Build an AABB from a set of convex sub-shapes, already in local space.
    pub fn from_shapes(shape: &Vec<Vec3>) -> Self {
        let mut min = Vec3::splat(Float::INFINITY);
        let mut max = Vec3::splat(Float::NEG_INFINITY);
        for v in shape {
            min = min.min(*v);
            max = max.max(*v);
        }
        Self { min, max }
    }

    pub fn transformed(&self, rot: Quat, translation: Vec3) -> Self {
        let center = rot * ((self.min + self.max) * 0.5) + translation;
        let half = (self.max - self.min) * 0.5;
        let world_half = Vec3::new(
            (rot * Vec3::X).abs().dot(half),
            (rot * Vec3::Y).abs().dot(half),
            (rot * Vec3::Z).abs().dot(half),
        );
        Self {
            min: center - world_half,
            max: center + world_half,
        }
    }

    #[inline]
    pub fn intersects(&self, other: &Aabb) -> bool {
        self.min.x <= other.max.x && self.max.x >= other.min.x &&
            self.min.y <= other.max.y && self.max.y >= other.min.y &&
            self.min.z <= other.max.z && self.max.z >= other.min.z
    }
}