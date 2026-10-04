use ferrum_core::math::{Float, Mat3, Quat, Vec3};

/// Axis-aligned bounding box for broadphase culling.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    /// Build an AABB from a set of points, already in local space.
    pub fn from_shapes(shape: &[Vec3]) -> Self {
        let mut min = Vec3::splat(Float::INFINITY);
        let mut max = Vec3::splat(Float::NEG_INFINITY);
        for v in shape {
            min = min.min(*v);
            max = max.max(*v);
        }
        Self { min, max }
    }

    pub fn union(&self, other: &Aabb) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    pub fn transformed(&self, rot: Quat, translation: Vec3) -> Self {
        let center = rot * ((self.min + self.max) * 0.5) + translation;
        let half = (self.max - self.min) * 0.5;
        // Each world axis extent is the sum of the rotated local half-extents
        // projected onto it, i.e. |R| * half (rows of R, not columns).
        let r = Mat3::from_quat(rot);
        let abs_r = Mat3::from_cols(r.x_axis.abs(), r.y_axis.abs(), r.z_axis.abs());
        let world_half = abs_r * half;
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
