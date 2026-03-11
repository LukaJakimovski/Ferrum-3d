use ferrum_core::math::{Float, Vec3};

#[derive(Copy, Clone, Default)]
pub struct Face{
    pub vert: [Vec3; 3],
    pub norm: Vec3,
    pub w: Float,
}