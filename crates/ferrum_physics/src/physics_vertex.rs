use ferrum_core::math::{Float, Vec3};

pub struct PhysicsVertex{
    pub position: Vec3,
    pub normal: Vec3,
    pub index: u32,
    pub w: Float,
}