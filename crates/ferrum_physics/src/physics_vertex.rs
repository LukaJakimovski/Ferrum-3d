use ferrum_core::math::{Float, Vec3};

#[derive(Copy, Clone, Default)]
pub struct Face{
    pub vert: [usize; 3],
    pub w: Float,
    pub norm: Vec3
}

#[derive(Default)]
pub struct Polyhedron{
    pub faces: Vec<Face>,
    pub vert: Vec<Vec3>,
}