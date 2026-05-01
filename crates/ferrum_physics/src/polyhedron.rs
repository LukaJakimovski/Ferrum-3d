use ferrum_core::math::{Float, Vec3};

#[derive(Clone, Default, PartialEq)]
pub struct Face{
    pub norm: Vec3,
    pub w: Float,
    pub verts: Vec<usize>,
}

#[derive(Default, Clone, PartialEq)]
pub struct Polyhedron{
    pub faces: Vec<Face>,
    pub vert: Vec<Vec3>,
}