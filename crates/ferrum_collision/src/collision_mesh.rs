use ferrum_core::math::Vec3;

#[derive(Clone, Default, PartialEq)]
#[derive(Debug)]
pub struct CollisionFace {
    pub normal: Vec3,
    pub verts: Vec<usize>, // indices into CollisionSubShape::verts
}

#[derive(Default, Clone, PartialEq)]
pub struct CollisionSubShape {
    pub verts: Vec<Vec3>,
    pub faces: Vec<CollisionFace>,
}

pub struct CollisionMesh {
    pub shapes: Vec<CollisionSubShape>,
}