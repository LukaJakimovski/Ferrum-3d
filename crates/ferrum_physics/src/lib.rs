use ferrum_core::math::{Float, Vec3};
use crate::energy::Energy;
use crate::polyhedron::Polyhedron;
use ferrum_collision::collision_mesh::CollisionMesh;
use crate::rigidbody_set::RigidBodySet;

pub mod rigidbody_set;
pub mod update;
mod rigidbody;
pub mod mass_properties;
pub mod polyhedron;
pub mod energy;
mod init;
pub mod gravity;
pub mod collision;
pub mod integration;

pub struct Physics {
    pub rigidbodies: RigidBodySet,
    pub parameters: Params,
    pub polyhedrons: Vec<Polyhedron>,
    pub collision_meshes: Vec<CollisionMesh>,
    pub energy: Energy
}

pub struct Params {
    pub gravity_mode: GravityMode,
    pub gravity_constant: Float,
    pub uniform_gravity: Vec3,

    pub delta_time_mode: DeltaTimeMode,
    pub multiplier: Float,
    pub delta_time: Float,

    pub substeps: usize,
    pub running: bool,
}


impl Default for Params {
    fn default() -> Self {
        Self {
            gravity_mode: Default::default(),
            gravity_constant: 1.0,
            uniform_gravity: Vec3::new(0.0, -9.80665, 0.0),
            delta_time_mode: Default::default(),
            multiplier: 1.0,
            delta_time: 0.001,
            substeps: 1,
            running: true,
        }
    }
}

#[derive(Default, Copy, Clone, Debug, PartialEq)]
pub enum GravityMode {
    #[default]
    Off = 0,
    Uniform = 1,
    Newtonian = 2,
}

#[derive(Default, Copy, Clone, Debug, PartialEq)]
pub enum DeltaTimeMode {
    #[default]
    RealTime = 0,
    Multiplier = 1,
    Constant = 2,
}