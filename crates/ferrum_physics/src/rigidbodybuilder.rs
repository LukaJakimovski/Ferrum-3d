use ferrum_core::math::{Float, Quat, Vec3};
use crate::rigidbody::RigidBodySet;

#[derive(Default)]
#[derive(Clone)]
pub struct RigidBody {
    pub(crate) position: Vec3,    // hot - read every frame
    pub(crate) velocity: Vec3,    // hot - read every frame
    pub(crate) orientation: Quat,    // hot - read every frame
    pub(crate) force: Vec3,    // hot - written every frame
    pub(crate) mesh: usize,
    pub(crate) inv_mass: Float,     // warm
    pub(crate) inertia: Vec3,    // warm
    pub(crate) restitution: Float,     // cold - only on collision
    pub(crate) _is_sleeping: bool,    // cold
    pub(crate) index: usize,
    pub(crate) omega: Vec3,
    pub(crate) mass: Float,
}

impl RigidBody {

    pub fn builder() -> Self {
        let mut body: RigidBody = Default::default();
        body.position = Vec3::ZERO;
        body.velocity = Vec3::ZERO;
        body.orientation = Quat::IDENTITY;
        body.force = Vec3::ZERO;
        body.inv_mass = 0.0;
        body.inertia = Vec3::ZERO;
        body.restitution = 0.0;
        body._is_sleeping = false;
        body.mesh = 0;
        body.index = 0;
        body.mass = 0.0;
        body
    }
    #[allow(unused)]
    pub fn position(mut self, position: Vec3) -> Self {
        self.position = position;
        self
    }
    #[allow(unused)]
    pub fn velocity(mut self, velocity: Vec3) -> Self {
        self.velocity = velocity;
        self
    }
    #[allow(unused)]
    pub fn orientation(mut self, orientation: Quat) -> Self {
        self.orientation = orientation;
        self
    }
    #[allow(unused)]
    pub fn force(mut self, force: Vec3) -> Self {
        self.force = force;
        self
    }
    #[allow(unused)]
    pub fn inv_mass(mut self, inv_mass: Float) -> Self {
        self.inv_mass = inv_mass;
        self.mass = 1.0 / inv_mass;
        self
    }
    
    pub fn mass(mut self, mass: Float) -> Self {
        self.mass = mass;
        self.inv_mass = 1.0 / mass;
        self
    }
    #[allow(unused)]
    pub fn inertia(mut self, inertia: Vec3) -> Self {
        self.inertia = inertia;
        self
    }
    #[allow(unused)]
    pub fn restitution(mut self, restitution: Float) -> Self {
        self.restitution = restitution;
        self
    }

    pub fn mesh(mut self, mesh: usize) -> Self {
        self.mesh = mesh;
        self
    }
    
    pub fn index(mut self, index: usize) -> Self {
        self.index = index;
        self
    }
    
    pub fn omega(mut self, omega: Vec3) -> Self {
        self.omega = omega;
        self
    }

    pub fn sleeping(mut self, sleeping: bool) -> Self {
        self._is_sleeping = sleeping;
        self
    }

    pub fn from_set(set: RigidBodySet, i: usize) -> Self {
        let body: RigidBody = Default::default();
        body.inv_mass(set.get_inv_mass(i))
            .mesh(set.get_mesh(i))
            .index(set.get_index(i))
            .force(set.forces[i])
            .omega(set.omega[i])
            .orientation(set.orientations[i])
            .velocity(set.velocities[i])
            .inertia(set.inertia[i])
            .position(set.positions[i])
            .restitution(set.restitution[i])
            .sleeping(set.is_sleeping[i])
    }
}