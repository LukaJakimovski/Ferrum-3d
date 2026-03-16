use ferrum_core::math::{Float, Mat3, Quat, Vec3};
use crate::mass_properties::comp_volume_integrals;
use crate::physics_vertex::Polyhedron;
use crate::rigidbody::RigidBodySet;

#[derive(Default)]
#[derive(Clone)]
pub struct RigidBody {
    pub(crate) position: Vec3,    // hot - read every frame
    pub(crate) velocity: Vec3,    // hot - read every frame
    pub(crate) orientation: Quat,    // hot - read every frame
    pub(crate) force: Vec3,    // hot - written every frame
    pub(crate) torque: Vec3,
    pub(crate) mesh: usize,
    pub(crate) inv_mass: Float,     // warm
    pub(crate) inertia: Mat3,    // warm
    pub(crate) restitution: Float,     // cold - only on collision
    pub(crate) _is_sleeping: bool,    // cold
    pub(crate) index: usize,
    pub(crate) omega: Vec3,
    pub(crate) mass: Float,
    pub inv_inertia: Mat3
}

impl RigidBody {

    pub fn builder() -> Self {
        let mut body: RigidBody = Default::default();
        body.position = Vec3::ZERO;
        body.velocity = Vec3::ZERO;
        body.orientation = Quat::IDENTITY;
        body.force = Vec3::ZERO;
        body.inv_mass = 0.0;
        body.inertia = Mat3::ZERO;
        body.inv_inertia = Mat3::ZERO;
        body.torque = Vec3::ZERO;
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
    fn torque(mut self, torque: Vec3) -> Self {
        self.torque = torque;
        self
    }
    #[allow(unused)]
    fn inv_mass(mut self, inv_mass: Float) -> Self {
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
    pub fn inertia(mut self, polyhedron: &Polyhedron) -> Self {
        #[allow(non_snake_case)]
        let (T0, T1, T2, TP) = comp_volume_integrals(polyhedron);
        let r = T1 / T0;
        let density = self.mass / T0;

        #[allow(non_snake_case)]
        let mut J = self.inertia.to_cols_array_2d();
        let mass = self.mass;


        /* compute inertia tensor */
        J[0][0] = density * (T2[1] + T2[2]);
        J[1][1] = density * (T2[2] + T2[0]);
        J[2][2] = density * (T2[0] + T2[1]);
        J[1][0] = - density * TP[0];
        J[0][1] = J[1][0];
        J[2][1] = - density * TP[1];
        J[1][2] = J[2][1];
        J[0][2] = - density * TP[2];
        J[2][0] = J[0][2];

        /* translate inertia tensor to center of mass */
        J[0][0] -= mass * (r[1]*r[1] + r[2]*r[2]);
        J[1][1] -= mass * (r[2]*r[2] + r[0]*r[0]);
        J[2][2] -= mass * (r[0]*r[0] + r[1]*r[1]);
        J[1][0] += mass * r[0] * r[1];
        J[2][1] += mass * r[1] * r[2];
        J[0][2] += mass * r[2] * r[0];
        J[0][1] = J[1][0];
        J[1][2] = J[2][1];
        J[2][0] = J[0][2];

        self.inertia = Mat3::from_cols_array_2d(&J);
        self.inv_inertia = self.inertia.inverse();
        self
    }
    fn set_inertia(mut self, inertia: Mat3) -> Self {
        self.inertia = inertia;
        self.inv_inertia = self.inertia.inverse();
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
            .torque(set.torques[i])
            .omega(set.omega[i])
            .orientation(set.orientations[i])
            .velocity(set.velocities[i])
            .set_inertia(set.inertia[i])
            .position(set.positions[i])
            .restitution(set.restitution[i])
            .sleeping(set.is_sleeping[i])
    }
}