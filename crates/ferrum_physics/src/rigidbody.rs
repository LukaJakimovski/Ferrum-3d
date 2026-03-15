use ferrum_core::math::{Vec3, Quat, Float, Mat3};
use crate::mass_properties::comp_volume_integrals;
use crate::physics_vertex::Polyhedron;
pub use crate::rigidbodybuilder::RigidBody;

#[derive(Clone)]
pub struct RigidBodySet {
    pub positions:    Vec<Vec3>,   // hot  - read every frame
    pub velocities:          Vec<Vec3>,   // hot  - read every frame
    pub omega:        Vec<Vec3>,
    pub(crate) orientations: Vec<Quat>,   // hot  - read every frame
    pub(crate) mesh:         Vec<usize>,  // hot  - read every frame
    pub(crate) forces:       Vec<Vec3>,   // hot  - written every frame
    pub(crate) torques:      Vec<Vec3>,
    pub(crate) inv_mass:     Vec<Float>,  // warm - read once every frame
    pub(crate) mass:         Vec<Float>,
    pub(crate) inertia:      Vec<Mat3>,   // warm - read once every frame
    pub(crate) inv_inertia:  Vec<Mat3>,
    pub(crate) restitution:  Vec<Float>,  // cold - only on collision
    pub(crate) is_sleeping:  Vec<bool>,   // cold
    pub(crate) index:        Vec<usize>,
}

impl RigidBodySet {
    pub fn translate(&mut self, translation: Vec3, body_id: usize) {
        self.positions[body_id] += translation;
    }

    pub fn move_to(&mut self, position: Vec3, body_id: usize) {
        self.positions[body_id].x = position.x;
        self.positions[body_id].y = position.y;
        self.positions[body_id].z = position.z;
    }

    pub fn rotate(&mut self, rotation: Quat, body_id: usize) {
        self.orientations[body_id] = self.orientations[body_id] * rotation;
    }

    pub fn get_position(&self, body_id: usize) -> Vec3 {
        self.positions[body_id]
    }
    pub fn get_velocity(&self, body_id: usize) -> Vec3 {
        self.velocities[body_id]
    }
    pub fn get_orientation(&self, body_id: usize) -> Quat {
        self.orientations[body_id]
    }
    pub fn get_inv_mass(&self, body_id: usize) -> Float { self.inv_mass[body_id] }
    pub fn get_mesh(&self, body_id: usize) -> usize { self.mesh[body_id] }
    pub fn get_omega(&self, body_id: usize) -> Vec3 { self.omega[body_id] }
    pub fn get_forces(&self, body_id: usize) -> Vec3 { self.forces[body_id] }
    pub fn get_torques(&self, body_id: usize) -> Vec3 { self.torques[body_id] }
    pub fn get_mass(&self, body_id: usize) -> Float { self.mass[body_id] }
    pub fn get_inertia(&self, body_id: usize) -> Mat3 { self.inertia[body_id] }
    pub fn get_inv_inertia(&self, body_id: usize) -> Mat3 { self.inv_inertia[body_id] }
    pub fn get_restitution(&self, body_id: usize) -> Float { self.restitution[body_id] }
    pub fn get_index(&self, body_id: usize) -> usize { self.index[body_id] }
    

    pub fn comp_inertia_tensor(&mut self, body_id: usize, polyhedron: &Polyhedron){
        #[allow(non_snake_case)]
        let (T0, T1, T2, TP) = comp_volume_integrals(polyhedron);
        let r = T1 / T0;
        let density = self.mass[body_id] / T0;

        #[allow(non_snake_case)]
        let mut J = self.inertia[body_id].to_cols_array_2d();
        let mass = self.mass[body_id];
        
        
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

        self.inertia[body_id] = Mat3::from_cols_array_2d(&J);
        self.inv_inertia[body_id] = self.inertia[body_id].inverse();
    }

    pub fn new(num_bodies: usize) -> RigidBodySet {
        RigidBodySet {
            positions:      vec![Vec3::ZERO; num_bodies],
            velocities:     vec![Vec3::ZERO; num_bodies],
            orientations:   vec![Quat::IDENTITY; num_bodies],
            forces:         vec![Vec3::ZERO; num_bodies],
            torques:        vec![Vec3::ZERO; num_bodies],
            inv_mass:       vec![0.0; num_bodies],
            mass:           vec![0.0; num_bodies],
            inv_inertia:    vec![Mat3::ZERO; num_bodies],
            inertia:        vec![Mat3::ZERO; num_bodies],
            restitution:    vec![0.0; num_bodies],
            is_sleeping:    vec![false; num_bodies],
            mesh:           vec![0; num_bodies],
            index:          vec![0; num_bodies],
            omega:          vec![Vec3::ZERO; num_bodies],
        }
    }
    
    pub fn len(&self) -> usize {
        self.positions.len()
    }
    pub fn add_default(&mut self) {
        self.positions.push(Vec3::ZERO);
        self.velocities.push(Vec3::ZERO);
        self.orientations.push(Quat::IDENTITY);
        self.forces.push(Vec3::ZERO);
        self.torques.push(Vec3::ZERO);
        self.inv_mass.push(0.0);
        self.inertia.push(Mat3::ZERO);
        self.inv_inertia.push(Mat3::ZERO);
        self.restitution.push(0.0);
        self.is_sleeping.push(false);
        self.mesh.push(0);
        self.index.push(0);
        self.omega.push(Vec3::ZERO);
    }
    
    pub fn add_body(&mut self, builder: RigidBody){
        self.positions.push(builder.position);
        self.orientations.push(builder.orientation);
        self.velocities.push(builder.velocity);
        self.forces.push(builder.force);
        self.torques.push(builder.torque);
        self.inv_mass.push(builder.inv_mass);
        self.mass.push(builder.mass);
        self.inertia.push(builder.inertia);
        self.inv_inertia.push(builder.inv_inertia);
        self.restitution.push(builder.restitution);
        self.is_sleeping.push(false);
        self.mesh.push(builder.mesh);
        self.index.push(builder.index);
        self.omega.push(builder.omega);
    }
    
}