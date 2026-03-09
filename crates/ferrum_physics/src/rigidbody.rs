use ferrum_core::math::{Vec3, Quat, Float};
pub use crate::rigidbodybuilder::RigidBody;

#[derive(Clone)]
pub struct RigidBodySet {
    pub(crate) positions:    Vec<Vec3>,   // hot  - read every frame
    pub(crate) velocities:   Vec<Vec3>,   // hot  - read every frame
    pub(crate) _omega:        Vec<Vec3>,
    pub(crate) orientations: Vec<Quat>,   // hot  - read every frame
    pub(crate) mesh:         Vec<usize>,  // hot  - read every frame
    pub(crate) _forces:      Vec<Vec3>,   // hot  - written every frame
    pub(crate) inv_mass:     Vec<Float>,  // warm - read once every frame
    pub(crate) _inertia:     Vec<Vec3>,   // warm - read once every frame
    pub(crate) _restitution: Vec<Float>,  // cold - only on collision
    pub(crate) _is_sleeping: Vec<bool>,   // cold
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

    pub fn get_index(&self, body_id: usize) -> usize { self.index[body_id] }
    
    pub fn new(num_bodies: usize) -> RigidBodySet {
        RigidBodySet {
            positions:      vec![Vec3::ZERO; num_bodies],
            velocities:     vec![Vec3::ZERO; num_bodies],
            orientations:   vec![Quat::IDENTITY; num_bodies],
            _forces:        vec![Vec3::ZERO; num_bodies],
            inv_mass:       vec![0.0; num_bodies],
            _inertia:       vec![Vec3::ZERO; num_bodies],
            _restitution:   vec![0.0; num_bodies],
            _is_sleeping:   vec![false; num_bodies],
            mesh:           vec![0; num_bodies],
            index:          vec![0; num_bodies],
            _omega:          vec![Vec3::ZERO; num_bodies],
        }
    }
    
    pub fn len(&self) -> usize {
        self.positions.len()
    }
    pub fn add_default(&mut self) {
        self.positions.push(Vec3::ZERO);
        self.velocities.push(Vec3::ZERO);
        self.orientations.push(Quat::IDENTITY);
        self._forces.push(Vec3::ZERO);
        self.inv_mass.push(0.0);
        self._inertia.push(Vec3::ZERO);
        self._restitution.push(0.0);
        self._is_sleeping.push(false);
        self.mesh.push(0);
        self.index.push(0);
        self._omega.push(Vec3::ZERO);
    }
    
    pub fn add_body(&mut self, builder: RigidBody){
        self.positions.push(builder.position);
        self.orientations.push(builder.orientation);
        self.velocities.push(builder.velocity);
        self._forces.push(builder._force);
        self.inv_mass.push(builder._inv_mass);
        self._inertia.push(builder._inertia);
        self._restitution.push(builder._restitution);
        self._is_sleeping.push(false);
        self.mesh.push(builder.mesh);
        self.index.push(builder.index);
        self._omega.push(builder.omega);
    }
    
}