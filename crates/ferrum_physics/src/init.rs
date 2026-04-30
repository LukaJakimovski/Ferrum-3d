use ferrum_core::constants::Mesh;
use ferrum_core::math::{Quat, Vec3};
use crate::rigidbody_set::RigidBody;
use crate::{GravityMode, Physics};

impl Physics {
    pub fn figure_eight(&mut self){
        let body1 = RigidBody::builder()
            .position(Vec3::new(-0.97000436, 0.24308753, 0.0))
            .velocity(Vec3::new(0.46620368, 0.43236573, 0.0))
            .mass(2.0)
            .omega(Vec3::X * 1.0)
            .mesh(Mesh::Cow as usize)
            .inertia(&self.polyhedrons[Mesh::Cow as usize]);

        let body2 = RigidBody::builder()
            .position(Vec3::new(0.97000436, -0.24308753, 0.0))
            .velocity(Vec3::new(0.46620368, 0.43236573, 0.0))
            .mass(2.0)
            .mesh(Mesh::Bunny as usize)
            .inertia(&self.polyhedrons[Mesh::Bunny as usize]);

        let body3 = RigidBody::builder()
            .position(Vec3::ZERO)
            .velocity(Vec3::new(-0.93240737, -0.86473146, 0.0))
            .mass(2.0)
            .mesh(Mesh::BunnyLowPoly as usize)
            .inertia(&self.polyhedrons[Mesh::BunnyLowPoly as usize]);

        self.rigidbodies.add_body(body1);
        self.rigidbodies.add_body(body2);
        self.rigidbodies.add_body(body3);

        self.parameters.gravity_constant = 0.25;
        self.parameters.gravity_mode = GravityMode::Newtonian;
        self.parameters.substeps = 100;

        self.energy.update_energy(&self.rigidbodies, &self.parameters);
        self.energy.start_energy = self.energy.total_energy;
    }

    pub fn two_objects(&mut self){
        let body1 = RigidBody::builder()
            .position(Vec3::new(0.0, 0.0, 0.0))
            .velocity(Vec3::new(0.0, 0.0, 0.0))
            .omega(Vec3::X * 1.0)
            .mass(1.0)
            .mesh(Mesh::Bunny as usize)
            .inertia(&self.polyhedrons[Mesh::Bunny as usize]);


        let body2 = RigidBody::builder()
            .position(Vec3::new(10.0, 0.0, 0.0))
            .velocity(Vec3::new(0.0, 0.0, 0.0))
            .omega(Vec3::X * 1.0)
            .mass(1.0)
            .mesh(Mesh::Monkey as usize)
            .inertia(&self.polyhedrons[Mesh::Monkey as usize]);

        self.rigidbodies.add_body(body1);
        self.rigidbodies.add_body(body2);

        self.energy.update_energy(&self.rigidbodies, &self.parameters);
        self.energy.start_energy = self.energy.total_energy;
    }


    pub fn flat_plane(&mut self){
        let body1 = RigidBody::builder()
            .position(Vec3::new(0.0, 0.0, 0.0))
            .velocity(Vec3::new(0.0, 0.0, 0.0))
            .omega(Vec3::X * 1.0)
            .mass(1.0)
            .restitution(0.3)
            .mesh(Mesh::Icosahedron as usize)
            .inertia(&self.polyhedrons[Mesh::Icosahedron as usize]);


        let body2 = RigidBody::builder()
            .position(Vec3::new(10.0, 0.0, 0.0))
            .velocity(Vec3::new(0.0, 0.0, 0.0))
            .omega(Vec3::X * 1.0)
            .mass(1.0)
            .restitution(0.3)
            .mesh(Mesh::Torus as usize)
            .inertia(&self.polyhedrons[Mesh::Torus as usize]);

        let body3 = RigidBody::builder()
            .position(Vec3::new(0.0, -10.0, 0.0))
            .velocity(Vec3::new(0.0, 0.0, 0.0))
            .mass(10000000.0)
            .restitution(0.3)
            .orientation(Quat::from_axis_angle(Vec3::X, 0.5))
            .mesh(Mesh::Plane as usize)
            .inertia(&self.polyhedrons[Mesh::Plane as usize]);

        self.rigidbodies.add_body(body1);
        self.rigidbodies.add_body(body2);
        self.rigidbodies.add_body(body3);

        self.parameters.uniform_gravity = Vec3::new(0.0,-9.81,0.0);
        self.parameters.gravity_mode = GravityMode::Uniform;
        self.parameters.substeps = 1;

        self.energy.update_energy(&self.rigidbodies, &self.parameters);
        self.energy.start_energy = self.energy.total_energy;
    }
}