use ferrum_core::constants::Mesh;
use ferrum_core::math::Vec3;
use crate::rigidbody::RigidBody;
use crate::update::Physics;

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

        self.energy.update_energy(&self.rigidbodies);
        self.energy.start_energy = self.energy.total_energy;
    }

    pub fn two_objects(&mut self){
        let body1 = RigidBody::builder()
            .position(Vec3::new(0.0, 0.0, 0.0))
            .velocity(Vec3::new(0.0, 0.0, 0.0))
            .omega(Vec3::X * 1.0)
            .mass(1.0)
            .mesh(Mesh::Icosahedron as usize)
            .inertia(&self.polyhedrons[Mesh::Icosahedron as usize]);


        let body2 = RigidBody::builder()
            .position(Vec3::new(10.0, 0.0, 0.0))
            .velocity(Vec3::new(0.0, 0.0, 0.0))
            .omega(Vec3::X * 1.0)
            .mass(1.0)
            .mesh(Mesh::Cylinder as usize)
            .inertia(&self.polyhedrons[Mesh::Cylinder as usize]);

        self.rigidbodies.add_body(body1);
        self.rigidbodies.add_body(body2);

        self.energy.update_energy(&self.rigidbodies);
        self.energy.start_energy = self.energy.total_energy;
    }
}