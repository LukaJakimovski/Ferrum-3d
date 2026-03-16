use std::f32::consts::PI;
use glam::{Quat, Vec3};
use ferrum_core::math;
use crate::instance::Instance;
use crate::{Mesh, State};

impl State{
    pub fn update(&mut self, mut dt: f64) {
        self.camera_controller.update_camera(&mut self.camera, dt);
        self.camera_uniform
            .update_view_proj(&self.camera, &self.projection);
        self.queue.write_buffer(
            &self.camera_buffer,
            0,
            bytemuck::cast_slice(&[self.camera_uniform]),
        );

        // Update the light
        let old_position: Vec3 = self.light_uniform.position.into();
        self.light_uniform.position = (Quat::from_axis_angle(
            (0.0, 1.0, 0.0).into(),
            (PI * dt as f32).to_radians(),
        ) * old_position)
            .into();
        self.queue.write_buffer(
            &self.light_buffer,
            0,
            bytemuck::cast_slice(&[self.light_uniform]),
        );

        for arrow in 0..self.instances[Mesh::Arrow as usize].len() {
            let vec = &self.physics.rigidbodies.velocities[arrow];
            let norm = vec.normalize();

            let rotation = if norm.dot(math::Vec3::NEG_X).abs() > 0.9999 {
                if norm.dot(math::Vec3::NEG_X) > 0.0 {
                    // Already pointing -X, no rotation needed
                    math::Quat::IDENTITY
                } else {
                    // Pointing +X, rotate 180° around Y
                    math::Quat::from_axis_angle(math::Vec3::Y, std::f64::consts::PI)
                }
            } else {
                let axis = norm.cross(math::Vec3::NEG_X).normalize();
                let angle = -norm.dot(math::Vec3::NEG_X).acos();
                math::Quat::from_axis_angle(axis, angle)
            };

            self.instances[Mesh::Arrow as usize][arrow].rotation = rotation.as_quat();
        }

        self.timer.runtime += dt;
        self.timer.fps = 1.0 / dt;
        self.timer.dt = dt;
        self.physics.physics_update(&mut dt);
        self.timer.sim_time += dt;
        self.update_instances();

        // Rebuild the raw instance data and write to the buffer
        for (mesh, instance) in self.instances.iter().enumerate() {
            let instance_data = instance.iter().map(Instance::to_raw).collect::<Vec<_>>();
            self.queue.write_buffer(
                &self.instance_buffers[mesh],
                0,
                bytemuck::cast_slice(&instance_data),
            );
        }

    }
}