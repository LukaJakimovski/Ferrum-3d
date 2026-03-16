use std::f32::consts::PI;
use glam::{Quat, Vec3};
use crate::instance::Instance;
use crate::State;

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

        self.timer.runtime += dt;
        self.timer.fps = 1.0 / dt;
        self.timer.dt = dt;
        self.physics.physics_update(&mut dt);
        self.timer.sim_time += dt;
        self.update_instances();
        for arrow in self.arrows.iter_mut() {
            arrow.update_orientation();
        }


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