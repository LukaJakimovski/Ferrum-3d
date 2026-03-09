use glam::{Mat3, Mat4, Quat, Vec3};
use ferrum_physics::rigidbody::RigidBody;
use crate::{model, State};
use wgpu::util::DeviceExt;
use ferrum_core::math::{ToFloat, ToF32};

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
#[allow(dead_code)]
pub struct InstanceRaw {
    model: [[f32; 4]; 4],
    normal: [[f32; 3]; 3],
}

impl model::Vertex for InstanceRaw {
    fn desc() -> wgpu::VertexBufferLayout<'static> {
        use std::mem;
        wgpu::VertexBufferLayout {
            array_stride: mem::size_of::<InstanceRaw>() as wgpu::BufferAddress,
            // We need to switch from using a step mode of Vertex to Instance
            // This means that our shaders will only change to use the next
            // instance when the shader starts processing a new instance
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &[
                wgpu::VertexAttribute {
                    offset: 0,
                    // While our vertex shader only uses locations 0, and 1 now, in later tutorials we'll
                    // be using 2, 3, and 4, for Vertex. We'll start at slot 5 not conflict with them later
                    shader_location: 5,
                    format: wgpu::VertexFormat::Float32x4,
                },
                // A mat4 takes up 4 vertex slots as it is technically 4 vec4s. We need to define a slot
                // for each vec4. We don't have to do this in code though.
                wgpu::VertexAttribute {
                    offset: mem::size_of::<[f32; 4]>() as wgpu::BufferAddress,
                    shader_location: 6,
                    format: wgpu::VertexFormat::Float32x4,
                },
                wgpu::VertexAttribute {
                    offset: mem::size_of::<[f32; 8]>() as wgpu::BufferAddress,
                    shader_location: 7,
                    format: wgpu::VertexFormat::Float32x4,
                },
                wgpu::VertexAttribute {
                    offset: mem::size_of::<[f32; 12]>() as wgpu::BufferAddress,
                    shader_location: 8,
                    format: wgpu::VertexFormat::Float32x4,
                },
                wgpu::VertexAttribute {
                    offset: mem::size_of::<[f32; 16]>() as wgpu::BufferAddress,
                    shader_location: 9,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: mem::size_of::<[f32; 19]>() as wgpu::BufferAddress,
                    shader_location: 10,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: mem::size_of::<[f32; 22]>() as wgpu::BufferAddress,
                    shader_location: 11,
                    format: wgpu::VertexFormat::Float32x3,
                },
            ],
        }
    }
}

#[derive(Clone)]
pub struct Instance {
    pub position: Vec3,
    pub rotation: Quat,
}

impl Instance {
    pub(crate) fn to_raw(&self) -> InstanceRaw {
        InstanceRaw {
            model: (Mat4::from_translation(self.position)
                * Mat4::from_quat(self.rotation))
                .to_cols_array_2d(),
            normal: Mat3::from_quat(self.rotation).to_cols_array_2d(),
        }
    }
}

impl State {
    pub fn update_instances(&mut self) {
        for id in 0..self.physics.rigidbodies.len() {
            let mesh = self.physics.rigidbodies.get_mesh(id);
            let index = self.physics.rigidbodies.get_index(id);
            let pos = self.physics.rigidbodies.get_position(id);
            let rot = self.physics.rigidbodies.get_orientation(id);

            self.instances[mesh][index].position = pos.to_f32();
            self.instances[mesh][index].rotation = rot.to_f32();
        }
    }

    #[allow(unused)]
    pub(crate) fn add_instance(&mut self, instance: Instance, mesh: usize) {
        let body = RigidBody::builder()
            .position(instance.position.to_float())
            .orientation(instance.rotation.to_float());

        self.physics.rigidbodies.add_body(body);
        self.instances[mesh].push(instance);


        self.instance_buffers[mesh].destroy();
        let instance_data = self.instances[mesh].iter().map(Instance::to_raw).collect::<Vec<_>>();
        self.instance_buffers[mesh] = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Instance Buffer"),
            contents: bytemuck::cast_slice(&instance_data),
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        });

    }
}