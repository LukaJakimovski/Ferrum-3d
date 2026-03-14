use egui_winit::winit;
use egui_wgpu::wgpu;
use std::sync::Arc;
use std::{f32::consts::PI, iter};
use wgpu::util::DeviceExt;
use winit::window::Window;

mod camera;
mod model;
mod resources;
mod texture;
mod instance;
mod input;
mod gui;

use model::{DrawLight, DrawModel, Vertex};
use camera::{CameraUniform};
use glam::{Vec3, Quat};
use ferrum_core::math::{Float, ToFloat};
use crate::instance::{InstanceRaw, Instance};
use ferrum_physics::rigidbody::{RigidBody, RigidBodySet};
use ferrum_physics::update::Physics;
#[allow(unused)]
use rand::RngExt;
use ferrum_core::math;
use ferrum_physics::physics_vertex::{Polyhedron};
use crate::gui::egui_tools::EguiRenderer;
use crate::resources::load_polyhedron;

#[allow(unused)]
const NUM_INSTANCES_PER_ROW: u32 = 12;

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct LightUniform {
    position: [f32; 3],
    _padding: u32,
    color: [f32; 3],
    _padding2: u32,
}

pub struct State {
    pub window: Arc<Window>,
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    render_pipeline: wgpu::RenderPipeline,
    obj_models: Vec<model::Model>,
    camera: camera::Camera,
    projection: camera::Projection,
    pub camera_controller: camera::CameraController,
    camera_uniform: CameraUniform,
    camera_buffer: wgpu::Buffer,
    camera_bind_group: wgpu::BindGroup,
    instances: Vec<Vec<Instance>>,
    #[allow(dead_code)]
    instance_buffers: Vec<wgpu::Buffer>,
    depth_texture: texture::Texture,
    is_surface_configured: bool,
    light_uniform: LightUniform,
    light_buffer: wgpu::Buffer,
    light_bind_group: wgpu::BindGroup,
    light_render_pipeline: wgpu::RenderPipeline,
    #[allow(dead_code)]
    pub mouse_pressed: bool,
    physics: Physics,
    pub egui_renderer: EguiRenderer,
    pub menus: [bool; 16],
}

fn create_render_pipeline(
    device: &wgpu::Device,
    layout: &wgpu::PipelineLayout,
    color_format: wgpu::TextureFormat,
    depth_format: Option<wgpu::TextureFormat>,
    vertex_layouts: &[wgpu::VertexBufferLayout],
    shader: wgpu::ShaderModuleDescriptor,
) -> wgpu::RenderPipeline {
    let shader = device.create_shader_module(shader);

    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some(&format!("{:?}", shader)),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module: &shader,
            entry_point: Some("vs_main"),
            buffers: vertex_layouts,
            compilation_options: Default::default(),
        },
        fragment: Some(wgpu::FragmentState {
            module: &shader,
            entry_point: Some("fs_main"),
            targets: &[Some(wgpu::ColorTargetState {
                format: color_format,
                blend: Some(wgpu::BlendState {
                    alpha: wgpu::BlendComponent::REPLACE,
                    color: wgpu::BlendComponent::REPLACE,
                }),
                write_mask: wgpu::ColorWrites::ALL,
            })],
            compilation_options: Default::default(),
        }),
        primitive: wgpu::PrimitiveState {
            topology: wgpu::PrimitiveTopology::TriangleList,
            strip_index_format: None,
            front_face: wgpu::FrontFace::Ccw,
            cull_mode: Some(wgpu::Face::Back),
            polygon_mode: wgpu::PolygonMode::Fill,
            unclipped_depth: false,
            conservative: false,
        },
        depth_stencil: depth_format.map(|format| wgpu::DepthStencilState {
            format,
            depth_write_enabled: true,
            depth_compare: wgpu::CompareFunction::Less,
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: wgpu::MultisampleState {
            count: 1,
            mask: !0,
            alpha_to_coverage_enabled: false,
        },
        multiview: None,
        cache: None,
    })
}

impl State {
    pub async fn new(window: Arc<Window>) -> anyhow::Result<State> {
        let size = window.inner_size();

        let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
            backends: wgpu::Backends::PRIMARY,
            ..Default::default()
        });

        let surface = instance.create_surface(window.clone()).unwrap();

        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::default(),
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await
            .unwrap();
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: None,
                required_features: wgpu::Features::empty(),
                experimental_features: wgpu::ExperimentalFeatures::disabled(),
                required_limits: wgpu::Limits::default(),
                memory_hints: Default::default(),
                trace: wgpu::Trace::Off,
            })
            .await
            .unwrap();

        let surface_caps = surface.get_capabilities(&adapter);
        let surface_format = surface_caps
            .formats
            .iter()
            .copied()
            .find(|f| f.is_srgb())
            .unwrap_or(surface_caps.formats[0]);

        #[cfg(target_os = "windows")]
        let vsync_mode = 2;
        #[cfg(all(target_os = "windows", target_arch = "x86_64", target_env = "gnu"))]
        let vsync_mode = 1;
        #[cfg(not(target_os = "windows"))]
        let vsync_mode = 0;

        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format: surface_format,
            width: size.width,
            height: size.height,
            present_mode: surface_caps.present_modes[vsync_mode],
            alpha_mode: surface_caps.alpha_modes[0],
            view_formats: vec![],
            desired_maximum_frame_latency: 2,
        };

        let texture_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                entries: &[
                    wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Texture {
                            multisampled: false,
                            sample_type: wgpu::TextureSampleType::Float { filterable: true },
                            view_dimension: wgpu::TextureViewDimension::D2,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 1,
                        visibility: wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                        count: None,
                    },
                ],
                label: Some("texture_bind_group_layout"),
            });

        let camera = camera::Camera::new((0.0, 5.0, 10.0), -90.0f32.to_radians(), -20.0f32.to_radians());
        let projection =
            camera::Projection::new(config.width, config.height, 45.0f32.to_radians(), 0.1, 10000.0);
        let camera_controller = camera::CameraController::new(4.0, 1.0);

        let mut camera_uniform = CameraUniform::new();
        camera_uniform.update_view_proj(&camera, &projection);

        let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Camera Buffer"),
            contents: bytemuck::cast_slice(&[camera_uniform]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let mut instances = vec![vec![]];
        //instances[0].push(Instance {position: Vec3::new(-0.97000436, 0.24308753, 0.0), rotation: Quat::IDENTITY});
        //instances[0].push(Instance{position: Vec3::new(0.97000436, -0.24308753, 0.0), rotation: Quat::IDENTITY});
        instances[0].push(Instance{position: Vec3::ZERO, rotation: Quat::IDENTITY});

        let mut instance_buffers = vec![];
        for instance in &instances {
            let instance_data = instance.iter().map(Instance::to_raw).collect::<Vec<_>>();
            instance_buffers.push(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Instance Buffer"),
                contents: bytemuck::cast_slice(&instance_data),
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            }));
        }


        let camera_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
                label: Some("camera_bind_group_layout"),
            });

        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &camera_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buffer.as_entire_binding(),
            }],
            label: Some("camera_bind_group"),
        });
        let mut obj_models = vec![];
        let obj_names = vec!["corkscrew.obj"];
        for name in &obj_names {
            obj_models.push(resources::load_model(name, &device, &queue, &texture_bind_group_layout)
                .await?);
        }


        let light_uniform = LightUniform {
            position: [10.0, 10.0, 10.0],
            _padding: 0,
            color: [1.0, 1.0, 1.0],
            _padding2: 0,
        };

        let light_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Light VB"),
            contents: bytemuck::cast_slice(&[light_uniform]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let light_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
                label: None,
            });

        let light_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &light_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: light_buffer.as_entire_binding(),
            }],
            label: None,
        });

        let depth_texture =
            texture::Texture::create_depth_texture(&device, &config, "depth_texture");

        let render_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Render Pipeline Layout"),
                bind_group_layouts: &[
                    &texture_bind_group_layout,
                    &camera_bind_group_layout,
                    &light_bind_group_layout,
                ],
                push_constant_ranges: &[],
            });

        let render_pipeline = {
            let shader = wgpu::ShaderModuleDescriptor {
                label: Some("Normal Shader"),
                source: wgpu::ShaderSource::Wgsl(include_str!("../shader/shader.wgsl").into()),
            };
            create_render_pipeline(
                &device,
                &render_pipeline_layout,
                config.format,
                Some(texture::Texture::DEPTH_FORMAT),
                &[model::ModelVertex::desc(), InstanceRaw::desc()],
                shader,
            )
        };

        let light_render_pipeline = {
            let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Light Pipeline Layout"),
                bind_group_layouts: &[&camera_bind_group_layout, &light_bind_group_layout],
                push_constant_ranges: &[],
            });
            let shader = wgpu::ShaderModuleDescriptor {
                label: Some("Light Shader"),
                source: wgpu::ShaderSource::Wgsl(include_str!("../shader/light.wgsl").into()),
            };
            create_render_pipeline(
                &device,
                &layout,
                config.format,
                Some(texture::Texture::DEPTH_FORMAT),
                &[model::ModelVertex::desc()],
                shader,
            )
        };

        let egui_renderer = EguiRenderer::new(&device, config.format, &window);


        let mut polyhedrons: Vec<Polyhedron> = Default::default();
        for name in &obj_names {
            polyhedrons.push(load_polyhedron(name));
        }


        let mut physics: Physics = Physics { rigidbodies: RigidBodySet::new(0), polyhedrons, timer: Default::default(), energy: Default::default() };
        for (mesh, instance) in instances.iter().enumerate() {
            for i in 0..instance.len(){
                let body = RigidBody::builder()
                    .position(instance[i].position.to_float())
                    .orientation(instance[i].rotation.to_float())
                    .inv_mass(0.5)
                    .mesh(mesh)
                    .index(i)
                    .omega(math::Vec3::new(10.0, 0.0, 0.0));
                physics.rigidbodies.add_body(body);
                physics.rigidbodies.comp_inertia_tensor(i, &physics.polyhedrons[mesh]);
            }
        }
        physics.energy.update_energy(&physics.rigidbodies);
        physics.energy.start_energy = physics.energy.total_energy;
        //physics.rigidbodies.velocities[0] = math::Vec3::new(0.46620368, 0.43236573, 0.0);
        //physics.rigidbodies.velocities[1] = math::Vec3::new(0.46620368, 0.43236573, 0.0);
        //physics.rigidbodies.velocities[2] = math::Vec3::new(-0.93240737, -0.86473146, 0.0);
        Ok(Self {
            window,
            surface,
            device,
            queue,
            config,
            render_pipeline,
            obj_models,
            camera,
            projection,
            camera_controller,
            camera_buffer,
            camera_bind_group,
            camera_uniform,
            instances,
            instance_buffers,
            depth_texture,
            is_surface_configured: false,
            egui_renderer,
            light_uniform,
            light_buffer,
            light_bind_group,
            light_render_pipeline,
            #[allow(dead_code)]
            mouse_pressed: false,
            physics,
            menus: [false; 16],
        })
    }

    pub fn resize(&mut self, width: u32, height: u32) {
        // UPDATED!
        if width > 0 && height > 0 {
            self.projection.resize(width, height);
            self.is_surface_configured = true;
            self.config.width = width;
            self.config.height = height;
            self.surface.configure(&self.device, &self.config);
            self.depth_texture =
                texture::Texture::create_depth_texture(&self.device, &self.config, "depth_texture");
        }
    }

    pub fn update(&mut self, dt: f64) {
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

        self.physics.physics_update(dt * 1.0 as Float);
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

    pub fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
        self.window.request_redraw();
        
        if !self.is_surface_configured {
            return Ok(());
        }

        let output = self.surface.get_current_texture()?;
        let view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Render Encoder"),
            });

        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Render Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.1,
                            g: 0.2,
                            b: 0.3,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                    depth_slice: None,
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.depth_texture.view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                occlusion_query_set: None,
                timestamp_writes: None,
            });

            render_pass.set_vertex_buffer(1, self.instance_buffers[0].slice(..));
            render_pass.set_pipeline(&self.light_render_pipeline);
            render_pass.draw_light_model(
                &self.obj_models[0],
                &self.camera_bind_group,
                &self.light_bind_group,
            );
            for (mesh, instance_buffer) in self.instance_buffers.iter().enumerate() {
                render_pass.set_vertex_buffer(1, instance_buffer.slice(..));
                render_pass.set_pipeline(&self.render_pipeline);
                render_pass.draw_model_instanced(
                    &self.obj_models[mesh],
                    0..self.instances[mesh].len() as u32,
                    &self.camera_bind_group,
                    &self.light_bind_group,
                );
            }
        }
        self.create_gui(&mut encoder, &view);
        
        self.queue.submit(iter::once(encoder.finish()));
        output.present();

        Ok(())
    }
}