use egui_winit::winit;
use egui_wgpu::wgpu;
use std::sync::Arc;
use wgpu::util::DeviceExt;
use winit::window::Window;
use ferrum_core::constants::*;

mod camera;
mod model;
mod resources;
mod texture;
mod instance;
mod input;
mod gui;
mod arrows;
mod update;
mod render;

use model::Vertex;
use camera::CameraUniform;
use ferrum_core::math::{ToGlamQuat, ToGlamVec3};
use ferrum_core::time::now;
use crate::instance::{InstanceRaw, Instance};
use ferrum_physics::rigidbody_set::RigidBodySet;
use ferrum_physics::Physics;
use ferrum_core::timing::Timing;
use ferrum_physics::polyhedron::{Polyhedron};
use crate::gui::egui_tools::EguiRenderer;
use crate::render::create_render_pipeline;
use crate::resources::load_polyhedron;
use crate::arrows::arrows::Arrow;

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
    pub timer: Timing,
    pub is_pointer_used: bool,
    pub selected_index: usize,
    arrows: Vec<Arrow>,
}

impl State {
    pub async fn new(
        window: Arc<Window>,
    ) -> anyhow::Result<State> {
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
        for name in OBJ_NAMES {
            obj_models.push(resources::load_model(name, &device, &queue, &texture_bind_group_layout)
                .await?);
        }

        let mut polyhedrons: Vec<Polyhedron> = Default::default();
        for name in OBJ_NAMES {
            polyhedrons.push(load_polyhedron(name));
        }

        let mut instances = vec![vec![]; OBJ_NAMES.len()];
        let mut physics: Physics = Physics { rigidbodies: RigidBodySet::new(0), polyhedrons, energy: Default::default(), parameters: Default::default() };
        let mut arrows: Vec<Arrow> = Default::default();
        physics.two_objects();

        for i in 0..physics.rigidbodies.len() {
            let mesh = physics.rigidbodies.get_mesh(i);
            let position = physics.rigidbodies.get_position(i).to_glam_vec3();
            let rotation = physics.rigidbodies.get_orientation(i).to_glam_quat();
            physics.rigidbodies.index[i] = instances[mesh].len();
            instances[mesh].push(Instance {position, rotation});
            let index = instances[mesh].len() - 1;
            if mesh == Mesh::Arrow as usize {
                arrows.push( Arrow { transform: Some(&mut instances[mesh][index]), vec: Some(&physics.rigidbodies.velocities[i])});
            }

        }
        let mut instance_buffers = vec![];
        for instance in &instances {
            let instance_data = instance.iter().map(Instance::to_raw).collect::<Vec<_>>();
            instance_buffers.push(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Instance Buffer"),
                contents: bytemuck::cast_slice(&instance_data),
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            }));
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
            mouse_pressed: false,
            physics,
            menus: [false; 16],
            timer: Timing { start_time: now(), ..Default::default()},
            is_pointer_used: false,
            selected_index: 0,
            arrows,
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
}