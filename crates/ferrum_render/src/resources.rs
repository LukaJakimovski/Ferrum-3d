use std::fs;
use std::io::{BufReader, Cursor};
use egui_wgpu::wgpu;
use glam::{Vec2, Vec3};
use wgpu::util::DeviceExt;
use ferrum_core::math;
use ferrum_core::math::Float;
use ferrum_physics::physics_vertex::{Face, Polyhedron};
use crate::{model, texture};
use crate::texture::Texture;

pub async fn load_string(file_name: &str) -> anyhow::Result<String> {
    let txt = {
        let path = std::env::current_dir()?
            .join("crates/ferrum_render/res")
            .join(file_name);
        std::fs::read_to_string(path)?
    };

    Ok(txt)
}

pub async fn load_binary(file_name: &str) -> anyhow::Result<Vec<u8>> {
    let data;
    if !file_name.is_empty(){
        data = {
            let path = std::env::current_dir()?
                .join("crates/ferrum_render/res")
                .join(file_name);
            std::fs::read(path)?
        };
    } else {
        data = vec![];
    }


    Ok(data)
}

pub async fn load_texture(
    file_name: &str,
    is_normal_map: bool,
    device: &wgpu::Device,
    queue: &wgpu::Queue,
) -> anyhow::Result<texture::Texture> {
    let data = load_binary(file_name).await?;
    texture::Texture::from_bytes(device, queue, &data, file_name, is_normal_map)
}

pub fn load_polyhedron(file_name: &str) -> Polyhedron {
    let path = std::env::current_dir()
        .expect("failed to get current dir")
        .join("crates/ferrum_render/res")
        .join(file_name);
    let contents = fs::read_to_string(path)
        .expect("Could not read file");

    let mut p: Polyhedron = Default::default();


    for line in contents.lines() {
        if line.as_bytes()[0] == 'v' as u8 {
            if line.as_bytes()[1] == ' ' as u8 {
                let floats: Vec<Float> = line
                    .split_whitespace()
                    .skip(1)
                    .map(|s| s.parse::<Float>().expect("Failed to parse float"))
                    .collect();

                p.vert.push(math::Vec3::new(
                    floats[0] as Float,
                    floats[1] as Float,
                    floats[2] as Float));

            }
        } else if line.as_bytes()[0] == 'f' as u8 {
            let mut f: Face = Default::default();

            let indices: Vec<Vec<usize>> = line
                .split_whitespace()
                .skip(1)
                .map(|group| {
                    group.split('/')
                        .map(|s| s.parse::<usize>()
                            .unwrap_or(0))
                        .collect()
                })
                .collect();
            for i in 0..indices.len() {
                f.verts.push(indices[i][0] - 1);
            }

            let d1 = p.vert[f.verts[1]] - p.vert[f.verts[0]];
            let d2 = p.vert[f.verts[2]] - p.vert[f.verts[0]];
            let n = d1.cross(d2);
            f.norm = n / n.length();

            f.w = -f.norm.x * p.vert[f.verts[0]].x
                  -f.norm.y * p.vert[f.verts[0]].y
                  -f.norm.z * p.vert[f.verts[0]].z;
            p.faces.push(f);
        }
    }
    p
}

pub async fn load_model(
    file_name: &str,
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    layout: &wgpu::BindGroupLayout,
) -> anyhow::Result<model::Model> {
    let obj_text = load_string(file_name).await?;
    let obj_cursor = Cursor::new(obj_text);
    let mut obj_reader = BufReader::new(obj_cursor);

    let (models, obj_materials) = tobj::load_obj_buf_async(
        &mut obj_reader,
        &tobj::LoadOptions {
            triangulate: true,
            single_index: true,
            ..Default::default()
        },
        |p| async move {
            let mat_text = load_string(&p).await.unwrap();
            tobj::load_mtl_buf(&mut BufReader::new(Cursor::new(mat_text)))
        },
    )
        .await?;

    let mut materials = Vec::new();
    for m in obj_materials? {
        let diffuse_texture = load_texture(&m.diffuse_texture, false, device, queue).await?;
        materials.push(model::Material::new(
            device,
            &m.name,
            diffuse_texture,
            layout,
        ));
    }


    if materials.len() == 0 {
        let dimensions = (1, 1);
        let rgba = [255, 0, 0, 255];

        let size = wgpu::Extent3d {
            width: dimensions.0,
            height: dimensions.1,
            depth_or_array_layers: 1,
        };
        let format = wgpu::TextureFormat::Rgba8UnormSrgb;
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: None,
            size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });

        queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                aspect: wgpu::TextureAspect::All,
                texture: &texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
            },
            &rgba,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(4 * dimensions.0),
                rows_per_image: Some(dimensions.1),
            },
            size,
        );

        let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Nearest,
            mipmap_filter: wgpu::FilterMode::Nearest,
            ..Default::default()
        });

        let diffuse_texture = Texture { texture, view, sampler };

        materials.push(
            model::Material::new(
            device,
            "default",
            diffuse_texture,
            layout,
            ));
    }


    let meshes = models
        .into_iter()
        .map(|m| {
            let mut vertices = (0..m.mesh.positions.len() / 3)
                .map(|i| model::ModelVertex {
                    position: [
                        m.mesh.positions[i * 3] / 10.0,
                        m.mesh.positions[i * 3 + 1] / 10.0,
                        m.mesh.positions[i * 3 + 2] / 10.0,
                    ],

                    tex_coords: if (i * 2 + 1) < m.mesh.texcoords.len() { [m.mesh.texcoords[i * 2], 1.0 - m.mesh.texcoords[i * 2 + 1]]} else {[0.0, 0.0]},
                    normal: if !m.mesh.normals.is_empty() {[
                        m.mesh.normals[i * 3],
                        m.mesh.normals[i * 3 + 1],
                        m.mesh.normals[i * 3 + 2],
                    ]} else {[0.0, 0.0, 0.0]},
                    // We'll calculate these later
                    tangent: [0.0; 3],
                    bitangent: [0.0; 3],
                })
                .collect::<Vec<_>>();

            let indices = &m.mesh.indices;
            let mut triangles_included = vec![0; vertices.len()];

            // Calculate tangents and bitangets. We're going to
            // use the triangles, so we need to loop through the
            // indices in chunks of 3
            for c in indices.chunks(3) {
                let v0 = vertices[c[0] as usize];
                let v1 = vertices[c[1] as usize];
                let v2 = vertices[c[2] as usize];

                let pos0: Vec3 = v0.position.into();
                let pos1: Vec3 = v1.position.into();
                let pos2: Vec3 = v2.position.into();

                let uv0: Vec2 = v0.tex_coords.into();
                let uv1: Vec2 = v1.tex_coords.into();
                let uv2: Vec2 = v2.tex_coords.into();

                // Calculate the edges of the triangle
                let delta_pos1 = pos1 - pos0;
                let delta_pos2 = pos2 - pos0;

                // This will give us a direction to calculate the
                // tangent and bitangent
                let delta_uv1 = uv1 - uv0;
                let delta_uv2 = uv2 - uv0;

                // Solving the following system of equations will
                // give us the tangent and bitangent.
                //     delta_pos1 = delta_uv1.x * T + delta_u.y * B
                //     delta_pos2 = delta_uv2.x * T + delta_uv2.y * B
                // Luckily, the place I found this equation provided
                // the solution!
                let r = 1.0 / (delta_uv1.x * delta_uv2.y - delta_uv1.y * delta_uv2.x);
                let tangent = (delta_pos1 * delta_uv2.y - delta_pos2 * delta_uv1.y) * r;
                // We flip the bitangent to enable right-handed normal
                // maps with wgpu texture coordinate system
                let bitangent = (delta_pos2 * delta_uv1.x - delta_pos1 * delta_uv2.x) * -r;

                // We'll use the same tangent/bitangent for each vertex in the triangle
                vertices[c[0] as usize].tangent =
                    (tangent + Vec3::from(vertices[c[0] as usize].tangent)).into();
                vertices[c[1] as usize].tangent =
                    (tangent + Vec3::from(vertices[c[1] as usize].tangent)).into();
                vertices[c[2] as usize].tangent =
                    (tangent + Vec3::from(vertices[c[2] as usize].tangent)).into();
                vertices[c[0] as usize].bitangent =
                    (bitangent + Vec3::from(vertices[c[0] as usize].bitangent)).into();
                vertices[c[1] as usize].bitangent =
                    (bitangent + Vec3::from(vertices[c[1] as usize].bitangent)).into();
                vertices[c[2] as usize].bitangent =
                    (bitangent + Vec3::from(vertices[c[2] as usize].bitangent)).into();

                // Used to average the tangents/bitangents
                triangles_included[c[0] as usize] += 1;
                triangles_included[c[1] as usize] += 1;
                triangles_included[c[2] as usize] += 1;
            }

            // Average the tangents/bitangents
            for (i, n) in triangles_included.into_iter().enumerate() {
                let denom = 1.0 / n as f32;
                let v = &mut vertices[i];
                v.tangent = (Vec3::from(v.tangent) * denom).into();
                v.bitangent = (Vec3::from(v.bitangent) * denom).into();
            }

            let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(&format!("{:?} Vertex Buffer", file_name)),
                contents: bytemuck::cast_slice(&vertices),
                usage: wgpu::BufferUsages::VERTEX,
            });
            let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(&format!("{:?} Index Buffer", file_name)),
                contents: bytemuck::cast_slice(&m.mesh.indices),
                usage: wgpu::BufferUsages::INDEX,
            });

            model::Mesh {
                name: file_name.to_string(),
                vertex_buffer,
                index_buffer,
                num_elements: m.mesh.indices.len() as u32,
                material: m.mesh.material_id.unwrap_or(0),
            }
        })
        .collect::<Vec<_>>();

    Ok(model::Model { meshes, materials })
}