//! GPU-side scene renderer: three pipelines (instanced lit primitives, unlit
//! lines, double-sided dynamic mesh) driven by an egui_wgpu paint callback.
//!
//! `SceneData` (below) is the frozen data contract other modules build scenes
//! against. Everything else in this file is renderer plumbing.

use eframe::egui_wgpu;
use eframe::wgpu::{self, util::DeviceExt};
use glam::{DVec3, Mat4};

// ---------------------------------------------------------------------
// Scene data contract
// ---------------------------------------------------------------------

pub struct CylinderInst {
    pub a: DVec3,
    pub b: DVec3,
    pub radius: f64,
    pub color: [f32; 4],
}

pub struct SphereInst {
    pub center: DVec3,
    pub radius: f64,
    pub color: [f32; 4],
}

pub struct LineSeg {
    pub a: DVec3,
    pub b: DVec3,
    pub color: [f32; 4],
}

pub struct ArrowInst {
    pub origin: DVec3,
    pub vec: DVec3,
    pub shaft_radius: f64,
    pub color: [f32; 4],
}

pub struct DynMesh {
    pub positions: Vec<DVec3>,
    pub indices: Vec<u32>,
    pub color: [f32; 4],
}

#[derive(Default)]
pub struct SceneData {
    pub cylinders: Vec<CylinderInst>,
    pub spheres: Vec<SphereInst>,
    pub lines: Vec<LineSeg>,
    pub arrows: Vec<ArrowInst>,
    pub meshes: Vec<DynMesh>,
    pub draw_grid: bool,
    pub draw_axes: bool,
}

// ---------------------------------------------------------------------
// GPU vertex / instance layouts
// ---------------------------------------------------------------------

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct VertexPN {
    pos: [f32; 3],
    normal: [f32; 3],
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct InstanceRaw {
    model: [[f32; 4]; 4],
    color: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct LineVertex {
    pos: [f32; 3],
    color: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct MeshVertex {
    pos: [f32; 3],
    normal: [f32; 3],
    color: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct CameraUniform {
    view_proj: [[f32; 4]; 4],
    light_dir: [f32; 4],
}

const VERTEX_PN_ATTRS: [wgpu::VertexAttribute; 2] =
    wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3];
const INSTANCE_ATTRS: [wgpu::VertexAttribute; 5] =
    wgpu::vertex_attr_array![2 => Float32x4, 3 => Float32x4, 4 => Float32x4, 5 => Float32x4, 6 => Float32x4];
const LINE_ATTRS: [wgpu::VertexAttribute; 2] = wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x4];
const MESH_ATTRS: [wgpu::VertexAttribute; 3] =
    wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x4];

fn instance_from(model: glam::DMat4, color: [f32; 4]) -> InstanceRaw {
    InstanceRaw {
        model: model.as_mat4().to_cols_array_2d(),
        color,
    }
}

/// Model matrix for a unit shape (built along local +Z, 0..1) stretched to
/// span `a..b` with the given radial scale.
fn segment_model(a: DVec3, b: DVec3, radial: f64) -> Option<glam::DMat4> {
    let axis = b - a;
    let len = axis.length();
    if len < 1e-9 {
        return None;
    }
    let dir = axis / len;
    let rot = glam::DQuat::from_rotation_arc(DVec3::Z, dir);
    Some(glam::DMat4::from_scale_rotation_translation(
        DVec3::new(radial, radial, len),
        rot,
        a,
    ))
}

// ---------------------------------------------------------------------
// Unit mesh generation (CPU, built once at registration time)
// ---------------------------------------------------------------------

struct MeshBuf {
    vbuf: wgpu::Buffer,
    ibuf: wgpu::Buffer,
    index_count: u32,
}

impl MeshBuf {
    fn new(device: &wgpu::Device, label: &str, verts: &[VertexPN], indices: &[u32]) -> Self {
        let vbuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(&format!("{label}-vbuf")),
            contents: bytemuck::cast_slice(verts),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let ibuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(&format!("{label}-ibuf")),
            contents: bytemuck::cast_slice(indices),
            usage: wgpu::BufferUsages::INDEX,
        });
        Self {
            vbuf,
            ibuf,
            index_count: indices.len() as u32,
        }
    }
}

const SEGMENTS: u32 = 12;

/// Unit cylinder: radius 1, spans local Z in [0, 1], with end caps.
fn build_cylinder(segs: u32) -> (Vec<VertexPN>, Vec<u32>) {
    let mut v = Vec::new();
    let mut idx = Vec::new();
    // sides
    for i in 0..segs {
        let a = i as f32 / segs as f32 * std::f32::consts::TAU;
        let (s, c) = a.sin_cos();
        let n = [c, s, 0.0];
        v.push(VertexPN { pos: [c, s, 0.0], normal: n });
        v.push(VertexPN { pos: [c, s, 1.0], normal: n });
    }
    for i in 0..segs {
        let j = (i + 1) % segs;
        let (i0, i1, j0, j1) = (i * 2, i * 2 + 1, j * 2, j * 2 + 1);
        idx.extend_from_slice(&[i0, j0, i1, i1, j0, j1]);
    }
    // caps
    for (z, n, flip) in [(0.0f32, -1.0f32, true), (1.0, 1.0, false)] {
        let center = v.len() as u32;
        v.push(VertexPN { pos: [0.0, 0.0, z], normal: [0.0, 0.0, n] });
        let ring0 = v.len() as u32;
        for i in 0..segs {
            let a = i as f32 / segs as f32 * std::f32::consts::TAU;
            let (s, c) = a.sin_cos();
            v.push(VertexPN { pos: [c, s, z], normal: [0.0, 0.0, n] });
        }
        for i in 0..segs {
            let j = (i + 1) % segs;
            if flip {
                idx.extend_from_slice(&[center, ring0 + j, ring0 + i]);
            } else {
                idx.extend_from_slice(&[center, ring0 + i, ring0 + j]);
            }
        }
    }
    (v, idx)
}

/// Unit UV sphere, radius 1 centered at origin.
fn build_sphere(segs: u32, rings: u32) -> (Vec<VertexPN>, Vec<u32>) {
    let mut v = Vec::new();
    for r in 0..=rings {
        let phi = r as f32 / rings as f32 * std::f32::consts::PI;
        let (sy, y) = phi.sin_cos();
        for s in 0..=segs {
            let theta = s as f32 / segs as f32 * std::f32::consts::TAU;
            let (st, ct) = theta.sin_cos();
            let pos = [sy * ct, y, sy * st];
            v.push(VertexPN { pos, normal: pos });
        }
    }
    let mut idx = Vec::new();
    let stride = segs + 1;
    for r in 0..rings {
        for s in 0..segs {
            let i0 = r * stride + s;
            let i1 = i0 + 1;
            let j0 = i0 + stride;
            let j1 = j0 + 1;
            idx.extend_from_slice(&[i0, j0, i1, i1, j0, j1]);
        }
    }
    (v, idx)
}

/// Unit cone: base ring radius 1 at z=0, apex at z=1, with a base cap.
/// ponytail: normal blend is a fixed 45-degree approximation, not the exact
/// cone slope normal — fine for small arrowhead accents in a debug view.
fn build_cone(segs: u32) -> (Vec<VertexPN>, Vec<u32>) {
    let mut v = Vec::new();
    let mut idx = Vec::new();
    let k = std::f32::consts::FRAC_1_SQRT_2;
    for i in 0..segs {
        let a = i as f32 / segs as f32 * std::f32::consts::TAU;
        let (s, c) = a.sin_cos();
        let n = [c * k, s * k, k];
        v.push(VertexPN { pos: [c, s, 0.0], normal: n });
        v.push(VertexPN { pos: [0.0, 0.0, 1.0], normal: n });
    }
    for i in 0..segs {
        let j = (i + 1) % segs;
        idx.extend_from_slice(&[i * 2, j * 2, i * 2 + 1]);
    }
    let center = v.len() as u32;
    v.push(VertexPN { pos: [0.0, 0.0, 0.0], normal: [0.0, 0.0, -1.0] });
    let ring0 = v.len() as u32;
    for i in 0..segs {
        let a = i as f32 / segs as f32 * std::f32::consts::TAU;
        let (s, c) = a.sin_cos();
        v.push(VertexPN { pos: [c, s, 0.0], normal: [0.0, 0.0, -1.0] });
    }
    for i in 0..segs {
        let j = (i + 1) % segs;
        idx.extend_from_slice(&[center, ring0 + j, ring0 + i]);
    }
    (v, idx)
}

// ---------------------------------------------------------------------
// Persistent GPU resources (registered once into egui_wgpu callback_resources)
// ---------------------------------------------------------------------

pub struct GpuResources {
    camera_buf: wgpu::Buffer,
    camera_bind_group: wgpu::BindGroup,
    inst_pipeline: wgpu::RenderPipeline,
    line_pipeline: wgpu::RenderPipeline,
    mesh_pipeline: wgpu::RenderPipeline,
    cylinder: MeshBuf,
    sphere: MeshBuf,
    cone: MeshBuf,
    // ponytail: rebuilt from scratch every frame (immediate-mode) instead of
    // resized/reused; simplest correct thing at this scale (dozens-hundreds
    // of primitives), revisit if scenes grow to thousands of instances.
    cyl_instances: Option<(wgpu::Buffer, u32)>,
    sphere_instances: Option<(wgpu::Buffer, u32)>,
    cone_instances: Option<(wgpu::Buffer, u32)>,
    line_verts: Option<(wgpu::Buffer, u32)>,
    mesh_verts: Option<(wgpu::Buffer, u32)>,
}

const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth24Plus;

impl GpuResources {
    pub fn new(device: &wgpu::Device, target_format: wgpu::TextureFormat) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("kite-viz-shaders"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders.wgsl").into()),
        });

        let camera_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("camera-uniform"),
            contents: bytemuck::bytes_of(&CameraUniform {
                view_proj: Mat4::IDENTITY.to_cols_array_2d(),
                light_dir: [0.4, 0.8, 0.3, 0.0],
            }),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        let camera_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("camera-bgl"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("camera-bg"),
            layout: &camera_bgl,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buf.as_entire_binding(),
            }],
        });

        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("kite-viz-pipeline-layout"),
            bind_group_layouts: &[Some(&camera_bgl)],
            immediate_size: 0,
        });

        let depth_state = |write: bool| {
            Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: Some(write),
                depth_compare: Some(wgpu::CompareFunction::Less),
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            })
        };
        // ponytail: no backface culling anywhere (cull_mode: None). Removes a
        // whole class of winding-order bugs; solid shapes still look correct
        // via the depth test, at a small, irrelevant-at-this-scale perf cost.
        let primitive = wgpu::PrimitiveState {
            topology: wgpu::PrimitiveTopology::TriangleList,
            cull_mode: None,
            ..Default::default()
        };

        let inst_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("inst-pipeline"),
            layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_instanced"),
                compilation_options: Default::default(),
                buffers: &[
                    wgpu::VertexBufferLayout {
                        array_stride: std::mem::size_of::<VertexPN>() as u64,
                        step_mode: wgpu::VertexStepMode::Vertex,
                        attributes: &VERTEX_PN_ATTRS,
                    },
                    wgpu::VertexBufferLayout {
                        array_stride: std::mem::size_of::<InstanceRaw>() as u64,
                        step_mode: wgpu::VertexStepMode::Instance,
                        attributes: &INSTANCE_ATTRS,
                    },
                ],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_lit"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: target_format,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive,
            depth_stencil: depth_state(true),
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });

        let line_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("line-pipeline"),
            layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_line"),
                compilation_options: Default::default(),
                buffers: &[wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<LineVertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &LINE_ATTRS,
                }],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_line"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: target_format,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                ..Default::default()
            },
            depth_stencil: depth_state(true),
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });

        let mesh_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("mesh-pipeline"),
            layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_mesh"),
                compilation_options: Default::default(),
                buffers: &[wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<MeshVertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &MESH_ATTRS,
                }],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_mesh"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: target_format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive,
            // ponytail: depth write stays on even for the translucent mesh
            // pipeline; correct vs. opaque geometry, only same-mesh
            // self-overlap ordering could look slightly off. Fine for a demo.
            depth_stencil: depth_state(true),
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });

        let (cv, ci) = build_cylinder(SEGMENTS);
        let (sv, si) = build_sphere(SEGMENTS, SEGMENTS / 2);
        let (nv, ni) = build_cone(SEGMENTS);

        Self {
            camera_buf,
            camera_bind_group,
            inst_pipeline,
            line_pipeline,
            mesh_pipeline,
            cylinder: MeshBuf::new(device, "cylinder", &cv, &ci),
            sphere: MeshBuf::new(device, "sphere", &sv, &si),
            cone: MeshBuf::new(device, "cone", &nv, &ni),
            cyl_instances: None,
            sphere_instances: None,
            cone_instances: None,
            line_verts: None,
            mesh_verts: None,
        }
    }
}

// ---------------------------------------------------------------------
// Per-frame paint callback
// ---------------------------------------------------------------------

pub struct FrameCallback {
    view_proj: Mat4,
    cyl_instances: Vec<InstanceRaw>,
    sphere_instances: Vec<InstanceRaw>,
    cone_instances: Vec<InstanceRaw>,
    line_verts: Vec<LineVertex>,
    mesh_verts: Vec<MeshVertex>,
}

fn push_grid(out: &mut Vec<LineVertex>) {
    let half = 10i32;
    for i in -half..=half {
        let x = i as f32;
        let major = i == 0;
        let c = if major { [0.55, 0.55, 0.6, 1.0] } else { [0.28, 0.28, 0.32, 1.0] };
        out.push(LineVertex { pos: [x, 0.0, -half as f32], color: c });
        out.push(LineVertex { pos: [x, 0.0, half as f32], color: c });
        out.push(LineVertex { pos: [-half as f32, 0.0, x], color: c });
        out.push(LineVertex { pos: [half as f32, 0.0, x], color: c });
    }
}

fn push_axes(out: &mut Vec<LineVertex>) {
    let o = [0.0, 0.0, 0.0];
    let axes: [([f32; 3], [f32; 4]); 3] = [
        ([1.0, 0.0, 0.0], [0.9, 0.15, 0.15, 1.0]),
        ([0.0, 1.0, 0.0], [0.15, 0.85, 0.15, 1.0]),
        ([0.0, 0.0, 1.0], [0.2, 0.4, 0.95, 1.0]),
    ];
    for (dir, color) in axes {
        out.push(LineVertex { pos: o, color });
        out.push(LineVertex { pos: dir, color });
    }
}

impl FrameCallback {
    pub fn build(scene: &SceneData, view_proj: Mat4) -> Self {
        let mut cyl_instances = Vec::with_capacity(scene.cylinders.len() + scene.arrows.len());
        for c in &scene.cylinders {
            if let Some(m) = segment_model(c.a, c.b, c.radius) {
                cyl_instances.push(instance_from(m, c.color));
            }
        }

        let mut sphere_instances = Vec::with_capacity(scene.spheres.len());
        for s in &scene.spheres {
            let m = glam::DMat4::from_scale_rotation_translation(
                DVec3::splat(s.radius),
                glam::DQuat::IDENTITY,
                s.center,
            );
            sphere_instances.push(instance_from(m, s.color));
        }

        let mut cone_instances = Vec::new();
        for arrow in &scene.arrows {
            let len = arrow.vec.length();
            if len < 1e-9 {
                continue;
            }
            let tip_len = (len * 0.25).max(arrow.shaft_radius * 2.0).min(len);
            let shaft_len = len - tip_len;
            let shaft_end = arrow.origin + arrow.vec / len * shaft_len;
            if shaft_len > 1e-9 {
                if let Some(m) = segment_model(arrow.origin, shaft_end, arrow.shaft_radius) {
                    cyl_instances.push(instance_from(m, arrow.color));
                }
            }
            let tip_end = arrow.origin + arrow.vec;
            if let Some(m) = segment_model(shaft_end, tip_end, arrow.shaft_radius * 2.5) {
                cone_instances.push(instance_from(m, arrow.color));
            }
        }

        let mut line_verts = Vec::with_capacity(scene.lines.len() * 2);
        for l in &scene.lines {
            line_verts.push(LineVertex { pos: l.a.as_vec3().to_array(), color: l.color });
            line_verts.push(LineVertex { pos: l.b.as_vec3().to_array(), color: l.color });
        }
        if scene.draw_grid {
            push_grid(&mut line_verts);
        }
        if scene.draw_axes {
            push_axes(&mut line_verts);
        }

        let mut mesh_verts = Vec::new();
        for m in &scene.meshes {
            for tri in m.indices.chunks_exact(3) {
                let (Some(&ia), Some(&ib), Some(&ic)) = (
                    m.positions.get(tri[0] as usize),
                    m.positions.get(tri[1] as usize),
                    m.positions.get(tri[2] as usize),
                ) else {
                    continue;
                };
                let normal = (ib - ia).cross(ic - ia);
                let normal = if normal.length_squared() > 1e-18 {
                    normal.normalize().as_vec3().to_array()
                } else {
                    [0.0, 1.0, 0.0]
                };
                for p in [ia, ib, ic] {
                    mesh_verts.push(MeshVertex { pos: p.as_vec3().to_array(), normal, color: m.color });
                }
            }
        }

        Self {
            view_proj,
            cyl_instances,
            sphere_instances,
            cone_instances,
            line_verts,
            mesh_verts,
        }
    }
}

fn upload<T: bytemuck::Pod>(device: &wgpu::Device, label: &str, data: &[T], usage: wgpu::BufferUsages) -> Option<(wgpu::Buffer, u32)> {
    if data.is_empty() {
        return None;
    }
    let buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some(label),
        contents: bytemuck::cast_slice(data),
        usage,
    });
    Some((buf, data.len() as u32))
}

impl egui_wgpu::CallbackTrait for FrameCallback {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _screen_descriptor: &egui_wgpu::ScreenDescriptor,
        _egui_encoder: &mut wgpu::CommandEncoder,
        callback_resources: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        let res = callback_resources
            .get_mut::<GpuResources>()
            .expect("GpuResources registered in Viewport::new");

        queue.write_buffer(
            &res.camera_buf,
            0,
            bytemuck::bytes_of(&CameraUniform {
                view_proj: self.view_proj.to_cols_array_2d(),
                light_dir: [0.4, 0.8, 0.3, 0.0],
            }),
        );

        res.cyl_instances = upload(device, "cyl-instances", &self.cyl_instances, wgpu::BufferUsages::VERTEX);
        res.sphere_instances = upload(device, "sphere-instances", &self.sphere_instances, wgpu::BufferUsages::VERTEX);
        res.cone_instances = upload(device, "cone-instances", &self.cone_instances, wgpu::BufferUsages::VERTEX);
        res.line_verts = upload(device, "line-verts", &self.line_verts, wgpu::BufferUsages::VERTEX);
        res.mesh_verts = upload(device, "mesh-verts", &self.mesh_verts, wgpu::BufferUsages::VERTEX);

        Vec::new()
    }

    fn paint(
        &self,
        _info: eframe::epaint::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        callback_resources: &egui_wgpu::CallbackResources,
    ) {
        let res = callback_resources
            .get::<GpuResources>()
            .expect("GpuResources registered in Viewport::new");

        render_pass.set_bind_group(0, &res.camera_bind_group, &[]);

        render_pass.set_pipeline(&res.inst_pipeline);
        let draw_mesh = |render_pass: &mut wgpu::RenderPass<'static>, mesh: &MeshBuf, instances: &Option<(wgpu::Buffer, u32)>| {
            if let Some((ibuf, count)) = instances {
                render_pass.set_vertex_buffer(0, mesh.vbuf.slice(..));
                render_pass.set_vertex_buffer(1, ibuf.slice(..));
                render_pass.set_index_buffer(mesh.ibuf.slice(..), wgpu::IndexFormat::Uint32);
                render_pass.draw_indexed(0..mesh.index_count, 0, 0..*count);
            }
        };
        draw_mesh(render_pass, &res.cylinder, &res.cyl_instances);
        draw_mesh(render_pass, &res.sphere, &res.sphere_instances);
        draw_mesh(render_pass, &res.cone, &res.cone_instances);

        if let Some((vbuf, count)) = &res.mesh_verts {
            render_pass.set_pipeline(&res.mesh_pipeline);
            render_pass.set_vertex_buffer(0, vbuf.slice(..));
            render_pass.draw(0..*count, 0..1);
        }

        if let Some((vbuf, count)) = &res.line_verts {
            render_pass.set_pipeline(&res.line_pipeline);
            render_pass.set_vertex_buffer(0, vbuf.slice(..));
            render_pass.draw(0..*count, 0..1);
        }
    }
}
