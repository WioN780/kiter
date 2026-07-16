struct Camera {
    view_proj: mat4x4<f32>,
    light_dir: vec4<f32>,
};
@group(0) @binding(0) var<uniform> camera: Camera;

// ---------------------------------------------------------------------
// Pipeline 1: instanced lit primitives (unit cylinder / sphere / cone)
// ---------------------------------------------------------------------

struct InstIn {
    @location(0) pos: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) m0: vec4<f32>,
    @location(3) m1: vec4<f32>,
    @location(4) m2: vec4<f32>,
    @location(5) m3: vec4<f32>,
    @location(6) color: vec4<f32>,
};

struct LitOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) world_normal: vec3<f32>,
    @location(1) color: vec4<f32>,
};

@vertex
fn vs_instanced(in: InstIn) -> LitOut {
    let model = mat4x4<f32>(in.m0, in.m1, in.m2, in.m3);
    var out: LitOut;
    out.clip_pos = camera.view_proj * model * vec4<f32>(in.pos, 1.0);
    // ponytail: normal transformed by model's linear part directly (no inverse-
    // transpose). Correct for uniform scale (spheres) and near-correct for the
    // radius/length scale used by cylinders/cones; visibly fine for debug viz.
    out.world_normal = normalize((model * vec4<f32>(in.normal, 0.0)).xyz);
    out.color = in.color;
    return out;
}

fn lambert(n: vec3<f32>, base: vec4<f32>) -> vec4<f32> {
    let l = normalize(camera.light_dir.xyz);
    let diffuse = max(dot(n, l), 0.0);
    let ambient = 0.35;
    let shade = ambient + (1.0 - ambient) * diffuse;
    return vec4<f32>(base.rgb * shade, base.a);
}

@fragment
fn fs_lit(in: LitOut) -> @location(0) vec4<f32> {
    return lambert(normalize(in.world_normal), in.color);
}

// ---------------------------------------------------------------------
// Pipeline 2: unlit line list (grid, axes, debug lines)
// ---------------------------------------------------------------------

struct LineIn {
    @location(0) pos: vec3<f32>,
    @location(1) color: vec4<f32>,
};

struct LineOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_line(in: LineIn) -> LineOut {
    var out: LineOut;
    out.clip_pos = camera.view_proj * vec4<f32>(in.pos, 1.0);
    out.color = in.color;
    return out;
}

@fragment
fn fs_line(in: LineOut) -> @location(0) vec4<f32> {
    return in.color;
}

// ---------------------------------------------------------------------
// Pipeline 3: dynamic mesh, double-sided, flat-ish shaded, alpha allowed
// ---------------------------------------------------------------------

struct MeshIn {
    @location(0) pos: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
};

@vertex
fn vs_mesh(in: MeshIn) -> LitOut {
    var out: LitOut;
    out.clip_pos = camera.view_proj * vec4<f32>(in.pos, 1.0);
    out.world_normal = in.normal;
    out.color = in.color;
    return out;
}

@fragment
fn fs_mesh(in: LitOut, @builtin(front_facing) front: bool) -> @location(0) vec4<f32> {
    var n = normalize(in.world_normal);
    if (!front) {
        n = -n;
    }
    return lambert(n, in.color);
}
