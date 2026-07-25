//! The 3D viewport widget: an orbit camera plus an egui_wgpu paint callback.
//! `Viewport`, `ViewportResponse`, `OrbitCamera` and `Ray` are the frozen API
//! that later milestones (editor tools, simulation playback) build against.

use crate::render::{FrameCallback, GpuResources, SceneData};
use eframe::{egui, egui_wgpu};

// ponytail: fields are unread within this crate; they're the frozen surface
// T3 (editor tools) and T4 (simulation playback) consume; allow(dead_code)
// instead of contriving a fake internal use.
#[allow(dead_code)]
pub struct Ray {
    pub origin: glam::DVec3,
    pub dir: glam::DVec3, // normalized
}

#[allow(dead_code)]
pub struct ViewportResponse {
    pub hovered: bool,
    pub pointer_ray: Option<Ray>,
    /// Raw screen-space cursor position (same sample as `pointer_ray`);
    /// used by rubber-band box select to draw the rect and test presses.
    pub pointer_pos: Option<egui::Pos2>,
    pub primary_clicked: bool,
    pub primary_down: bool,
    pub primary_released: bool,
    pub modifiers: egui::Modifiers,
    /// This frame's view-projection and viewport rect, so callers can
    /// project world points to screen space for box select without
    /// recomputing the camera matrix themselves.
    pub view_proj: glam::Mat4,
    pub rect: egui::Rect,
}

pub struct OrbitCamera {
    pub target: glam::DVec3,
    pub yaw: f64,
    pub pitch: f64,
    pub dist: f64,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            target: glam::DVec3::ZERO,
            yaw: -0.7,
            pitch: 0.5,
            dist: 14.0,
        }
    }
}

const FOVY_RADIANS: f64 = 60.0 * std::f64::consts::PI / 180.0;
const NEAR: f64 = 0.05;
const FAR: f64 = 2000.0;
const MIN_DIST: f64 = 0.05;
const MAX_DIST: f64 = 500.0;
const PITCH_LIMIT: f64 = 89.0 * std::f64::consts::PI / 180.0;

impl OrbitCamera {
    pub fn eye(&self) -> glam::DVec3 {
        let cp = self.pitch.cos();
        let dir = glam::DVec3::new(cp * self.yaw.sin(), self.pitch.sin(), cp * self.yaw.cos());
        self.target + dir * self.dist
    }

    /// f32 for GPU consumption; all camera state itself stays f64.
    pub fn view_proj(&self, aspect: f32) -> glam::Mat4 {
        let eye = self.eye();
        let view = glam::DMat4::look_at_rh(eye, self.target, glam::DVec3::Y);
        let proj = glam::DMat4::perspective_rh(FOVY_RADIANS, aspect.max(1e-6) as f64, NEAR, FAR);
        (proj * view).as_mat4()
    }

    /// Unprojects a screen-space cursor position (within `rect`) into a world ray.
    pub fn screen_ray(&self, pos: egui::Pos2, rect: egui::Rect) -> Ray {
        let aspect = rect.width() / rect.height().max(1.0);
        let ndc_x = (pos.x - rect.left()) / rect.width() * 2.0 - 1.0;
        let ndc_y = 1.0 - (pos.y - rect.top()) / rect.height() * 2.0;
        let inv = self.view_proj(aspect).inverse();
        let unproject = |z: f32| {
            let clip = inv * glam::Vec4::new(ndc_x, ndc_y, z, 1.0);
            (clip.truncate() / clip.w).as_dvec3()
        };
        let near = unproject(0.0);
        let far = unproject(1.0);
        Ray {
            origin: self.eye(),
            dir: (far - near).normalize(),
        }
    }

    fn orbit(&mut self, delta: egui::Vec2) {
        const SPEED: f64 = 0.01;
        self.yaw -= delta.x as f64 * SPEED;
        self.pitch = (self.pitch + delta.y as f64 * SPEED).clamp(-PITCH_LIMIT, PITCH_LIMIT);
    }

    fn pan(&mut self, delta: egui::Vec2) {
        let fwd = (self.target - self.eye()).normalize();
        let right = fwd.cross(glam::DVec3::Y).normalize();
        let up = right.cross(fwd).normalize();
        let scale = self.dist * 0.0015; // pan speed scales with zoom so it feels consistent
        self.target -= right * (delta.x as f64 * scale);
        self.target += up * (delta.y as f64 * scale);
    }

    fn zoom(&mut self, scroll_y: f32) {
        let factor = (-scroll_y as f64 * 0.001).exp();
        self.dist = (self.dist * factor).clamp(MIN_DIST, MAX_DIST);
    }
}

pub struct Viewport {
    pub camera: OrbitCamera,
}

impl Viewport {
    /// Registers the GPU pipelines/meshes once into egui_wgpu's shared
    /// callback_resources; must be called from `App::new` with the
    /// eframe `CreationContext`.
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let render_state = cc
            .wgpu_render_state
            .as_ref()
            .expect("kite-viz requires eframe's wgpu backend (NativeOptions::renderer = Wgpu)");
        let resources = GpuResources::new(&render_state.device, render_state.target_format);
        render_state.renderer.write().callback_resources.insert(resources);
        Self {
            camera: OrbitCamera::default(),
        }
    }

    pub fn show(&mut self, ui: &mut egui::Ui, scene: &SceneData) -> ViewportResponse {
        let rect = ui.available_rect_before_wrap();
        let response = ui.interact(rect, ui.id().with("kite-viewport"), egui::Sense::click_and_drag());
        let modifiers = ui.input(|i| i.modifiers);

        let delta = response.drag_delta();
        if response.dragged_by(egui::PointerButton::Middle)
            || (modifiers.shift && response.dragged_by(egui::PointerButton::Secondary))
        {
            self.camera.pan(delta);
        } else if response.dragged_by(egui::PointerButton::Secondary) {
            self.camera.orbit(delta);
        }
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                self.camera.zoom(scroll);
            }
        }

        let aspect = rect.width() / rect.height().max(1.0);
        let view_proj = self.camera.view_proj(aspect);
        let cb = FrameCallback::build(scene, view_proj);
        ui.painter().add(egui_wgpu::Callback::new_paint_callback(rect, cb));

        ViewportResponse {
            hovered: response.hovered(),
            pointer_ray: response.hover_pos().map(|p| self.camera.screen_ray(p, rect)),
            pointer_pos: response.hover_pos(),
            primary_clicked: response.clicked(),
            primary_down: ui.input(|i| i.pointer.primary_down()),
            primary_released: ui.input(|i| i.pointer.primary_released()),
            modifiers,
            view_proj,
            rect,
        }
    }
}
