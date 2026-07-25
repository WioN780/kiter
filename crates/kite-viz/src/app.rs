use eframe::egui;
use glam::{DQuat, DVec3};

use crate::doc::{valid_name, EditorDoc, Scenario};
use crate::panels;
use crate::render::{ArrowInst, CylinderInst, DynMesh, LineSeg, SceneData, SphereInst};
use crate::session::SimSession;
use crate::snapshot::Snapshot;
use crate::tools::{self, GizmoAxis, Selection, SelectionSet, Tool, ToolState};
use crate::viewport::{Ray, Viewport};
use crate::windviz::WindViz;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Edit,
    Simulate,
}

pub struct ViewToggles {
    pub draw_grid: bool,
    pub draw_axes: bool,
    /// Simulate-mode only: lift/drag arrows at panel centroids.
    pub show_aero: bool,
    /// Simulate-mode only: cloth edge force overlay lines.
    pub show_cloth_force: bool,
}

impl Default for ViewToggles {
    fn default() -> Self {
        Self { draw_grid: true, draw_axes: true, show_aero: false, show_cloth_force: false }
    }
}

/// What a `GizmoDrag` does with the per-frame ray: slide points along an
/// axis, or spin them around one. Fixed at press time from the Alt modifier
/// (mirrored by which handle — arrow vs ring — was actually picked), so
/// toggling Alt mid-drag can't switch a drag already in progress.
enum GizmoDragKind {
    /// Point on the axis line picked up at press time; the reference the
    /// per-frame delta is measured against.
    Translate { start_on_axis: DVec3 },
    /// Pivot and start angle (see `tools::ring_angle`) picked up at press
    /// time; the reference the per-frame swept angle is measured against.
    Rotate { center: DVec3, start_angle: f64 },
}

/// Live axis-constrained gizmo drag: the axis, what it does (see
/// `GizmoDragKind`), and every affected point's pre-drag position (also
/// doubles as the Escape-cancel restore set).
struct GizmoDrag {
    axis: GizmoAxis,
    kind: GizmoDragKind,
    starts: Vec<(usize, DVec3)>,
}

pub struct App {
    mode: Mode,
    viewport: Viewport,
    doc: EditorDoc,
    sel: SelectionSet,
    tool_state: ToolState,
    dragging: Option<usize>,
    gizmo_drag: Option<GizmoDrag>,
    /// Screen-space press position that started a rubber-band box select
    /// (press landed on empty space, no gizmo/point/segment/panel hit).
    /// `None` once released or cancelled.
    box_select_start: Option<egui::Pos2>,
    was_down: bool,
    view: ViewToggles,
    scenario_name: String,
    pending_delete: Option<usize>,
    status: String,
    /// Lazily created on first entry into Simulate mode; survives Simulate
    /// <-> Edit toggles so scrubbed history isn't lost by switching tabs.
    /// Dropped by New/Load, which invalidate any live world outright.
    session: Option<SimSession>,
    /// Wind arrow grid / streak particles overlay; survives mode switches
    /// and New/Load like `view` does (it's a viewer setting, not doc state).
    windviz: WindViz,
}

impl App {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        Self {
            mode: Mode::Edit,
            viewport: Viewport::new(cc),
            doc: EditorDoc::default(),
            sel: SelectionSet::default(),
            tool_state: ToolState::default(),
            dragging: None,
            gizmo_drag: None,
            box_select_start: None,
            was_down: false,
            view: ViewToggles::default(),
            scenario_name: "untitled".to_string(),
            pending_delete: None,
            status: String::new(),
            session: None,
            windviz: WindViz::new(),
        }
    }

    fn new_doc(&mut self) {
        self.doc = EditorDoc::default();
        self.sel = SelectionSet::default();
        self.tool_state = ToolState::default();
        self.gizmo_drag = None;
        self.box_select_start = None;
        self.scenario_name = self.doc.name.clone();
        self.status = "New scenario".to_string();
        self.session = None;
    }

    fn load(&mut self, name: &str) {
        let path = format!("scenarios/{name}.toml");
        match std::fs::read_to_string(&path).and_then(|s| {
            toml::from_str::<Scenario>(&s).map_err(|e| std::io::Error::other(e.to_string()))
        }) {
            Ok(scenario) => {
                self.doc = EditorDoc::from_scenario(&scenario);
                self.sel = SelectionSet::default();
                self.tool_state = ToolState::default();
                self.gizmo_drag = None;
                self.box_select_start = None;
                self.scenario_name = name.to_string();
                self.status = format!("Loaded {name}");
                self.session = None;
            }
            Err(e) => self.status = format!("Load failed: {e}"),
        }
    }

    fn save_as(&mut self, name: &str) {
        if !valid_name(name) {
            self.status = "Invalid scenario name".to_string();
            return;
        }
        let toml_str = match toml::to_string_pretty(&self.doc.to_scenario()) {
            Ok(s) => s,
            Err(e) => {
                self.status = format!("Serialize failed: {e}");
                return;
            }
        };
        let _ = std::fs::create_dir_all("scenarios");
        match std::fs::write(format!("scenarios/{name}.toml"), toml_str) {
            Ok(()) => {
                self.scenario_name = name.to_string();
                self.status = format!("Saved {name}");
            }
            Err(e) => self.status = format!("Save failed: {e}"),
        }
    }

    fn try_delete_selected(&mut self) {
        // Multiple elements: skip the single-point cascade-confirm dialog
        // (ponytail: minimal path — deleting several points and eyeballing
        // "does this look like a lot of stuff" is on the user; the dialog
        // only guards the easy-to-fat-finger single-point case).
        if self.sel.len() > 1 {
            let sels: Vec<Selection> = self.sel.iter().copied().collect();
            tools::delete_selected(&mut self.doc, &sels);
            self.sel.clear();
            self.tool_state.pending.clear();
            self.gizmo_drag = None;
            return;
        }
        match self.sel.single().unwrap_or(Selection::None) {
            Selection::Point(i) => {
                let (s, p, b) = self.doc.dependents_of_point(i);
                if s + p + b > 0 {
                    self.pending_delete = Some(i);
                } else {
                    self.doc.delete_point(i);
                    self.sel.clear();
                    // Point deletion reindexes every point above `i`; any
                    // in-progress multi-click tool selection (Spar/Bridle/
                    // Panel) is holding indices into the old numbering and
                    // must be dropped, not silently pointed at the wrong
                    // point (or an index made stale by the deletion).
                    self.tool_state.pending.clear();
                }
            }
            Selection::Spar(i) => {
                self.doc.delete_spar(i);
                self.sel.clear();
            }
            Selection::Panel(i) => {
                self.doc.delete_panel(i);
                self.sel.clear();
            }
            Selection::Bridle(i) => {
                self.doc.delete_bridle(i);
                self.sel.clear();
            }
            Selection::None => {}
        }
    }

    fn tool_button(&mut self, ui: &mut egui::Ui, tool: Tool, label: &str, hotkey: &str) {
        let resp = ui
            .selectable_label(self.tool_state.tool == tool, label)
            .on_hover_text(format!("Hotkey: {hotkey}"));
        if resp.clicked() {
            self.tool_state.set_tool(tool);
        }
    }

    fn handle_hotkeys(&mut self, ui: &mut egui::Ui) {
        // Don't steal keystrokes while a text field (name edit, etc) has focus.
        if ui.memory(|m| m.focused()).is_some() {
            return;
        }
        let (esc, q, p, s, f, b, j, n, k, l, del) = ui.input(|i| {
            (
                i.key_pressed(egui::Key::Escape),
                i.key_pressed(egui::Key::Q),
                i.key_pressed(egui::Key::P),
                i.key_pressed(egui::Key::S),
                i.key_pressed(egui::Key::F),
                i.key_pressed(egui::Key::B),
                i.key_pressed(egui::Key::J),
                i.key_pressed(egui::Key::N),
                i.key_pressed(egui::Key::K),
                i.key_pressed(egui::Key::L),
                i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::X),
            )
        });
        if esc {
            self.tool_state.pending.clear();
            if let Some(drag) = self.gizmo_drag.take() {
                for (i, orig) in drag.starts {
                    self.doc.points[i] = orig;
                }
            }
            self.box_select_start = None;
        }
        if q {
            self.tool_state.set_tool(Tool::Select);
        }
        if p {
            self.tool_state.set_tool(Tool::AddPoint);
        }
        if s {
            self.tool_state.set_tool(Tool::Spar);
        }
        if f {
            self.tool_state.set_tool(Tool::Panel);
        }
        if b {
            self.tool_state.set_tool(Tool::Bridle);
        }
        if j {
            self.tool_state.set_tool(Tool::Junction);
        }
        if n {
            self.tool_state.set_tool(Tool::Pin);
        }
        if k {
            self.tool_state.set_tool(Tool::StiffJoint);
        }
        if l {
            self.tool_state.set_tool(Tool::Lashing);
        }
        if del {
            self.try_delete_selected();
        }
    }

    fn delete_confirm_popup(&mut self, ctx: &egui::Context) {
        let Some(idx) = self.pending_delete else { return };
        let (s, p, b) = self.doc.dependents_of_point(idx);
        let mut close = false;
        egui::Window::new("Confirm delete")
            .collapsible(false)
            .resizable(false)
            .show(ctx, |ui| {
                ui.label(format!(
                    "Deleting this point also removes {s} spar(s), {p} panel(s), {b} bridle(s)."
                ));
                ui.horizontal(|ui| {
                    if ui.button("Delete").clicked() {
                        self.doc.delete_point(idx);
                        self.sel.clear();
                        self.tool_state.pending.clear();
                        close = true;
                    }
                    if ui.button("Cancel").clicked() {
                        close = true;
                    }
                });
            });
        if close {
            self.pending_delete = None;
        }
    }
}

fn build_scene(doc: &EditorDoc, sel: &SelectionSet, tool_state: &ToolState, view: &ViewToggles, eye: DVec3, rotate_gizmo: bool) -> SceneData {
    let mut scene = SceneData {
        draw_grid: view.draw_grid,
        draw_axes: view.draw_axes,
        ..Default::default()
    };

    for (i, &p) in doc.points.iter().enumerate() {
        let mut color = [0.8, 0.8, 0.85, 1.0];
        if doc.pinned.contains(&i) {
            color = [0.9, 0.2, 0.2, 1.0];
        }
        if doc.junction == Some(i) {
            color = [0.95, 0.6, 0.1, 1.0];
        }
        if doc.stiff_joints.iter().any(|sj| sj.point == i) {
            color = [0.7, 0.2, 0.9, 1.0];
        }
        if doc.lashings.iter().any(|la| la.point == i) {
            color = [1.0, 0.55, 0.0, 1.0];
        }
        if tool_state.pending.contains(&i) {
            color = [0.95, 0.95, 0.2, 1.0];
        }
        if sel.contains(Selection::Point(i)) {
            color = [0.2, 0.95, 0.95, 1.0];
        }
        scene.spheres.push(SphereInst { center: p, radius: 0.04, color });
    }

    for (i, s) in doc.spars.iter().enumerate() {
        let color = if sel.contains(Selection::Spar(i)) { [0.95, 0.85, 0.3, 1.0] } else { [0.75, 0.6, 0.35, 1.0] };
        scene.cylinders.push(CylinderInst {
            a: doc.points[s.a],
            b: doc.points[s.b],
            radius: s.radius.max(0.005),
            color,
        });
    }

    for (i, p) in doc.panels.iter().enumerate() {
        let color = if sel.contains(Selection::Panel(i)) { [0.55, 0.8, 1.0, 0.7] } else { [0.3, 0.55, 0.9, 0.4] };
        scene.meshes.push(DynMesh {
            positions: vec![doc.points[p.a], doc.points[p.b], doc.points[p.c]],
            indices: vec![0, 1, 2],
            color,
        });
    }

    for (i, b) in doc.bridles.iter().enumerate() {
        let color = if sel.contains(Selection::Bridle(i)) { [1.0, 1.0, 1.0, 1.0] } else { [0.8, 0.8, 0.8, 1.0] };
        scene.lines.push(LineSeg { a: doc.points[b.a], b: doc.points[b.b], color });
    }

    // Translate/rotate gizmo: three world-axis arrows (or, with Alt held,
    // rotation rings) anchored at the centroid of the selection, drawn last
    // so they land on top of the scene. Only in the Select tool — other
    // tools have their own click semantics and showing drag handles there
    // would be misleading.
    if tool_state.tool == Tool::Select && !sel.is_empty() {
        let pts = tools::gizmo_points(doc, sel.as_slice());
        if let Some(c) = tools::centroid(doc, &pts) {
            let len = tools::gizmo_arrow_len(c, eye);
            for axis in [GizmoAxis::X, GizmoAxis::Y, GizmoAxis::Z] {
                if rotate_gizmo {
                    let ring = tools::gizmo_ring_points(c, axis, len);
                    for seg in ring.windows(2) {
                        scene.lines.push(LineSeg { a: seg[0], b: seg[1], color: axis.color() });
                    }
                } else {
                    scene.arrows.push(ArrowInst {
                        origin: c,
                        vec: axis.dir() * len,
                        shaft_radius: len * 0.03,
                        color: axis.color(),
                    });
                }
            }
        }
    }

    scene
}

/// The frame Simulate-mode rendering/hover reads positions and forces from:
/// the live world normally, or a scrubbed history snapshot while scrubbing.
/// Topology (particle/constraint indices) never changes post-build, so
/// callers always iterate `session.world`'s constraint `Vec`s for indices
/// and only consult `SimView` for the per-frame values.
enum SimView<'a> {
    Live(&'a kite_core::World),
    Snap(&'a Snapshot),
}

fn sim_view(session: &SimSession) -> SimView<'_> {
    match session.scrub {
        Some(i) => SimView::Snap(&session.history[i]),
        None => SimView::Live(&session.world),
    }
}

impl SimView<'_> {
    fn pos(&self, i: usize) -> DVec3 {
        match self {
            SimView::Live(w) => w.particles.pos[i],
            SimView::Snap(s) => s.pos[i].as_dvec3(),
        }
    }
    fn spar_force(&self, i: usize) -> f64 {
        match self {
            SimView::Live(w) => {
                let c = &w.stretch_shear_constraints[i];
                if w.last_h > 0.0 { -c.lambda.z / (w.last_h * w.last_h) } else { 0.0 }
            }
            SimView::Snap(s) => s.spar_force[i] as f64,
        }
    }
    fn bridle_tension(&self, i: usize) -> f64 {
        match self {
            SimView::Live(w) => {
                let c = &w.unilateral_constraints[i];
                if w.last_h > 0.0 { -c.lambda / (w.last_h * w.last_h) } else { 0.0 }
            }
            SimView::Snap(s) => s.bridle_tension[i] as f64,
        }
    }
    fn cloth_force(&self, i: usize) -> f64 {
        match self {
            SimView::Live(w) => {
                let c = &w.distance_constraints[i];
                if w.last_h > 0.0 { -c.lambda / (w.last_h * w.last_h) } else { 0.0 }
            }
            SimView::Snap(s) => s.cloth_force[i] as f64,
        }
    }
    fn panel_aero(&self, i: usize) -> (DVec3, DVec3, f64) {
        match self {
            SimView::Live(w) => {
                let a = w.aero_readback.get(i).copied().unwrap_or_default();
                (a.lift, a.drag, a.alpha)
            }
            SimView::Snap(s) => {
                // ponytail: `panel_aero` is only populated once `step` runs
                // the aero solve, so the pre-step history[0] snapshot has it
                // empty even when the kite has panels — guard like the Live
                // branch instead of indexing straight in.
                let a = s.panel_aero.get(i).copied().unwrap_or_default();
                (a.lift.as_dvec3(), a.drag.as_dvec3(), a.alpha as f64)
            }
        }
    }
}

/// Diverging blue-white-red ramp for signed forces (compression/tension),
/// autoscaled by `max_abs` (the session's decaying running max).
fn diverging_color(value: f32, max_abs: f32) -> [f32; 4] {
    let max_abs = max_abs.max(1e-6);
    let t = (value / max_abs).clamp(-1.0, 1.0);
    if t >= 0.0 {
        [1.0, 1.0 - t, 1.0 - t, 1.0]
    } else {
        let s = -t;
        [1.0 - s, 1.0 - s, 1.0, 1.0]
    }
}

/// Gray when slack (tension <= 0), yellow-to-red sequential ramp when taut.
fn bridle_color(tension: f32, max_abs: f32) -> [f32; 4] {
    if tension <= 1e-6 {
        return [0.5, 0.5, 0.5, 1.0];
    }
    let t = (tension / max_abs.max(1e-6)).clamp(0.0, 1.0);
    [1.0, 1.0 - t, 0.0, 1.0]
}

fn build_sim_scene(session: &SimSession, view_toggles: &ViewToggles) -> SceneData {
    let mut scene = SceneData {
        draw_grid: view_toggles.draw_grid,
        draw_axes: view_toggles.draw_axes,
        ..Default::default()
    };
    let view = sim_view(session);
    let world = &session.world;

    for (i, c) in world.stretch_shear_constraints.iter().enumerate() {
        scene.cylinders.push(CylinderInst {
            a: view.pos(c.p1),
            b: view.pos(c.p2),
            radius: (c.diameter * 0.5).max(0.002),
            color: diverging_color(view.spar_force(i) as f32, session.color_scale.spar),
        });
    }

    for (i, c) in world.unilateral_constraints.iter().enumerate() {
        scene.lines.push(LineSeg {
            a: view.pos(c.p1),
            b: view.pos(c.p2),
            color: bridle_color(view.bridle_tension(i) as f32, session.color_scale.bridle),
        });
    }

    for p in &world.canopy_panels {
        scene.meshes.push(DynMesh {
            positions: vec![view.pos(p.p1), view.pos(p.p2), view.pos(p.p3)],
            indices: vec![0, 1, 2],
            color: [0.3, 0.55, 0.9, 0.55],
        });
    }

    if view_toggles.show_cloth_force {
        for (i, c) in world.distance_constraints.iter().enumerate() {
            scene.lines.push(LineSeg {
                a: view.pos(c.p1),
                b: view.pos(c.p2),
                color: diverging_color(view.cloth_force(i) as f32, session.color_scale.cloth),
            });
        }
    }

    if view_toggles.show_aero {
        // Autoscale arrow length so the largest current force reads as ~0.5m.
        let arrow_scale = 0.5 / session.color_scale.aero.max(1e-6) as f64;
        for (i, p) in world.canopy_panels.iter().enumerate() {
            let centroid = (view.pos(p.p1) + view.pos(p.p2) + view.pos(p.p3)) / 3.0;
            let (lift, drag, _alpha) = view.panel_aero(i);
            if lift.length_squared() > 1e-12 {
                scene.arrows.push(ArrowInst {
                    origin: centroid,
                    vec: lift * arrow_scale,
                    shaft_radius: 0.006,
                    color: [0.3, 0.9, 0.4, 1.0],
                });
            }
            if drag.length_squared() > 1e-12 {
                scene.arrows.push(ArrowInst {
                    origin: centroid,
                    vec: drag * arrow_scale,
                    shaft_radius: 0.006,
                    color: [0.9, 0.4, 0.3, 1.0],
                });
            }
        }
    }

    for i in 0..world.particles.len() {
        let grabbed = session.grab.as_ref().is_some_and(|g| g.particle == i);
        let pinned = world.particles.inv_mass[i] == 0.0;
        let color = if grabbed {
            [0.2, 0.95, 0.95, 1.0]
        } else if pinned {
            [0.9, 0.2, 0.2, 1.0]
        } else {
            [0.75, 0.75, 0.8, 1.0]
        };
        scene.spheres.push(SphereInst { center: view.pos(i), radius: 0.03, color });
    }

    // Event markers persist for the whole run regardless of scrub position.
    for m in &session.event_markers {
        let color = if m.broken { [1.0, 0.15, 0.15, 1.0] } else { [0.95, 0.85, 0.1, 1.0] };
        scene.spheres.push(SphereInst { center: view.pos(m.particle), radius: 0.05, color });
    }

    scene
}

/// Maps a global stretch-shear constraint index to a `"spar_name seg N"`
/// label, using `doc.spars`' `num_segments` in build order (matches how
/// `build_kite_from_def` lays out `stretch_shear_constraints`: spars in
/// order, `num_segments` consecutive constraints each).
fn spar_label(doc: &EditorDoc, global_idx: usize) -> String {
    let mut offset = 0usize;
    for s in &doc.spars {
        if global_idx < offset + s.num_segments {
            return format!("{} seg {}", s.name, global_idx - offset);
        }
        offset += s.num_segments;
    }
    format!("spar segment {global_idx}")
}

/// Hover probe: raycasts spars, bridles, then panels against the current
/// sim view and returns a one-line description of the closest hit.
/// `bridle i` / `panel i` map 1:1 onto `doc.bridles[i]` / `doc.panels[i]`
/// because `build_kite_from_def` pushes both in definition order.
fn sim_hover(doc: &EditorDoc, session: &SimSession, ray: &Ray, eye: DVec3) -> Option<String> {
    let world = &session.world;
    let view = sim_view(session);
    const RAY_LEN: f64 = 1000.0;
    let seg_end = ray.origin + ray.dir * RAY_LEN;
    let mut best: Option<(f64, String)> = None;

    for (i, c) in world.stretch_shear_constraints.iter().enumerate() {
        let a = view.pos(c.p1);
        let b = view.pos(c.p2);
        let (s, _t, dist) = tools::closest_segment_segment(ray.origin, seg_end, a, b);
        let radius = (0.02 * (a.lerp(b, 0.5) - eye).length()).max(0.03);
        if dist < radius {
            let t_ray = s * RAY_LEN;
            if best.as_ref().is_none_or(|(bt, _)| t_ray < *bt) {
                best = Some((t_ray, format!("{} — {:.1} N", spar_label(doc, i), view.spar_force(i))));
            }
        }
    }

    for (i, c) in world.unilateral_constraints.iter().enumerate() {
        let a = view.pos(c.p1);
        let b = view.pos(c.p2);
        let (s, _t, dist) = tools::closest_segment_segment(ray.origin, seg_end, a, b);
        let radius = (0.02 * (a.lerp(b, 0.5) - eye).length()).max(0.03);
        if dist < radius {
            let t_ray = s * RAY_LEN;
            if best.as_ref().is_none_or(|(bt, _)| t_ray < *bt) {
                let tension = view.bridle_tension(i);
                let name = doc.bridles.get(i).map(|b| b.name.as_str()).unwrap_or("bridle");
                let state = if tension <= 1e-6 { "slack" } else { "taut" };
                best = Some((t_ray, format!("{name} — {tension:.1} N ({state})")));
            }
        }
    }

    for (i, p) in world.canopy_panels.iter().enumerate() {
        let (a, b, c) = (view.pos(p.p1), view.pos(p.p2), view.pos(p.p3));
        if let Some(t) = tools::ray_triangle(ray, a, b, c) {
            if best.as_ref().is_none_or(|(bt, _)| t < *bt) {
                let (lift, _drag, alpha) = view.panel_aero(i);
                let name = doc.panels.get(i).map(|p| p.name.as_str()).unwrap_or("panel");
                best = Some((t, format!("{name} — α = {:.1}°, |L| = {:.1} N", alpha.to_degrees(), lift.length())));
            }
        }
    }

    best.map(|(_, s)| s)
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        ui.ctx().request_repaint();
        let frame_elapsed = ui.ctx().input(|i| i.stable_dt) as f64;

        if self.mode == Mode::Simulate {
            if let Some(session) = &mut self.session {
                session.tick(frame_elapsed);
            }
        }

        egui::Panel::top("toolbar").show(ui, |ui| {
            ui.horizontal(|ui| {
                if ui.selectable_label(self.mode == Mode::Edit, "Edit").clicked() {
                    self.mode = Mode::Edit;
                }
                if ui.selectable_label(self.mode == Mode::Simulate, "Simulate").clicked() {
                    self.mode = Mode::Simulate;
                    // First entry builds the live world; subsequent toggles
                    // back into Simulate reuse the existing session (and its
                    // history) — Apply & Restart is the explicit rebuild path.
                    if self.session.is_none() {
                        self.session = Some(SimSession::new(&self.doc));
                    }
                }
                ui.separator();

                ui.add_enabled_ui(self.mode == Mode::Edit, |ui| {
                    self.tool_button(ui, Tool::Select, "Select", "Q");
                    self.tool_button(ui, Tool::AddPoint, "Add Point", "P");
                    self.tool_button(ui, Tool::Spar, "Spar", "S");
                    self.tool_button(ui, Tool::Panel, "Panel", "F");
                    self.tool_button(ui, Tool::Bridle, "Bridle", "B");
                    self.tool_button(ui, Tool::Junction, "Junction", "J");
                    self.tool_button(ui, Tool::Pin, "Pin", "N");
                    self.tool_button(ui, Tool::StiffJoint, "Stiff Joint", "K");
                    self.tool_button(ui, Tool::Lashing, "Lashing", "L");
                    if ui.button("Delete").on_hover_text("Hotkey: Del / X").clicked() {
                        self.try_delete_selected();
                    }
                });

                ui.separator();
                ui.label("Scenario:");
                egui::ComboBox::from_id_salt("scenario-combo")
                    .selected_text(&self.scenario_name)
                    .show_ui(ui, |ui| {
                        for name in list_scenario_files() {
                            if ui.selectable_label(self.scenario_name == name, &name).clicked() {
                                self.scenario_name = name;
                            }
                        }
                    });
                ui.text_edit_singleline(&mut self.scenario_name);
                if ui.button("New").clicked() {
                    self.new_doc();
                }
                if ui.button("Load").clicked() {
                    self.load(&self.scenario_name.clone());
                }
                if ui.button("Save").clicked() {
                    self.save_as(&self.scenario_name.clone());
                }
                if !self.status.is_empty() {
                    ui.separator();
                    ui.label(&self.status);
                }
            });
        });

        egui::Panel::left("inspector").show(ui, |ui| match self.mode {
            Mode::Edit => panels::inspector(ui, &mut self.doc, &mut self.sel, &mut self.view, &mut self.windviz),
            Mode::Simulate => {
                if let Some(session) = &mut self.session {
                    panels::simulate_inspector(ui, &mut self.doc, session, &mut self.view, &mut self.windviz);
                } else {
                    ui.label("(enter Simulate mode to start a session)");
                }
            }
        });

        egui::Panel::bottom("timeline").show(ui, |ui| match self.mode {
            Mode::Edit => {
                ui.heading("Timeline");
                ui.label("(switch to Simulate to run the sim)");
            }
            Mode::Simulate => {
                if let Some(session) = &mut self.session {
                    panels::timeline(ui, session, &self.doc);
                }
            }
        });

        self.delete_confirm_popup(ui.ctx());

        egui::CentralPanel::default().show(ui, |ui| {
            if self.mode == Mode::Edit && self.pending_delete.is_none() {
                self.handle_hotkeys(ui);
            }

            let eye = self.viewport.camera.eye();
            let rotate_gizmo = ui.input(|i| i.modifiers.alt);
            let mut scene = match self.mode {
                Mode::Edit => build_scene(&self.doc, &self.sel, &self.tool_state, &self.view, eye, rotate_gizmo),
                Mode::Simulate => self
                    .session
                    .as_ref()
                    .map(|s| build_sim_scene(s, &self.view))
                    .unwrap_or_default(),
            };
            match self.mode {
                Mode::Edit => {
                    let t = self.windviz.tick(frame_elapsed, None, &self.doc.wind);
                    self.windviz.build_arrows(t, &self.doc.wind, &mut scene);
                    self.windviz.build_streaks(&mut scene);
                }
                Mode::Simulate => {
                    if let Some(session) = &self.session {
                        let sim_t = match session.scrub {
                            Some(i) => session.history[i].t,
                            None => session.world.time,
                        };
                        let cfg = &session.world.cfg.wind;
                        let t = self.windviz.tick(frame_elapsed, Some(sim_t), cfg);
                        self.windviz.build_arrows(t, cfg, &mut scene);
                        self.windviz.build_streaks(&mut scene);
                    }
                }
            }
            let resp = self.viewport.show(ui, &scene);

            // Rubber-band box-select overlay: drawn whenever a box drag is
            // in progress, on top of the viewport.
            if let (Some(start), Some(cur)) = (self.box_select_start, resp.pointer_pos) {
                let box_rect = egui::Rect::from_two_pos(start, cur);
                let painter = ui.painter_at(resp.rect);
                painter.rect_filled(box_rect, 0.0, egui::Color32::from_rgba_unmultiplied(90, 160, 255, 40));
                painter.rect_stroke(
                    box_rect,
                    0.0,
                    egui::Stroke::new(1.0, egui::Color32::from_rgb(90, 160, 255)),
                    egui::StrokeKind::Middle,
                );
            }

            if self.mode == Mode::Edit && self.pending_delete.is_none() {
                let down_now = resp.primary_down;
                if self.tool_state.tool == Tool::Select {
                    if down_now && !self.was_down {
                        // Gizmo axis takes priority over regular picking.
                        // Alt held picks a rotation ring instead of a
                        // translate arrow; which one was actually hit is
                        // baked into `gizmo_hit` so the drag can't change
                        // kind mid-flight if Alt is toggled afterward.
                        let gizmo_hit = (!self.sel.is_empty())
                            .then(|| tools::gizmo_points(&self.doc, self.sel.as_slice()))
                            .and_then(|pts| tools::centroid(&self.doc, &pts).map(|c| (pts, c)))
                            .zip(resp.pointer_ray.as_ref())
                            .and_then(|((pts, c), ray)| {
                                let len = tools::gizmo_arrow_len(c, eye);
                                let axis = if rotate_gizmo {
                                    tools::pick_gizmo_rotate(c, len, ray, eye)
                                } else {
                                    tools::pick_gizmo(c, len, ray, eye)
                                };
                                axis.map(|axis| (axis, c, pts))
                            });

                        if let (Some((axis, centroid, pts)), Some(ray)) = (gizmo_hit, &resp.pointer_ray) {
                            let starts = pts.iter().map(|&i| (i, self.doc.points[i])).collect();
                            let kind = if rotate_gizmo {
                                let hit = tools::ray_plane(ray, centroid, axis.dir()).unwrap_or(centroid);
                                GizmoDragKind::Rotate { center: centroid, start_angle: tools::ring_angle(centroid, axis, hit) }
                            } else {
                                GizmoDragKind::Translate { start_on_axis: tools::closest_point_on_axis(ray, centroid, axis.dir()) }
                            };
                            self.gizmo_drag = Some(GizmoDrag { axis, kind, starts });
                            self.dragging = None;
                            self.box_select_start = None;
                        } else if let Some(ray) = &resp.pointer_ray {
                            self.gizmo_drag = None;
                            let picked = tools::pick(&self.doc, ray, eye);
                            if picked == Selection::None {
                                // Empty press: might be a plain click (clear,
                                // decided on release if the drag stays tiny)
                                // or the start of a rubber-band box select.
                                self.box_select_start = resp.pointer_pos;
                                self.dragging = None;
                            } else {
                                self.box_select_start = None;
                                if resp.modifiers.command {
                                    self.sel.remove(picked);
                                    self.dragging = None;
                                } else {
                                    if resp.modifiers.shift {
                                        self.sel.add(picked);
                                    } else {
                                        self.sel.set(picked);
                                    }
                                    self.dragging = match picked {
                                        Selection::Point(i) => Some(i),
                                        _ => None,
                                    };
                                }
                            }
                        }
                    } else if down_now {
                        if let Some(ray) = &resp.pointer_ray {
                            if let Some(drag) = &self.gizmo_drag {
                                match drag.kind {
                                    GizmoDragKind::Translate { start_on_axis } => {
                                        let cur = tools::closest_point_on_axis(ray, start_on_axis, drag.axis.dir());
                                        let mut t = (cur - start_on_axis).dot(drag.axis.dir());
                                        if resp.modifiers.shift {
                                            t = (t / 0.01).round() * 0.01;
                                        }
                                        let delta = drag.axis.dir() * t;
                                        for &(i, orig) in &drag.starts {
                                            self.doc.points[i] = orig + delta;
                                        }
                                    }
                                    GizmoDragKind::Rotate { center, start_angle } => {
                                        if let Some(hit) = tools::ray_plane(ray, center, drag.axis.dir()) {
                                            let mut delta = tools::ring_angle(center, drag.axis, hit) - start_angle;
                                            if resp.modifiers.shift {
                                                let step = 5.0_f64.to_radians();
                                                delta = (delta / step).round() * step;
                                            }
                                            let rot = DQuat::from_axis_angle(drag.axis.dir(), delta);
                                            for &(i, orig) in &drag.starts {
                                                self.doc.points[i] = center + rot * (orig - center);
                                            }
                                        }
                                    }
                                }
                            } else if let Some(i) = self.dragging {
                                let p = self.doc.points[i];
                                let to_eye = eye - p;
                                let normal = if to_eye.length_squared() > 1e-9 {
                                    to_eye.normalize()
                                } else {
                                    glam::DVec3::Y
                                };
                                if let Some(mut hit) = tools::ray_plane(ray, p, normal) {
                                    if resp.modifiers.shift {
                                        hit = tools::snap(hit, 0.01);
                                    }
                                    self.doc.points[i] = hit;
                                }
                            }
                        }
                        // else: box_select_start (if any) just stays put; the
                        // overlay above already tracks the live cursor.
                    }
                    if resp.primary_released {
                        if let Some(start) = self.box_select_start.take() {
                            if let Some(end) = resp.pointer_pos {
                                const CLICK_TOLERANCE: f32 = 4.0;
                                if start.distance(end) >= CLICK_TOLERANCE {
                                    let box_rect = egui::Rect::from_two_pos(start, end);
                                    let hits = tools::points_in_rect(&self.doc, resp.view_proj, resp.rect, box_rect);
                                    if resp.modifiers.command {
                                        for i in hits {
                                            self.sel.remove(Selection::Point(i));
                                        }
                                    } else {
                                        if !resp.modifiers.shift {
                                            self.sel.clear();
                                        }
                                        for i in hits {
                                            self.sel.add(Selection::Point(i));
                                        }
                                    }
                                } else if !resp.modifiers.shift && !resp.modifiers.command {
                                    // Tiny drag on empty space with no
                                    // modifiers: a normal empty click.
                                    self.sel.clear();
                                }
                            }
                        }
                        self.dragging = None;
                        self.gizmo_drag = None;
                    }
                } else if resp.primary_clicked {
                    if let Some(ray) = &resp.pointer_ray {
                        if let Some(status) =
                            tools::handle_click(&mut self.doc, &mut self.tool_state, &mut self.sel, ray, eye)
                        {
                            self.status = status;
                        }
                    }
                }
                self.was_down = down_now;
            } else if self.mode == Mode::Simulate {
                if let Some(session) = &mut self.session {
                    let down_now = resp.primary_down;

                    // Grab tool: Select-click-drag a live particle. Disabled
                    // while scrubbing history (nothing "live" to grab) or
                    // after divergence (positions are NaN).
                    if session.scrub.is_none() && session.diverged.is_none() {
                        if down_now && !self.was_down {
                            if let Some(ray) = &resp.pointer_ray {
                                if let Some(i) = tools::pick_point(&session.world.particles.pos, ray, eye) {
                                    session.try_grab(i);
                                }
                            }
                        } else if down_now {
                            if let (Some(g), Some(ray)) = (&session.grab, &resp.pointer_ray) {
                                let p = session.world.particles.pos[g.particle];
                                let to_eye = eye - p;
                                let normal = if to_eye.length_squared() > 1e-9 { to_eye.normalize() } else { DVec3::Y };
                                if let Some(hit) = tools::ray_plane(ray, p, normal) {
                                    session.drag_grab(hit);
                                }
                            }
                        }
                        if resp.primary_released {
                            session.release_grab();
                        }
                    }
                    self.was_down = down_now;

                    if let Some(ray) = &resp.pointer_ray {
                        if let Some(text) = sim_hover(&self.doc, session, ray, eye) {
                            if let Some(pos) = ui.ctx().input(|i| i.pointer.hover_pos()) {
                                egui::Area::new(egui::Id::new("sim-hover-tooltip"))
                                    .fixed_pos(pos + egui::vec2(16.0, 16.0))
                                    .order(egui::Order::Tooltip)
                                    .interactable(false)
                                    .show(ui.ctx(), |ui| {
                                        egui::Frame::popup(ui.style()).show(ui, |ui| {
                                            ui.label(text);
                                        });
                                    });
                            }
                        }
                    }
                }
            }
        });
    }
}

fn list_scenario_files() -> Vec<String> {
    let mut v = Vec::new();
    if let Ok(rd) = std::fs::read_dir("scenarios") {
        for entry in rd.flatten() {
            let path = entry.path();
            if path.extension().and_then(|e| e.to_str()) == Some("toml") {
                if let Some(stem) = path.file_stem().and_then(|s| s.to_str()) {
                    v.push(stem.to_string());
                }
            }
        }
    }
    v.sort();
    v
}
