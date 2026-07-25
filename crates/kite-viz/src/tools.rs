//! Edit-mode tools: picking (CPU ray tests against the doc's geometry), the
//! multi-click tool state machine, and the mutations each tool performs.
//! Camera dragging/orbiting stays inside `Viewport`; this module only
//! consumes the `Ray` it hands back.

use eframe::egui;
use glam::DVec3;

use crate::doc::EditorDoc;
use crate::viewport::Ray;

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Tool {
    #[default]
    Select,
    AddPoint,
    Spar,
    Panel,
    Bridle,
    Junction,
    Pin,
    StiffJoint,
    Lashing,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Selection {
    #[default]
    None,
    Point(usize),
    Spar(usize),
    Panel(usize),
    Bridle(usize),
}

/// Multi-click tool progress (e.g. two points picked so far for Spar).
#[derive(Default)]
pub struct ToolState {
    pub tool: Tool,
    pub pending: Vec<usize>,
}

impl ToolState {
    pub fn set_tool(&mut self, tool: Tool) {
        self.tool = tool;
        self.pending.clear();
    }
}

/// Blender-style multi-selection: an ordered, deduplicated set of picked
/// elements. Plain click replaces the whole set; shift+click toggles one
/// member in/out; clicking empty space clears it (via `set(Selection::None)`).
#[derive(Default, Clone)]
pub struct SelectionSet(Vec<Selection>);

impl SelectionSet {
    pub fn iter(&self) -> impl Iterator<Item = &Selection> {
        self.0.iter()
    }
    pub fn as_slice(&self) -> &[Selection] {
        &self.0
    }
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
    pub fn len(&self) -> usize {
        self.0.len()
    }
    pub fn contains(&self, s: Selection) -> bool {
        s != Selection::None && self.0.contains(&s)
    }
    pub fn clear(&mut self) {
        self.0.clear();
    }
    /// Replaces the whole set with `s` (or clears it, for `Selection::None`).
    pub fn set(&mut self, s: Selection) {
        self.0.clear();
        if s != Selection::None {
            self.0.push(s);
        }
    }
    /// Toggles `s` in/out of the set; a no-op for `Selection::None` (so
    /// shift-clicking empty space leaves the current selection alone).
    pub fn toggle(&mut self, s: Selection) {
        if s == Selection::None {
            return;
        }
        match self.0.iter().position(|&x| x == s) {
            Some(pos) => {
                self.0.remove(pos);
            }
            None => self.0.push(s),
        }
    }
    /// Removes `s` from the set if present (Ctrl-click / Ctrl-drag subtract);
    /// a no-op if it isn't a member (including `Selection::None`, which is
    /// never inserted in the first place).
    pub fn remove(&mut self, s: Selection) {
        if let Some(pos) = self.0.iter().position(|&x| x == s) {
            self.0.remove(pos);
        }
    }
    /// Adds `s` to the set if it isn't already present (Shift-click / Shift
    /// box-select "add" semantics applied to a single element).
    pub fn add(&mut self, s: Selection) {
        if s != Selection::None && !self.0.contains(&s) {
            self.0.push(s);
        }
    }
    /// `Some` only when exactly one element is selected, drives the
    /// full-inspector-vs-"N selected" split in the inspector panel.
    pub fn single(&self) -> Option<Selection> {
        (self.0.len() == 1).then(|| self.0[0])
    }
}

/// The translate gizmo's three world-space axes.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum GizmoAxis {
    X,
    Y,
    Z,
}

impl GizmoAxis {
    pub fn dir(self) -> DVec3 {
        match self {
            GizmoAxis::X => DVec3::X,
            GizmoAxis::Y => DVec3::Y,
            GizmoAxis::Z => DVec3::Z,
        }
    }
    /// Matches the existing world-axes overlay colors in `render.rs`.
    pub fn color(self) -> [f32; 4] {
        match self {
            GizmoAxis::X => [0.9, 0.15, 0.15, 1.0],
            GizmoAxis::Y => [0.15, 0.85, 0.15, 1.0],
            GizmoAxis::Z => [0.2, 0.4, 0.95, 1.0],
        }
    }
}

/// Apparent-size-constant gizmo arrow length, echoing the `0.03 * dist`
/// pick-radius heuristic used elsewhere in this module.
pub fn gizmo_arrow_len(centroid: DVec3, eye: DVec3) -> f64 {
    (0.15 * (centroid - eye).length()).max(0.15)
}

/// Point indices the gizmo translates: directly-selected points plus the
/// endpoint/corner points of every selected spar/panel/bridle.
pub fn gizmo_points(doc: &EditorDoc, sels: &[Selection]) -> Vec<usize> {
    let mut set = std::collections::BTreeSet::new();
    for s in sels {
        match *s {
            Selection::Point(i) => {
                set.insert(i);
            }
            Selection::Spar(i) => {
                if let Some(sp) = doc.spars.get(i) {
                    set.insert(sp.a);
                    set.insert(sp.b);
                }
            }
            Selection::Panel(i) => {
                if let Some(p) = doc.panels.get(i) {
                    set.insert(p.a);
                    set.insert(p.b);
                    set.insert(p.c);
                }
            }
            Selection::Bridle(i) => {
                if let Some(b) = doc.bridles.get(i) {
                    set.insert(b.a);
                    set.insert(b.b);
                }
            }
            Selection::None => {}
        }
    }
    set.into_iter().collect()
}

/// Centroid of `points`' current positions; `None` if empty.
pub fn centroid(doc: &EditorDoc, points: &[usize]) -> Option<DVec3> {
    if points.is_empty() {
        return None;
    }
    let sum = points.iter().fold(DVec3::ZERO, |acc, &i| acc + doc.points[i]);
    Some(sum / points.len() as f64)
}

/// The two unit vectors spanning the plane perpendicular to `axis`, ordered
/// so `axis.dir() == u.cross(v)`, i.e. rotating a point at `u` by +90°
/// about `axis` (right-hand rule) lands it at `v`. Used both to draw each
/// rotation ring and to measure the drag angle around it.
fn ring_basis(axis: GizmoAxis) -> (DVec3, DVec3) {
    match axis {
        GizmoAxis::X => (DVec3::Y, DVec3::Z),
        GizmoAxis::Y => (DVec3::Z, DVec3::X),
        GizmoAxis::Z => (DVec3::X, DVec3::Y),
    }
}

/// World-space points of the rotation ring for `axis`, centered at `center`
/// with radius `radius`, a plain polyline loop, drawn via `SceneData::lines`
/// like any other segment (no dedicated ring primitive needed).
pub fn gizmo_ring_points(center: DVec3, axis: GizmoAxis, radius: f64) -> Vec<DVec3> {
    const SEGMENTS: usize = 48;
    let (u, v) = ring_basis(axis);
    (0..=SEGMENTS)
        .map(|i| {
            let a = (i as f64 / SEGMENTS as f64) * std::f64::consts::TAU;
            center + (u * a.cos() + v * a.sin()) * radius
        })
        .collect()
}

/// Angle (radians) of world point `p`, projected onto the plane through
/// `center` perpendicular to `axis`, measured from `ring_basis(axis).0`
/// toward `.1`. Two calls' difference is the signed rotation swept between
/// them, that's all the rotate-drag needs.
pub fn ring_angle(center: DVec3, axis: GizmoAxis, p: DVec3) -> f64 {
    let (u, v) = ring_basis(axis);
    let rel = p - center;
    rel.dot(v).atan2(rel.dot(u))
}

/// Picks the nearest rotation ring hit by `ray`: intersects `ray` with each
/// axis's ring plane through `center` and checks whether the hit lands close
/// to the ring's `radius` (same generous tolerance as `pick_gizmo`'s arrows).
pub fn pick_gizmo_rotate(center: DVec3, radius: f64, ray: &Ray, eye: DVec3) -> Option<GizmoAxis> {
    let tol = (0.05 * (center - eye).length()).max(0.05);
    let mut best: Option<(f64, GizmoAxis)> = None;
    for axis in [GizmoAxis::X, GizmoAxis::Y, GizmoAxis::Z] {
        if let Some(hit) = ray_plane(ray, center, axis.dir()) {
            if ((hit - center).length() - radius).abs() < tol {
                let t = (hit - ray.origin).length();
                if best.is_none_or(|(bt, _)| t < bt) {
                    best = Some((t, axis));
                }
            }
        }
    }
    best.map(|(_, a)| a)
}

/// Picks the nearest gizmo axis hit by `ray`, testing the three arrow
/// segments from `centroid` out to `len` with a generous radius (grabbing
/// an axis handle should be forgiving compared to point-picking).
pub fn pick_gizmo(centroid: DVec3, len: f64, ray: &Ray, eye: DVec3) -> Option<GizmoAxis> {
    const RAY_LEN: f64 = 1000.0;
    let seg_end = ray.origin + ray.dir * RAY_LEN;
    let radius = (0.05 * (centroid - eye).length()).max(0.05);
    let mut best: Option<(f64, GizmoAxis)> = None;
    for axis in [GizmoAxis::X, GizmoAxis::Y, GizmoAxis::Z] {
        let tip = centroid + axis.dir() * len;
        let (s, _t, dist) = closest_segment_segment(ray.origin, seg_end, centroid, tip);
        if dist < radius {
            let t_ray = s * RAY_LEN;
            if best.is_none_or(|(bt, _)| t_ray < bt) {
                best = Some((t_ray, axis));
            }
        }
    }
    best.map(|(_, a)| a)
}

/// Closest point on the infinite line through `anchor` along unit `axis_dir`
/// to `ray`. The gizmo's axis-constrained drag projects the mouse ray onto
/// this line every frame. Approximates "infinite line" as a very long
/// segment and reuses `closest_segment_segment` rather than deriving a
/// dedicated ray/line formula.
pub fn closest_point_on_axis(ray: &Ray, anchor: DVec3, axis_dir: DVec3) -> DVec3 {
    const HALF_LEN: f64 = 1.0e4;
    const RAY_LEN: f64 = 1.0e4;
    let a = anchor - axis_dir * HALF_LEN;
    let b = anchor + axis_dir * HALF_LEN;
    let seg_end = ray.origin + ray.dir * RAY_LEN;
    let (_s, t, _dist) = closest_segment_segment(ray.origin, seg_end, a, b);
    a.lerp(b, t)
}

/// Projects a world point to screen space via `view_proj`/`rect` (the same
/// matrix `Viewport` builds its rays from). `None` if the point is behind
/// the camera, mirroring the "skip points behind the camera" rubber-band
/// box-select rule.
pub fn project_to_screen(view_proj: glam::Mat4, rect: egui::Rect, p: DVec3) -> Option<egui::Pos2> {
    let clip = view_proj * p.as_vec3().extend(1.0);
    if clip.w <= 1.0e-6 {
        return None;
    }
    let ndc = clip.truncate() / clip.w;
    let x = rect.left() + (ndc.x * 0.5 + 0.5) * rect.width();
    let y = rect.top() + (1.0 - (ndc.y * 0.5 + 0.5)) * rect.height();
    Some(egui::pos2(x, y))
}

/// Every point index whose screen projection falls inside `rect` (a
/// rubber-band box-select rectangle in the same screen space as
/// `project_to_screen`'s output).
pub fn points_in_rect(doc: &EditorDoc, view_proj: glam::Mat4, viewport_rect: egui::Rect, box_rect: egui::Rect) -> Vec<usize> {
    doc.points
        .iter()
        .enumerate()
        .filter_map(|(i, &p)| {
            let s = project_to_screen(view_proj, viewport_rect, p)?;
            box_rect.contains(s).then_some(i)
        })
        .collect()
}

/// Deletes every selected element. Non-point types are removed by index
/// (descending, per type) *before* any points, so a point's cascade delete,
/// which may re-remove an element that was also directly selected, never
/// has to chase a stale index: by the time points are deleted, every
/// directly-selected spar/panel/bridle is already gone and the cascading
/// `retain` in `delete_point` is simply a no-op for those.
pub fn delete_selected(doc: &mut EditorDoc, sels: &[Selection]) {
    let mut spars = Vec::new();
    let mut panels = Vec::new();
    let mut bridles = Vec::new();
    let mut points = Vec::new();
    for s in sels {
        match *s {
            Selection::Spar(i) => spars.push(i),
            Selection::Panel(i) => panels.push(i),
            Selection::Bridle(i) => bridles.push(i),
            Selection::Point(i) => points.push(i),
            Selection::None => {}
        }
    }
    for v in [&mut spars, &mut panels, &mut bridles, &mut points] {
        v.sort_unstable_by(|a, b| b.cmp(a));
        v.dedup();
    }
    for i in spars {
        doc.delete_spar(i);
    }
    for i in panels {
        doc.delete_panel(i);
    }
    for i in bridles {
        doc.delete_bridle(i);
    }
    for i in points {
        doc.delete_point(i);
    }
}

/// Intersects `ray` with the plane through `plane_point` with normal
/// `plane_normal`; `None` if parallel or behind the ray origin.
pub fn ray_plane(ray: &Ray, plane_point: DVec3, plane_normal: DVec3) -> Option<DVec3> {
    let denom = ray.dir.dot(plane_normal);
    if denom.abs() < 1e-9 {
        return None;
    }
    let t = (plane_point - ray.origin).dot(plane_normal) / denom;
    if t < 0.0 {
        return None;
    }
    Some(ray.origin + ray.dir * t)
}

/// Snaps each component to the nearest multiple of `step`.
pub fn snap(p: DVec3, step: f64) -> DVec3 {
    (p / step).round() * step
}

/// Closest points between segments `p1..q1` and `p2..q2` (Ericson,
/// *Real-Time Collision Detection*, `ClosestPtSegmentSegment`). Returns
/// `(s, t, distance)` where `s`/`t` in `[0, 1]` locate the closest points.
/// `pub(crate)`: also used by Simulate mode's hover probe (`app::sim_hover`),
/// which picks against live/snapshotted world positions instead of the doc.
pub(crate) fn closest_segment_segment(p1: DVec3, q1: DVec3, p2: DVec3, q2: DVec3) -> (f64, f64, f64) {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);

    let (s, t) = if a <= 1e-12 && e <= 1e-12 {
        (0.0, 0.0)
    } else if a <= 1e-12 {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = d1.dot(r);
        if e <= 1e-12 {
            (( -c / a).clamp(0.0, 1.0), 0.0)
        } else {
            let b = d1.dot(d2);
            let denom = a * e - b * b;
            let mut s = if denom.abs() > 1e-12 {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let mut t = (b * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }
            (s, t)
        }
    };
    let c1 = p1 + d1 * s;
    let c2 = p2 + d2 * t;
    (s, t, (c1 - c2).length())
}

/// Möller-Trumbore ray-triangle intersection; double-sided (matches the
/// no-backface-culling mesh pipeline). Returns the hit distance along `ray`.
/// `pub(crate)`: also used by Simulate mode's hover probe.
pub(crate) fn ray_triangle(ray: &Ray, a: DVec3, b: DVec3, c: DVec3) -> Option<f64> {
    const EPS: f64 = 1e-9;
    let e1 = b - a;
    let e2 = c - a;
    let h = ray.dir.cross(e2);
    let det = e1.dot(h);
    if det.abs() < EPS {
        return None;
    }
    let inv_det = 1.0 / det;
    let s = ray.origin - a;
    let u = s.dot(h) * inv_det;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = s.cross(e1);
    let v = ray.dir.dot(q) * inv_det;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = e2.dot(q) * inv_det;
    (t > EPS).then_some(t)
}

/// Picking priority: points, then spar/bridle segments, then panels.
/// `eye` scales hit radii so picking feels consistent at any zoom level.
pub fn pick(doc: &EditorDoc, ray: &Ray, eye: DVec3) -> Selection {
    let mut best_point: Option<(f64, usize)> = None;
    for (i, &p) in doc.points.iter().enumerate() {
        let radius = (0.03 * (p - eye).length()).max(0.03);
        let t = (p - ray.origin).dot(ray.dir);
        if t < 0.0 {
            continue;
        }
        let closest = ray.origin + ray.dir * t;
        if (closest - p).length() < radius && best_point.is_none_or(|(bt, _)| t < bt) {
            best_point = Some((t, i));
        }
    }
    if let Some((_, i)) = best_point {
        return Selection::Point(i);
    }

    const RAY_LEN: f64 = 1000.0;
    let seg_end = ray.origin + ray.dir * RAY_LEN;
    let mut best_seg: Option<(f64, Selection)> = None;
    let test_segment = |a: DVec3, b: DVec3, sel: Selection, best_seg: &mut Option<(f64, Selection)>| {
        let (s, _t, dist) = closest_segment_segment(ray.origin, seg_end, a, b);
        let radius = (0.02 * (a.lerp(b, 0.5) - eye).length()).max(0.03);
        if dist < radius {
            let t_ray = s * RAY_LEN;
            if best_seg.is_none_or(|(bt, _)| t_ray < bt) {
                *best_seg = Some((t_ray, sel));
            }
        }
    };
    for (i, sp) in doc.spars.iter().enumerate() {
        test_segment(doc.points[sp.a], doc.points[sp.b], Selection::Spar(i), &mut best_seg);
    }
    for (i, br) in doc.bridles.iter().enumerate() {
        test_segment(doc.points[br.a], doc.points[br.b], Selection::Bridle(i), &mut best_seg);
    }
    if let Some((_, sel)) = best_seg {
        return sel;
    }

    let mut best_panel: Option<(f64, usize)> = None;
    for (i, p) in doc.panels.iter().enumerate() {
        if let Some(t) = ray_triangle(ray, doc.points[p.a], doc.points[p.b], doc.points[p.c]) {
            if best_panel.is_none_or(|(bt, _)| t < bt) {
                best_panel = Some((t, i));
            }
        }
    }
    if let Some((_, i)) = best_panel {
        return Selection::Panel(i);
    }

    Selection::None
}

/// Picks the nearest point in `positions` hit by `ray`, using the same
/// point-picking radius heuristic as `pick`. Used by Simulate mode's grab
/// tool, which picks against live/snapshotted particle positions rather
/// than `EditorDoc::points`.
pub fn pick_point(positions: &[DVec3], ray: &Ray, eye: DVec3) -> Option<usize> {
    let mut best: Option<(f64, usize)> = None;
    for (i, &p) in positions.iter().enumerate() {
        let radius = (0.03 * (p - eye).length()).max(0.03);
        let t = (p - ray.origin).dot(ray.dir);
        if t < 0.0 {
            continue;
        }
        let closest = ray.origin + ray.dir * t;
        if (closest - p).length() < radius && best.is_none_or(|(bt, _)| t < bt) {
            best = Some((t, i));
        }
    }
    best.map(|(_, i)| i)
}

/// Picks the nearest bridle interior "knot" node hit by `ray`: a virtual
/// point at `k/num_segments` straight-line fraction between a chained
/// bridle's two endpoints (only bridles with `num_segments >= 2` have any).
/// Mirrors `pick`'s point radius/priority test, just against these computed
/// positions instead of `doc.points`. Used by the Bridle tool to snap a new
/// endpoint onto an existing bridle's interior node (a bridle-to-bridle
/// knot) when the click misses every real point.
pub fn pick_bridle_node(doc: &EditorDoc, ray: &Ray, eye: DVec3) -> Option<(usize, usize, usize)> {
    let mut best: Option<(f64, usize, usize, usize)> = None;
    for (bi, b) in doc.bridles.iter().enumerate() {
        let n = b.num_segments;
        if n < 2 {
            continue;
        }
        let from = doc.points[b.a];
        let to = doc.points[b.b];
        for k in 1..n {
            let p = from.lerp(to, k as f64 / n as f64);
            let radius = (0.03 * (p - eye).length()).max(0.03);
            let t = (p - ray.origin).dot(ray.dir);
            if t < 0.0 {
                continue;
            }
            let closest = ray.origin + ray.dir * t;
            if (closest - p).length() < radius && best.is_none_or(|(bt, ..)| t < bt) {
                best = Some((t, bi, k, n));
            }
        }
    }
    best.map(|(_, bi, k, n)| (bi, k, n))
}

/// Dispatches a click for the non-Select tools (Select's click+drag is
/// handled directly in `App` since it needs press/release edges). Returns a
/// status-line message when the click did something worth calling out beyond
/// the obvious (currently: bridle-to-bridle knot snapping).
pub fn handle_click(doc: &mut EditorDoc, state: &mut ToolState, sel: &mut SelectionSet, ray: &Ray, eye: DVec3) -> Option<String> {
    match state.tool {
        Tool::Select => None,
        Tool::AddPoint => {
            if let Some(p) = ray_plane(ray, DVec3::ZERO, DVec3::Y) {
                let i = doc.weld_point(p);
                sel.set(Selection::Point(i));
            }
            None
        }
        Tool::Spar | Tool::Bridle | Tool::Panel => {
            let (i, status) = match pick(doc, ray, eye) {
                Selection::Point(i) => (i, None),
                _ if state.tool == Tool::Bridle => match pick_bridle_node(doc, ray, eye) {
                    Some((bi, k, n)) => {
                        let b = &doc.bridles[bi];
                        let pos = doc.points[b.a].lerp(doc.points[b.b], k as f64 / n as f64);
                        let name = b.name.clone();
                        let idx = doc.weld_point(pos);
                        (idx, Some(format!("snapped to node {k}/{n} of {name}")))
                    }
                    None => return None,
                },
                _ => return None,
            };
            if state.pending.last() != Some(&i) {
                state.pending.push(i);
            }
            let need = if state.tool == Tool::Panel { 3 } else { 2 };
            if state.pending.len() >= need {
                let pts: Vec<usize> = state.pending.drain(..).collect();
                sel.set(match state.tool {
                    Tool::Spar => Selection::Spar(doc.add_spar(pts[0], pts[1])),
                    Tool::Bridle => Selection::Bridle(doc.add_bridle(pts[0], pts[1])),
                    Tool::Panel => Selection::Panel(doc.add_panel(pts[0], pts[1], pts[2])),
                    _ => unreachable!(),
                });
            }
            status
        }
        Tool::Junction => {
            if let Selection::Point(i) = pick(doc, ray, eye) {
                doc.junction = Some(i);
                sel.set(Selection::Point(i));
            }
            None
        }
        Tool::Pin => {
            if let Selection::Point(i) = pick(doc, ray, eye) {
                if !doc.pinned.insert(i) {
                    doc.pinned.remove(&i);
                }
                sel.set(Selection::Point(i));
            }
            None
        }
        Tool::StiffJoint => {
            if let Selection::Point(i) = pick(doc, ray, eye) {
                doc.toggle_stiff_joint(i);
                sel.set(Selection::Point(i));
            }
            None
        }
        Tool::Lashing => {
            if let Selection::Point(i) = pick(doc, ray, eye) {
                doc.toggle_lashing(i);
                sel.set(Selection::Point(i));
            }
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn axis_drag_projects_ray_onto_axis_line() {
        // Ray travels straight down -Z at (x=3, y=0); the closest point on
        // the infinite X axis (y=0, z=0) is exactly (3, 0, 0).
        let ray = Ray {
            origin: DVec3::new(3.0, 0.0, 5.0),
            dir: DVec3::new(0.0, 0.0, -1.0),
        };
        let hit = closest_point_on_axis(&ray, DVec3::ZERO, DVec3::X);
        assert!((hit - DVec3::new(3.0, 0.0, 0.0)).length() < 1e-6, "hit = {hit:?}");
    }

    #[test]
    fn ring_angle_and_pick_agree_on_rotation_sense() {
        // Z-axis ring: basis is (X, Y), so a point straight out along +X
        // reads as angle 0 and +Y as +90°, a +90° rotation about +Z takes
        // +X to +Y, matching DQuat::from_axis_angle's right-hand convention.
        let center = DVec3::ZERO;
        let a0 = ring_angle(center, GizmoAxis::Z, DVec3::X);
        let a90 = ring_angle(center, GizmoAxis::Z, DVec3::Y);
        assert!(a0.abs() < 1e-9, "a0 = {a0}");
        assert!((a90 - std::f64::consts::FRAC_PI_2).abs() < 1e-9, "a90 = {a90}");

        // A ray straight down (-Z) through (1, 0, 5) crosses the Z-axis ring
        // (radius 1, centered at origin) exactly on its edge.
        let ray = Ray { origin: DVec3::new(1.0, 0.0, 5.0), dir: DVec3::new(0.0, 0.0, -1.0) };
        let eye = DVec3::new(1.0, 0.0, 5.0);
        assert_eq!(pick_gizmo_rotate(center, 1.0, &ray, eye), Some(GizmoAxis::Z));
    }

    #[test]
    fn selection_set_replace_add_remove() {
        let mut sel = SelectionSet::default();
        sel.set(Selection::Point(0)); // plain click/drag: replace
        assert_eq!(sel.len(), 1);
        sel.add(Selection::Point(1)); // shift: add
        assert!(sel.contains(Selection::Point(0)) && sel.contains(Selection::Point(1)));
        sel.remove(Selection::Point(0)); // ctrl: subtract
        assert!(!sel.contains(Selection::Point(0)));
        assert!(sel.contains(Selection::Point(1)));
        sel.set(Selection::Point(2)); // plain click again replaces the whole set
        assert_eq!(sel.len(), 1);
        assert!(sel.contains(Selection::Point(2)));
    }

    #[test]
    fn delete_selected_is_index_safe_across_types_and_cascade() {
        let mut doc = EditorDoc::default();
        let p0 = doc.weld_point(DVec3::new(0.0, 0.0, 0.0));
        let p1 = doc.weld_point(DVec3::new(1.0, 0.0, 0.0));
        let p2 = doc.weld_point(DVec3::new(2.0, 0.0, 0.0));
        let p3 = doc.weld_point(DVec3::new(3.0, 0.0, 0.0));
        let spar_dependent = doc.add_spar(p0, p1); // will be cascade-deleted with p0
        let spar_direct = doc.add_spar(p2, p3); // directly selected, independent of deleted points

        // Select p0 (drags spar_dependent down with it via cascade) and also
        // directly select spar_direct: mixed point + non-point selection.
        let sels = [Selection::Point(p0), Selection::Spar(spar_direct)];
        delete_selected(&mut doc, &sels);

        assert_eq!(doc.points.len(), 3, "one point removed");
        assert_eq!(doc.spars.len(), 0, "both spars gone: one cascaded, one direct");
        let _ = spar_dependent; // only referenced for documentation above
    }

    #[test]
    fn bridle_click_snaps_to_nearest_interior_node() {
        let mut doc = EditorDoc::default();
        let a = doc.weld_point(DVec3::new(0.0, 0.0, 0.0));
        let b = doc.weld_point(DVec3::new(3.0, 0.0, 0.0));
        let bi = doc.add_bridle(a, b);
        doc.bridles[bi].num_segments = 3; // interior nodes at x = 1.0 and x = 2.0

        // Straight down through (2, 0, z) hits the node at x=2 (k=2 of 3).
        let ray = Ray { origin: DVec3::new(2.0, 0.0, 5.0), dir: DVec3::new(0.0, 0.0, -1.0) };
        let eye = DVec3::new(2.0, 0.0, 5.0);
        assert_eq!(pick_bridle_node(&doc, &ray, eye), Some((bi, 2, 3)));

        // Feeding that ray through the Bridle tool's click handler welds a
        // new point at the node and queues it as the first pending endpoint.
        let mut state = ToolState { tool: Tool::Bridle, pending: Vec::new() };
        let mut sel = SelectionSet::default();
        let status = handle_click(&mut doc, &mut state, &mut sel, &ray, eye);
        assert_eq!(status, Some("snapped to node 2/3 of bridle_1".to_string()));
        assert_eq!(state.pending.len(), 1);
        let snapped = state.pending[0];
        assert!((doc.points[snapped] - DVec3::new(2.0, 0.0, 0.0)).length() < 1e-9);

        // A single-segment bridle has no interior nodes to snap to.
        let mut doc2 = EditorDoc::default();
        let a2 = doc2.weld_point(DVec3::new(0.0, 0.0, 0.0));
        let b2 = doc2.weld_point(DVec3::new(3.0, 0.0, 0.0));
        doc2.add_bridle(a2, b2); // default num_segments == 1
        assert_eq!(pick_bridle_node(&doc2, &ray, eye), None);
    }
}
