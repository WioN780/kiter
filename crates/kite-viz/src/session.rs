//! Live simulation session: owns the running `World`, paces stepping against
//! wall-clock time, records playback history, and hosts the grab tool.
//!
//! **Live tinkering invariant**: Config/Wind edits made while in Simulate
//! mode write DIRECTLY into `world.cfg` (see `panels::sim_live_panel`). This
//! is safe because cfg-only mutations never touch the constraint `Vec`s, so
//! `world.coloring` (the cached parallel-solve graph coloring) stays valid.
//! Structural doc edits (points/spars/panels/bridles) do NOT reach the live
//! `World` — `SimSession::is_stale` detects them so the UI can offer
//! "Apply & Restart" instead of silently going out of sync.

use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use glam::DVec3;
use kite_core::{build_kite_from_def, Event, World};

use crate::doc::EditorDoc;
use crate::snapshot::Snapshot;

/// Ring cap on `history`; oldest frames are dropped once exceeded.
const HISTORY_CAP: usize = 4096;
/// Hard cap on steps taken in a single UI frame, so a sim that can't keep up
/// with `speed x realtime` never freezes the UI trying to catch up.
const MAX_STEPS_PER_FRAME: usize = 64;
/// Per-step decay of the running max-abs used to autoscale force colors.
const COLOR_DECAY: f32 = 0.995;

pub struct GrabState {
    pub particle: usize,
    pub orig_inv_mass: f64,
}

/// Decaying running max-abs magnitude per force channel, used to autoscale
/// the color ramps and arrow lengths in the Simulate-mode scene.
#[derive(Default, Clone, Copy)]
pub struct ColorScale {
    pub spar: f32,
    pub bridle: f32,
    pub cloth: f32,
    pub aero: f32,
}

/// A structural-failure event, resolved to the joint particle it happened at
/// and the history index it happened on, for persistent 3D markers + timeline
/// ticks.
pub struct EventMarker {
    pub step: usize,
    pub particle: usize,
    /// true = SparBroken (rendered red), false = LeadingEdgeFolded (yellow).
    pub broken: bool,
}

pub struct SimSession {
    pub world: World,
    pub history: Vec<Snapshot>,
    pub playing: bool,
    pub speed: f64,
    pub dt: f64,
    pub budget: f64,
    pub sim_rate: f64,
    pub scrub: Option<usize>,
    pub diverged: Option<f64>,
    pub grab: Option<GrabState>,
    pub color_scale: ColorScale,
    pub event_markers: Vec<EventMarker>,
    built_hash: u64,
}

impl SimSession {
    pub fn new(doc: &EditorDoc) -> Self {
        let mut s = Self {
            world: build_world(doc),
            history: Vec::new(),
            playing: false,
            speed: 1.0,
            dt: 0.01,
            budget: 0.0,
            sim_rate: 0.0,
            scrub: None,
            diverged: None,
            grab: None,
            color_scale: ColorScale::default(),
            event_markers: Vec::new(),
            built_hash: structural_hash(doc),
        };
        s.push_snapshot(Vec::new());
        s
    }

    /// Rebuilds the live world from `doc` (used by "Apply & Restart"), wiping
    /// history, playback state and the grab tool.
    pub fn rebuild(&mut self, doc: &EditorDoc) {
        self.world = build_world(doc);
        self.history.clear();
        self.playing = false;
        self.budget = 0.0;
        self.sim_rate = 0.0;
        self.scrub = None;
        self.diverged = None;
        self.grab = None;
        self.color_scale = ColorScale::default();
        self.event_markers.clear();
        self.built_hash = structural_hash(doc);
        self.push_snapshot(Vec::new());
    }

    /// True once `doc` has structural edits (points/spars/panels/bridles/
    /// pins/junction) the live world was not built from.
    pub fn is_stale(&self, doc: &EditorDoc) -> bool {
        self.built_hash != structural_hash(doc)
    }

    fn push_snapshot(&mut self, events: Vec<Event>) {
        for ev in &events {
            let (joint_index, broken) = match *ev {
                Event::SparBroken { joint_index } => (joint_index, true),
                Event::LeadingEdgeFolded { joint_index } => (joint_index, false),
            };
            if let Some(particle) = joint_particle(&self.world, joint_index) {
                self.event_markers.push(EventMarker { step: self.history.len(), particle, broken });
            }
        }

        let snap = Snapshot::capture(&self.world, events);

        let max_abs = |vals: &[f32]| vals.iter().fold(0.0f32, |m, &v| m.max(v.abs()));
        self.color_scale.spar = (self.color_scale.spar * COLOR_DECAY).max(max_abs(&snap.spar_force));
        self.color_scale.bridle = (self.color_scale.bridle * COLOR_DECAY).max(max_abs(&snap.bridle_tension));
        self.color_scale.cloth = (self.color_scale.cloth * COLOR_DECAY).max(max_abs(&snap.cloth_force));
        let aero_max = snap
            .panel_aero
            .iter()
            .fold(0.0f32, |m, a| m.max(a.lift.length()).max(a.drag.length()));
        self.color_scale.aero = (self.color_scale.aero * COLOR_DECAY).max(aero_max);

        self.history.push(snap);
        if self.history.len() > HISTORY_CAP {
            // ponytail: O(n) drop-oldest via Vec::remove(0) instead of a
            // VecDeque/ring buffer. Fine at n <= 4096 for a debug tool;
            // switch structures if this ever profiles hot.
            self.history.remove(0);
            self.event_markers.retain_mut(|m| {
                if m.step == 0 {
                    false
                } else {
                    m.step -= 1;
                    true
                }
            });
        }
    }

    pub fn step_once(&mut self) {
        let before = self.world.events.len();
        self.world.step(self.dt);
        let events_this_step: Vec<Event> = self.world.events[before..].to_vec();

        if self.diverged.is_none() && self.world.particles.pos.iter().any(|p| !p.is_finite()) {
            self.diverged = Some(self.world.time);
            self.playing = false;
        }

        self.push_snapshot(events_this_step);
    }

    /// Advances playback by wall-clock `frame_elapsed` seconds, called once
    /// per UI frame. Steps at `dt` granularity until the accumulated budget
    /// (scaled by `speed`) is spent, capped so a slow sim can't stall the UI.
    pub fn tick(&mut self, frame_elapsed: f64) {
        if !self.playing || self.diverged.is_some() {
            self.sim_rate = 0.0;
            return;
        }
        self.budget += frame_elapsed * self.speed;
        let mut steps = 0usize;
        while self.budget >= self.dt && steps < MAX_STEPS_PER_FRAME {
            self.step_once();
            self.budget -= self.dt;
            steps += 1;
            if self.diverged.is_some() {
                break;
            }
        }
        if steps >= MAX_STEPS_PER_FRAME {
            // ponytail: drop the backlog rather than catching up over
            // several frames — keeps the UI responsive if the sim can't
            // keep up with speed x realtime.
            self.budget = 0.0;
        }
        self.sim_rate = if frame_elapsed > 1e-9 { (steps as f64 * self.dt) / frame_elapsed } else { 0.0 };
    }

    /// Runs `duration / dt` steps synchronously, filling history.
    pub fn bake(&mut self, duration: f64) {
        // ponytail: one blocking loop, no progress indicator/chunking across
        // frames. Bake runs are short at this tool's scale (a few hundred to
        // low-thousand steps); pick the chunked version if bakes grow long
        // enough to visibly stall the UI.
        let n = (duration / self.dt).round().max(0.0) as usize;
        for _ in 0..n {
            self.step_once();
            if self.diverged.is_some() {
                break;
            }
        }
    }

    /// Grabs `particle`, zeroing its inverse mass so the solver treats it as
    /// pinned while held.
    pub fn try_grab(&mut self, particle: usize) {
        let orig_inv_mass = self.world.particles.inv_mass[particle];
        self.world.particles.inv_mass[particle] = 0.0;
        self.grab = Some(GrabState { particle, orig_inv_mass });
    }

    /// Pins the grabbed particle's position/prev_pos/pred_pos to `target` and
    /// zeroes its velocity, for as long as it's held.
    pub fn drag_grab(&mut self, target: DVec3) {
        if let Some(g) = &self.grab {
            let i = g.particle;
            self.world.particles.pos[i] = target;
            self.world.particles.prev_pos[i] = target;
            self.world.particles.pred_pos[i] = target;
            self.world.particles.vel[i] = DVec3::ZERO;
        }
    }

    /// Restores the grabbed particle's original inverse mass VERBATIM — a
    /// particle that was already pinned before the grab stays pinned.
    pub fn release_grab(&mut self) {
        if let Some(g) = self.grab.take() {
            self.world.particles.inv_mass[g.particle] = g.orig_inv_mass;
        }
    }
}

/// Resolves a `bend_twist_constraints[joint_index]` failure to the particle
/// at its joint: `p2` of the stretch-shear constraint sharing its `q1_index`.
fn joint_particle(world: &World, joint_index: usize) -> Option<usize> {
    let bt = world.bend_twist_constraints.get(joint_index)?;
    world
        .stretch_shear_constraints
        .iter()
        .find(|c| c.q_index == bt.q1_index)
        .map(|c| c.p2)
}

fn build_world(doc: &EditorDoc) -> World {
    let mut world = World::new();
    world.cfg.gravity = doc.gravity;
    world.cfg.wind = doc.wind.clone();
    if let Some(v) = doc.sim.substeps {
        world.cfg.substeps = v;
    }
    if let Some(v) = doc.sim.iterations_per_substep {
        world.cfg.iterations_per_substep = v;
    }
    if let Some(v) = doc.sim.damping {
        world.cfg.damping = v;
    }
    if let Some(v) = doc.sim.bladder_pressure {
        world.cfg.bladder_pressure = v;
    }
    if let Some(v) = doc.sim.k_pressure {
        world.cfg.k_pressure = v;
    }
    if let Some(v) = doc.sim.ground_collision_enabled {
        world.cfg.ground_collision_enabled = v;
    }
    if let Some(v) = doc.sim.self_collision_enabled {
        world.cfg.self_collision_enabled = v;
    }
    let scenario = doc.to_scenario();
    if let Some(kite) = &scenario.kite {
        build_kite_from_def(&mut world, kite);
    }
    world
}

fn hash_f64(h: &mut DefaultHasher, v: f64) {
    v.to_bits().hash(h);
}

fn hash_dvec3(h: &mut DefaultHasher, v: DVec3) {
    hash_f64(h, v.x);
    hash_f64(h, v.y);
    hash_f64(h, v.z);
}

/// Hashes exactly the doc fields that feed `build_kite_from_def`, so
/// `SimSession::is_stale` can cheaply detect "doc edited since the live
/// world was built" without deriving `PartialEq`/`Hash` across the whole
/// `kite-core` definition tree (which spans crates). Names/duration/etc are
/// deliberately excluded — cosmetic fields the live world doesn't consume.
fn structural_hash(doc: &EditorDoc) -> u64 {
    let mut h = DefaultHasher::new();
    for p in &doc.points {
        hash_dvec3(&mut h, *p);
    }
    for s in &doc.spars {
        s.a.hash(&mut h);
        s.b.hash(&mut h);
        s.num_segments.hash(&mut h);
        hash_f64(&mut h, s.radius);
        hash_f64(&mut h, s.youngs_modulus);
        hash_f64(&mut h, s.shear_modulus);
        hash_f64(&mut h, s.density);
    }
    for p in &doc.panels {
        p.a.hash(&mut h);
        p.b.hash(&mut h);
        p.c.hash(&mut h);
        hash_f64(&mut h, p.warp_compliance);
        hash_f64(&mut h, p.weft_compliance);
        hash_f64(&mut h, p.shear_compliance);
        hash_f64(&mut h, p.bending_compliance);
        p.subdivisions.hash(&mut h);
        hash_f64(&mut h, p.areal_density);
    }
    for b in &doc.bridles {
        b.a.hash(&mut h);
        b.b.hash(&mut h);
        hash_f64(&mut h, b.rest_length);
        hash_f64(&mut h, b.compliance);
        hash_f64(&mut h, b.diameter);
        b.num_segments.hash(&mut h);
        hash_f64(&mut h, b.density);
    }
    for sj in &doc.stiff_joints {
        sj.point.hash(&mut h);
        hash_f64(&mut h, sj.compliance);
    }
    doc.junction.hash(&mut h);
    doc.junction_pinned.hash(&mut h);
    let mut pinned: Vec<usize> = doc.pinned.iter().copied().collect();
    pinned.sort_unstable();
    pinned.hash(&mut h);
    h.finish()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Pacing must not exceed the per-frame step cap even given a huge
    /// wall-clock stall (e.g. the window was minimized), and must spend
    /// exactly the steps it took out of the accumulated budget.
    #[test]
    fn tick_caps_steps_per_frame_and_drains_budget() {
        let mut s = SimSession::new(&EditorDoc::default());
        s.playing = true;
        s.dt = 0.01;
        s.speed = 1.0;

        s.tick(10.0); // 1000 steps worth of budget at dt=0.01, speed=1

        assert_eq!(s.history.len() - 1, MAX_STEPS_PER_FRAME, "must cap steps at the per-frame limit");
        assert_eq!(s.budget, 0.0, "leftover backlog beyond the cap must be dropped, not deferred");
    }

    /// A frame budget smaller than `dt` should take zero steps and simply
    /// accumulate.
    #[test]
    fn tick_accumulates_subthreshold_budget() {
        let mut s = SimSession::new(&EditorDoc::default());
        s.playing = true;
        s.dt = 0.01;
        s.speed = 1.0;

        s.tick(0.004);

        assert_eq!(s.history.len(), 1, "no step should have run yet");
        assert!((s.budget - 0.004).abs() < 1e-12);
    }

    #[test]
    fn structural_hash_changes_on_point_edit_not_on_cosmetic_edit() {
        let mut doc = EditorDoc::default();
        doc.points.push(DVec3::ZERO);
        doc.points.push(DVec3::new(1.0, 0.0, 0.0));
        doc.add_spar(0, 1);
        let h0 = structural_hash(&doc);

        doc.name = "renamed".to_string();
        doc.duration = 99.0;
        assert_eq!(h0, structural_hash(&doc), "cosmetic fields must not affect the structural hash");

        doc.points[1].x = 2.0;
        assert_ne!(h0, structural_hash(&doc), "a point move must change the structural hash");
    }
}
