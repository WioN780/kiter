//! Wind visualization overlay: a sampled arrow grid and advected streak
//! particles, both driven by `kite_core::wind::wind_at`. Owned by `App` so
//! toggle/box state and streak particles survive Edit <-> Simulate switches.

use std::collections::VecDeque;

use eframe::egui;
use glam::DVec3;
use kite_core::wind::{wind_at, WindConfig};

use crate::render::{ArrowInst, LineSeg, SceneData};

const NUM_PARTICLES: usize = 200;
const TRAIL_LEN: usize = 16;
const MAX_AGE: f64 = 6.0;
/// Clamp on the per-frame advection step so a UI stall doesn't fling
/// particles far outside the box in one jump.
const MAX_FRAME_DT: f64 = 0.05;

struct Streak {
    pos: DVec3,
    trail: VecDeque<DVec3>,
    age: f64,
}

/// splitmix64: tiny, dependency-free PRNG. Cosmetic randomness only (streak
/// respawn positions), not part of any simulation state, so it doesn't need
/// to route through `World`'s seeded RNG (masterplan rule 6 is about
/// engine determinism; this is a debug-viz particle spawner).
fn next_u64(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E3779B97F4A7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
    z ^ (z >> 31)
}

/// Uniform in `[-0.5, 0.5)`.
fn next_signed_unit(state: &mut u64) -> f64 {
    (next_u64(state) >> 11) as f64 * (1.0 / (1u64 << 53) as f64) - 0.5
}

impl Streak {
    fn spawn(rng: &mut u64, box_center: DVec3, box_size: DVec3) -> Self {
        let pos = box_center
            + DVec3::new(
                next_signed_unit(rng) * box_size.x,
                next_signed_unit(rng) * box_size.y,
                next_signed_unit(rng) * box_size.z,
            );
        Self { pos, trail: VecDeque::with_capacity(TRAIL_LEN), age: 0.0 }
    }
}

pub struct WindViz {
    pub grid_enabled: bool,
    pub streaks_enabled: bool,
    pub box_center: DVec3,
    pub box_size: DVec3,
    pub grid_res: [usize; 3],
    particles: Vec<Streak>,
    /// Animated preview clock for Edit mode (no live `World::time` to read
    /// there); accumulates real UI frame time so turbulence is previewable
    /// before running.
    edit_clock: f64,
    rng: u64,
}

impl WindViz {
    pub fn new() -> Self {
        let box_center = DVec3::new(0.0, 3.0, 0.0);
        let box_size = DVec3::new(8.0, 6.0, 8.0);
        let mut rng = 0x2545_F491_4F6C_DD1Du64;
        let particles = (0..NUM_PARTICLES).map(|_| Streak::spawn(&mut rng, box_center, box_size)).collect();
        Self {
            grid_enabled: false,
            streaks_enabled: false,
            box_center,
            box_size,
            grid_res: [8, 4, 8],
            particles,
            edit_clock: 0.0,
            rng,
        }
    }

    /// Advances streak particles by one UI frame and returns the wind-sample
    /// time to use this frame: `sim_t` verbatim in Simulate mode, or the
    /// accumulated preview clock in Edit mode (`sim_t = None`).
    pub fn tick(&mut self, frame_dt: f64, sim_t: Option<f64>, cfg: &WindConfig) -> f64 {
        let t = match sim_t {
            Some(t) => t,
            None => {
                self.edit_clock += frame_dt.max(0.0);
                self.edit_clock
            }
        };
        if self.streaks_enabled {
            let dt = frame_dt.clamp(0.0, MAX_FRAME_DT);
            let (box_center, box_size) = (self.box_center, self.box_size);
            for p in &mut self.particles {
                let v = wind_at(p.pos, t, cfg);
                p.pos += v * dt;
                p.age += dt;
                p.trail.push_back(p.pos);
                if p.trail.len() > TRAIL_LEN {
                    p.trail.pop_front();
                }
                if p.age > MAX_AGE || !box_contains(p.pos, box_center, box_size) {
                    *p = Streak::spawn(&mut self.rng, box_center, box_size);
                }
            }
        }
        t
    }

    /// Emits one arrow per grid cell center, length autoscaled so the
    /// fastest sampled cell reads as ~90% of the cell size, colored on a
    /// blue (slow) -> red-orange (fast) ramp.
    pub fn build_arrows(&self, t: f64, cfg: &WindConfig, scene: &mut SceneData) {
        if !self.grid_enabled {
            return;
        }
        let [nx, ny, nz] = self.grid_res;
        if nx == 0 || ny == 0 || nz == 0 {
            return;
        }
        let cell = DVec3::new(self.box_size.x / nx as f64, self.box_size.y / ny as f64, self.box_size.z / nz as f64);
        let origin = self.box_center - self.box_size * 0.5;

        let mut samples = Vec::with_capacity(nx * ny * nz);
        let mut max_speed = 1e-6f64;
        for ix in 0..nx {
            for iy in 0..ny {
                for iz in 0..nz {
                    let p = origin
                        + DVec3::new((ix as f64 + 0.5) * cell.x, (iy as f64 + 0.5) * cell.y, (iz as f64 + 0.5) * cell.z);
                    let v = wind_at(p, t, cfg);
                    max_speed = max_speed.max(v.length());
                    samples.push((p, v));
                }
            }
        }

        let cell_min = cell.x.min(cell.y).min(cell.z);
        let scale = cell_min * 0.9 / max_speed;
        for (p, v) in samples {
            let speed = v.length();
            if speed < 1e-6 {
                continue;
            }
            scene.arrows.push(ArrowInst {
                origin: p,
                vec: v * scale,
                shaft_radius: (cell_min * 0.03).max(0.01),
                color: speed_color((speed / max_speed) as f32),
            });
        }
    }

    /// Emits each streak's trail as a chain of fading line segments (newest
    /// segment brightest; the color ramp does the fading since the shared
    /// line pipeline doesn't alpha-blend).
    pub fn build_streaks(&self, scene: &mut SceneData) {
        if !self.streaks_enabled {
            return;
        }
        for p in &self.particles {
            let n = p.trail.len();
            if n < 2 {
                continue;
            }
            for i in 1..n {
                let brightness = (i as f32 / (n - 1) as f32).max(0.12);
                scene.lines.push(LineSeg {
                    a: p.trail[i - 1],
                    b: p.trail[i],
                    color: [0.35 * brightness, 0.8 * brightness, 1.0 * brightness, 1.0],
                });
            }
        }
    }

    pub fn inspector(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Wind Viz").show(ui, |ui| {
            ui.checkbox(&mut self.grid_enabled, "Arrow grid");
            ui.checkbox(&mut self.streaks_enabled, "Streaks");
            ui.horizontal(|ui| {
                ui.label("Box center");
                ui.add(egui::DragValue::new(&mut self.box_center.x).speed(0.1).prefix("x: "));
                ui.add(egui::DragValue::new(&mut self.box_center.y).speed(0.1).prefix("y: "));
                ui.add(egui::DragValue::new(&mut self.box_center.z).speed(0.1).prefix("z: "));
            });
            ui.horizontal(|ui| {
                ui.label("Box size");
                ui.add(egui::DragValue::new(&mut self.box_size.x).speed(0.1).range(0.1..=f64::MAX).prefix("x: "));
                ui.add(egui::DragValue::new(&mut self.box_size.y).speed(0.1).range(0.1..=f64::MAX).prefix("y: "));
                ui.add(egui::DragValue::new(&mut self.box_size.z).speed(0.1).range(0.1..=f64::MAX).prefix("z: "));
            });
            ui.horizontal(|ui| {
                ui.label("Grid res");
                ui.add(egui::DragValue::new(&mut self.grid_res[0]).range(1..=32));
                ui.add(egui::DragValue::new(&mut self.grid_res[1]).range(1..=32));
                ui.add(egui::DragValue::new(&mut self.grid_res[2]).range(1..=32));
            });
        });
    }
}

impl Default for WindViz {
    fn default() -> Self {
        Self::new()
    }
}

fn box_contains(p: DVec3, box_center: DVec3, box_size: DVec3) -> bool {
    let d = p - box_center;
    d.x.abs() <= box_size.x * 0.5 && d.y.abs() <= box_size.y * 0.5 && d.z.abs() <= box_size.z * 0.5
}

/// Diverging speed ramp: blue-teal (slow) -> red-orange (fast), `t` in `[0, 1]`.
fn speed_color(t: f32) -> [f32; 4] {
    let t = t.clamp(0.0, 1.0);
    let slow = [0.15, 0.55, 0.95];
    let fast = [1.0, 0.35, 0.1];
    [slow[0] + (fast[0] - slow[0]) * t, slow[1] + (fast[1] - slow[1]) * t, slow[2] + (fast[2] - slow[2]) * t, 1.0]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// After many ticks with a nonzero mean wind, every particle stays
    /// within the box (respawn-on-exit keeps the visualization bounded) and
    /// no coordinate goes non-finite.
    #[test]
    fn streaks_stay_bounded_and_finite() {
        let mut wv = WindViz::new();
        wv.streaks_enabled = true;
        let cfg = WindConfig { v_ref: 8.0, ..Default::default() };
        for i in 0..500 {
            wv.tick(0.02, Some(i as f64 * 0.02), &cfg);
        }
        for p in &wv.particles {
            assert!(p.pos.is_finite(), "particle position went non-finite");
            assert!(
                box_contains(p.pos, wv.box_center, wv.box_size),
                "particle escaped the box without respawning"
            );
        }
    }
}
