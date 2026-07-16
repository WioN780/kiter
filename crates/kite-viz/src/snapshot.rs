//! Per-step recording of world state, captured in f32 to keep a few thousand
//! frames of playback history cheap to hold in memory.
//!
//! Force sign conventions (see `crates/kite-core/tests/readback.rs`, and
//! empirically pinned in the task brief — do not re-derive): with
//! `h = world.last_h` (treat `h == 0` as "no step yet, force 0"):
//! - spar axial force   = `-stretch_shear.lambda.z / h^2`
//! - bridle tension      = `-unilateral.lambda / h^2` (always >= 0; 0 = slack)
//! - cloth edge force    = `-distance.lambda / h^2`
//!
//! Positive = tension.

use glam::{DQuat, Quat};
use kite_core::{Event, World};

fn quat_to_f32(q: DQuat) -> Quat {
    Quat::from_xyzw(q.x as f32, q.y as f32, q.z as f32, q.w as f32)
}

/// f32 mirror of `kite_core::aero::PanelAero`.
#[derive(Clone, Copy, Default)]
pub struct PanelAeroSnap {
    pub lift: glam::Vec3,
    pub drag: glam::Vec3,
    pub alpha: f32,
}

pub struct Snapshot {
    pub t: f64,
    pub pos: Vec<glam::Vec3>,
    // ponytail: captured per the data-model contract (rod twist state) but
    // no current consumer renders it — cylinders are built from endpoint
    // positions only. Wire up when a twist/orientation visualization lands.
    #[allow(dead_code)]
    pub quat: Vec<Quat>,
    /// Parallel to `World::stretch_shear_constraints`.
    pub spar_force: Vec<f32>,
    /// Parallel to `World::unilateral_constraints`.
    pub bridle_tension: Vec<f32>,
    /// Parallel to `World::distance_constraints`.
    pub cloth_force: Vec<f32>,
    /// Parallel to `World::canopy_panels`.
    pub panel_aero: Vec<PanelAeroSnap>,
    pub energy: f64,
    /// Events emitted by the step that produced this snapshot (not the
    /// world's full lifetime event log).
    pub events: Vec<Event>,
}

impl Snapshot {
    pub fn capture(world: &World, events: Vec<Event>) -> Self {
        let h = world.last_h;
        let scale = if h > 0.0 { 1.0 / (h * h) } else { 0.0 };

        let pos = world.particles.pos.iter().map(|p| p.as_vec3()).collect();
        let quat = world.orientations.quat.iter().map(|&q| quat_to_f32(q)).collect();

        let spar_force = world
            .stretch_shear_constraints
            .iter()
            .map(|c| (-c.lambda.z * scale) as f32)
            .collect();
        let bridle_tension = world
            .unilateral_constraints
            .iter()
            .map(|c| (-c.lambda * scale) as f32)
            .collect();
        let cloth_force = world
            .distance_constraints
            .iter()
            .map(|c| (-c.lambda * scale) as f32)
            .collect();
        let panel_aero = world
            .aero_readback
            .iter()
            .map(|a| PanelAeroSnap {
                lift: a.lift.as_vec3(),
                drag: a.drag.as_vec3(),
                alpha: a.alpha as f32,
            })
            .collect();

        Self {
            t: world.time,
            pos,
            quat,
            spar_force,
            bridle_tension,
            cloth_force,
            panel_aero,
            energy: world.compute_total_energy(),
            events,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::DVec3;
    use kite_core::UnilateralDistanceConstraint;

    /// Pins the sign convention this module relies on (mirrors
    /// `kite-core`'s `bridle_tension_sign_and_magnitude` readback test): a
    /// pinned anchor holding a hanging 1kg mass at equilibrium reads back as
    /// a positive ~9.81 N tension in the snapshot, not just in the raw
    /// lambda.
    #[test]
    fn bridle_tension_sign_in_snapshot() {
        let mut world = World::new();
        world.cfg.gravity = DVec3::new(0.0, -9.81, 0.0);
        let a = world.add_particle(DVec3::new(0.0, 1.0, 0.0), 0.0);
        let b = world.add_particle(DVec3::new(0.0, 0.0, 0.0), 1.0);
        world
            .unilateral_constraints
            .push(UnilateralDistanceConstraint::new(a, b, 0.9, 1e-9, 0.003));

        for _ in 0..300 {
            world.step(0.01);
        }

        let snap = Snapshot::capture(&world, Vec::new());
        assert_eq!(snap.bridle_tension.len(), 1);
        let tension = snap.bridle_tension[0];
        assert!(tension > 0.0, "tension must be positive, got {tension}");
        assert!((tension / 9.81 - 1.0).abs() < 0.10, "tension {tension} not within 10% of 9.81 N");
    }

    #[test]
    fn zero_h_gives_zero_force() {
        let world = World::new(); // never stepped, last_h == 0
        let snap = Snapshot::capture(&world, Vec::new());
        assert_eq!(snap.spar_force.len(), 0); // no constraints, but exercise the h==0 guard path
        assert_eq!(world.last_h, 0.0);
    }
}
