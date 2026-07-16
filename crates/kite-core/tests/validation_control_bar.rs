//! Milestone 10c acceptance: dynamic control bar (masterplan §5.5).
//!
//! The bar is a rigid body pinned at its center to a fixed pilot anchor by a
//! spherical joint (Rapier — approved for truly rigid accessories only). Kite
//! lines couple to it via two kinematic XPBD anchor particles: bar tips drive
//! the anchors each substep, and the recovered line forces (F = λ/h²) drive
//! the bar back.

use glam::DVec3;
use kite_core::{ControlBar, UnilateralDistanceConstraint, World};

const HALF_SPAN: f64 = 0.25;
const BAR_MASS: f64 = 2.0;
const LOAD_MASS: f64 = 0.05;
const DT: f64 = 1.0 / 60.0;

/// Bar centered at the origin with a hanging point mass on the left tip line
/// and, optionally, a matching one on the right.
///
/// The load is deliberately light relative to the bar: the §5.5 boundary
/// exchange is an *explicit* staggered coupling, so the loaded tip must
/// accelerate downward slower than g (α·L = m·g·L²/I < g) or the unilateral
/// line slack-snaps, and each snap feeds an impulsive λ/h² kick back into the
/// bar — a positive-feedback loop that diverges. Same stability rule any
/// co-simulation force exchange has.
fn make_bar_world(right_mass: Option<f64>) -> (World, usize, usize) {
    let mut world = World::new();
    world.cfg.gravity = DVec3::new(0.0, -9.81, 0.0);
    world.cfg.substeps = 8;
    world.cfg.iterations_per_substep = 8;
    world.cfg.damping = 2.0;

    // Kinematic anchor particles at the bar tip positions.
    let left = world.add_particle(DVec3::new(-HALF_SPAN, 0.0, 0.0), 0.0);
    let right = world.add_particle(DVec3::new(HALF_SPAN, 0.0, 0.0), 0.0);

    // Hanging mass on a 1 m line from the left tip, starting exactly at rest
    // length so tension builds gravitationally rather than as a snap load.
    let m_left = world.add_particle(DVec3::new(-HALF_SPAN, -1.0, 0.0), LOAD_MASS);
    world
        .unilateral_constraints
        .push(UnilateralDistanceConstraint::new(
            left, m_left, 1.0, 1e-8, 0.002,
        ));

    if let Some(mass) = right_mass {
        let m_right = world.add_particle(DVec3::new(HALF_SPAN, -1.0, 0.0), mass);
        world
            .unilateral_constraints
            .push(UnilateralDistanceConstraint::new(
                right, m_right, 1.0, 1e-8, 0.002,
            ));
    }

    world.control_bar = Some(ControlBar::new(
        DVec3::ZERO,
        HALF_SPAN,
        BAR_MASS,
        left,
        right,
    ));
    (world, left, right)
}

fn assert_all_finite(world: &World) {
    for p in &world.particles.pos {
        assert!(p.is_finite(), "non-finite particle position {p:?}");
    }
    let bar = world.control_bar.as_ref().unwrap();
    assert!(bar.tip_position(true).is_finite());
    assert!(bar.tip_position(false).is_finite());
    let q = bar.orientation();
    assert!(
        (q.length() - 1.0).abs() < 1e-6,
        "bar quaternion drifted: {q:?}"
    );
}

#[test]
fn test_symmetric_load_keeps_bar_level_and_centered() {
    let (mut world, left, right) = make_bar_world(Some(LOAD_MASS));

    for _ in 0..180 {
        world.step(DT);
        let (pl, pr) = (world.particles.pos[left], world.particles.pos[right]);
        // Mirror-symmetric load: no net torque, bar stays level...
        assert!(
            (pl.y - pr.y).abs() < 1e-6,
            "bar tilted under symmetric load: {} vs {}",
            pl.y,
            pr.y
        );
        // ...and the spherical joint keeps the bar center at the pilot.
        assert!(
            ((pl + pr) * 0.5).length() < 1e-3,
            "bar center drifted off the pilot anchor: {:?}",
            (pl + pr) * 0.5
        );
    }
    assert_all_finite(&world);
}

#[test]
fn test_asymmetric_load_swings_loaded_tip_down() {
    let (mut world, left, right) = make_bar_world(None);

    // The line tension must torque the loaded (-X) tip down and the free tip
    // up: within the first swing (≲1 s — the bar then swings through like the
    // underdamped pendulum it is) the loaded tip must drop well below the
    // free one. Detecting the crossing rather than sampling a fixed time
    // keeps the test independent of the exact swing phase.
    let mut swung_down = false;
    for _ in 0..60 {
        world.step(DT);
        let (y_left, y_right) = (world.particles.pos[left].y, world.particles.pos[right].y);
        if y_left < y_right - 0.1 {
            swung_down = true;
            break;
        }
    }
    assert!(swung_down, "loaded tip never swung below the free tip");

    // Long run: everything stays finite and the joint keeps the bar center
    // pinned at the pilot regardless of the rotation.
    for _ in 0..300 {
        world.step(DT);
        let center = (world.particles.pos[left] + world.particles.pos[right]) * 0.5;
        assert!(center.length() < 1e-3, "bar center drifted: {center:?}");
    }
    assert_all_finite(&world);
}

#[test]
fn test_anchors_track_bar_tips() {
    let (mut world, left, _) = make_bar_world(None);
    for _ in 0..60 {
        world.step(DT);
    }
    // The anchor is re-synced at the top of each substep, so after a full step
    // it may lag the bar by at most one substep of tip travel.
    let bar = world.control_bar.as_ref().unwrap();
    let lag = (world.particles.pos[left] - bar.tip_position(true)).length();
    assert!(lag < 0.01, "anchor lost the bar tip: lag {lag}");
}
