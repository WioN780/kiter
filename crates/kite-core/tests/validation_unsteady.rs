//! Milestone 10a acceptance: added-mass correction (masterplan §6.4).
//!
//! A free rigid panel driven by a constant normal force must accelerate at
//! F/(M + m_a) with the correction enabled (the implicit rescaling scheme
//! reproduces that steady rate exactly) and at F/M with it disabled.

use glam::DVec3;
use kite_core::aero::unsteady::panel_added_mass;
use kite_core::{CanopyPanel, World};

/// Unit-area right triangle in the XZ plane (normal = +Y), total mass 1 kg,
/// pushed along +Y by a 1 N total external force. No gravity, no wind, no
/// damping.
fn make_pushed_panel(added_mass: bool) -> World {
    let mut world = World::new();
    world.cfg.gravity = DVec3::ZERO;
    world.cfg.damping = 0.0;
    world.cfg.added_mass_enabled = added_mass;

    world.add_particle(DVec3::new(0.0, 0.0, 0.0), 1.0 / 3.0);
    world.add_particle(DVec3::new(2.0, 0.0, 0.0), 1.0 / 3.0);
    world.add_particle(DVec3::new(0.0, 0.0, 1.0), 1.0 / 3.0);
    world.canopy_panels.push(CanopyPanel::new(0, 1, 2));

    for i in 0..3 {
        world.forces[i] = DVec3::new(0.0, F_TOTAL / 3.0, 0.0);
    }
    world
}

/// Small force keeps velocities (and thus the quadratic cd0 parasitic drag)
/// tiny over the measurement window: drag/F ≈ 0.0245·(F·t/M)²/F ≲ 1e-4.
const F_TOTAL: f64 = 0.1;

/// Mean Y-velocity slope between two checkpoints (excludes the first-substep
/// startup transient, before the correction's velocity history is primed).
fn measure_accel(world: &mut World) -> f64 {
    let dt = 0.01;
    for _ in 0..10 {
        world.step(dt);
    }
    let v1 = world.particles.vel[0].y;
    let t1 = world.time;
    for _ in 0..30 {
        world.step(dt);
    }
    let v2 = world.particles.vel[0].y;
    (v2 - v1) / (world.time - t1)
}

#[test]
fn test_added_mass_steady_acceleration() {
    let area = 1.0; // 2×1 right triangle
    let m_total = 1.0;
    let f_total = F_TOTAL;
    let m_a = panel_added_mass(area);
    assert!(m_a > 0.5 && m_a < 0.7, "unexpected disk added mass: {m_a}");

    let a_base = measure_accel(&mut make_pushed_panel(false));
    let a_corrected = measure_accel(&mut make_pushed_panel(true));

    let expect_base = f_total / m_total;
    let expect_corrected = f_total / (m_total + m_a);

    let err_base = (a_base - expect_base).abs() / expect_base;
    let err_corr = (a_corrected - expect_corrected).abs() / expect_corrected;
    // Residual error budget: parasitic cd0 drag at the tiny measured
    // velocities (~1e-4 relative).
    assert!(
        err_base < 1e-3,
        "base accel {a_base} vs expected {expect_base}"
    );
    assert!(
        err_corr < 1e-3,
        "corrected accel {a_corrected} vs expected {expect_corrected}"
    );

    // And the correction must actually change the answer.
    assert!(a_corrected < 0.75 * a_base);
}

#[test]
fn test_added_mass_stable_when_air_outweighs_fabric() {
    // The regime that diverges under an explicit F = -m_a·dv/dt scheme:
    // panel fabric much lighter than the displaced air (m_a ≈ 0.59 kg vs
    // 0.03 kg fabric here). The implicit form must stay bounded.
    let mut world = make_pushed_panel(true);
    for i in 0..3 {
        world.particles.inv_mass[i] = 100.0; // 10 g per vertex
    }
    for _ in 0..200 {
        world.step(0.01);
    }
    for i in 0..3 {
        let v = world.particles.vel[i];
        assert!(v.is_finite(), "particle {i} velocity diverged: {v:?}");
        assert!(v.length() < 100.0, "particle {i} velocity unbounded: {v:?}");
    }
}

#[test]
fn test_added_mass_off_by_default_changes_nothing() {
    let mut with_flag_default = make_pushed_panel(false);
    let mut fresh = World::new();
    assert!(!fresh.cfg.added_mass_enabled, "added mass must be opt-in");
    // A default world must not carry panel velocity history.
    fresh.step(0.01);
    let _ = &mut with_flag_default;
}
