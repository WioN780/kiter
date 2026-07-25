//! Milestone 9 acceptance: the graph-colored parallel solver must agree with
//! the serial reference, and be deterministic run-to-run.
//!
//! Notes (masterplan §8.5):
//! - Parallel mode reorders constraint solves (color-class order instead of
//!   insertion order). For scenes with a *unique* equilibrium the two modes
//!   must converge to the same answer tightly; that's what the chain and rod
//!   scenes assert per constraint kernel.
//! - A wrinkling cloth is *bistable*: solve-order perturbation can select a
//!   different fold ~mm apart. The cloth test therefore only asserts loose
//!   agreement plus convergence, and documents that this is expected physics,
//!   not solver drift.
//! - Within a color class constraints touch disjoint state, so parallel
//!   execution itself is bit-deterministic run-to-run.
//!
//! Under `reference-mode` the parallel path is compiled out, making every
//! comparison here vacuous (serial vs serial), so the whole file is skipped.
#![cfg(not(feature = "reference-mode"))]

use glam::{DQuat, DVec3};
use kite_core::geometry::build_cloth_grid;
use kite_core::materials::{bend_twist_compliance, stretch_shear_compliance};
use kite_core::solver::PARALLEL_CONSTRAINT_THRESHOLD;
use kite_core::{
    BendTwistConstraint, DistanceConstraint, Material, SectionGeometry, StretchShearConstraint,
    UnilateralDistanceConstraint, World,
};

fn base_world(parallel: bool) -> World {
    let mut world = World::new();
    world.cfg.parallel_solve = parallel;
    world.cfg.substeps = 8;
    world.cfg.iterations_per_substep = 2;
    world.cfg.gravity = DVec3::new(0.0, -9.81, 0.0);
    world.cfg.damping = 6.0;
    world
}

fn run(world: &mut World, steps: usize) -> Vec<DVec3> {
    for _ in 0..steps {
        world.step(0.01);
    }
    world.particles.pos.clone()
}

fn max_deviation(a: &[DVec3], b: &[DVec3]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (*x - *y).length())
        .fold(0.0f64, f64::max)
}

/// 40 hanging chains, each with a unilateral tether from its pin to its bottom
/// particle that goes taut at equilibrium. Exercises the distance and
/// unilateral kernels; the equilibrium (each chain straight, tether taut) is
/// unique.
fn make_chain_forest(parallel: bool) -> World {
    let mut world = base_world(parallel);
    let chains = 40;
    let links = 15;
    let link_len = 0.05;

    for c in 0..chains {
        let root = DVec3::new(c as f64 * 0.3, 0.0, 0.0);
        let first = world.particles.len();
        for i in 0..=links {
            // Start horizontal (+Z) so the chains actually have to swing down.
            let mass = if i == 0 { 0.0 } else { 0.05 };
            world.add_particle(root + DVec3::new(0.0, 0.0, i as f64 * link_len), mass);
        }
        for i in 0..links {
            world.distance_constraints.push(DistanceConstraint::new(
                first + i,
                first + i + 1,
                link_len,
                1e-8,
            ));
        }
        // Tether shorter than the chain: taut at equilibrium, so the
        // unilateral kernel does real work.
        world
            .unilateral_constraints
            .push(UnilateralDistanceConstraint::new(
                first,
                first + links,
                0.8 * links as f64 * link_len,
                1e-8,
                0.001,
            ));
    }
    world
}

/// 40 cantilever rods bending under gravity. Exercises the Cosserat
/// stretch-shear and bend-twist kernels; the small-deflection equilibrium is
/// unique.
fn make_rod_forest(parallel: bool) -> World {
    let mut world = base_world(parallel);
    let rods = 40;
    let segments = 10;
    let length = 1.0;
    let dx = length / segments as f64;
    let radius = 0.01;
    let density = 1000.0;
    let geom = SectionGeometry::SolidRound { radius };
    let segment_mass = density * geom.area() * dx;
    let material = Material {
        youngs_modulus: 5.0e9,
        shear_modulus: 2.0e9,
        density,
        tensile_strength: 1.0e12,
    };
    let comp_ss = stretch_shear_compliance(&material, &geom, dx);
    let comp_bt = bend_twist_compliance(&material, &geom, dx);
    let inertia = geom.compute_inertia(dx, segment_mass);
    let inv_inertia = DVec3::new(1.0 / inertia.x, 1.0 / inertia.y, 1.0 / inertia.z);

    for r in 0..rods {
        let root = DVec3::new(r as f64 * 0.5, 0.0, 0.0);
        let first_p = world.particles.len();
        let first_q = world.orientations.len();
        for i in 0..=segments {
            let mass = if i == 0 { 0.0 } else { segment_mass };
            world.add_particle(root + DVec3::new(0.0, 0.0, i as f64 * dx), mass);
        }
        for i in 0..segments {
            let w = if i == 0 { DVec3::ZERO } else { inv_inertia };
            world.add_segment(DQuat::IDENTITY, w);
        }
        for i in 0..segments {
            world
                .stretch_shear_constraints
                .push(StretchShearConstraint::new(
                    first_p + i,
                    first_p + i + 1,
                    first_q + i,
                    dx,
                    comp_ss,
                    2.0 * radius,
                ));
        }
        for i in 0..segments - 1 {
            world.bend_twist_constraints.push(BendTwistConstraint::new(
                first_q + i,
                first_q + i + 1,
                DVec3::ZERO,
                comp_bt,
                f64::MAX,
                false,
            ));
        }
    }
    world
}

fn make_cloth_world(parallel: bool) -> World {
    let mut world = base_world(parallel);
    build_cloth_grid(&mut world, 24, 0.05, 0.02);
    world
}

#[test]
fn test_scenes_exceed_parallel_threshold() {
    for (name, world) in [
        ("chains", make_chain_forest(true)),
        ("rods", make_rod_forest(true)),
        ("cloth", make_cloth_world(true)),
    ] {
        let total = world.distance_constraints.len()
            + world.stretch_shear_constraints.len()
            + world.bend_twist_constraints.len()
            + world.dihedral_bending_constraints.len()
            + world.unilateral_constraints.len();
        assert!(
            total >= PARALLEL_CONSTRAINT_THRESHOLD,
            "{name} scene too small to exercise the parallel path: {total} constraints"
        );
    }
}

#[test]
fn test_parallel_matches_serial_chains() {
    let serial = run(&mut make_chain_forest(false), 400);
    let parallel = run(&mut make_chain_forest(true), 400);
    // Measured ordering residual: 8.3e-5 m on a 0.75 m chain (0.011%).
    let dev = max_deviation(&serial, &parallel);
    assert!(dev < 5e-4, "chain equilibria deviate by {dev} m");

    // Sanity: chains actually swung down and the tether went taut.
    let bottom = parallel[15]; // bottom of first chain
    assert!(bottom.y < -0.5, "chain never fell: bottom at {bottom:?}");
}

#[test]
fn test_parallel_matches_serial_rods() {
    // Gauss-Seidel solve *order* shifts the under-iterated fixed point, so
    // serial vs parallel differ by an amount that shrinks as iterations rise
    // (probed: gap 4.4e-3 @ 2 iters → 1.1e-3 @ 8 iters → 3.6e-4 @ 20 iters,
    // both modes converging toward the Euler-Bernoulli deflection). Assert
    // both the absolute bound at the cheap setting and the shrink trend:
    // a genuine kernel-math divergence would fail the trend.
    let serial_2 = run(&mut make_rod_forest(false), 500);
    let parallel_2 = run(&mut make_rod_forest(true), 500);
    let gap_2 = max_deviation(&serial_2, &parallel_2);
    assert!(
        gap_2 < 1e-2,
        "rod equilibria deviate by {gap_2} m at 2 iters"
    );

    let mut ws8 = make_rod_forest(false);
    let mut wp8 = make_rod_forest(true);
    ws8.cfg.iterations_per_substep = 8;
    wp8.cfg.iterations_per_substep = 8;
    let gap_8 = max_deviation(&run(&mut ws8, 500), &run(&mut wp8, 500));
    assert!(
        gap_8 < 0.5 * gap_2,
        "serial/parallel gap did not shrink with iterations: {gap_2} @ 2 iters vs {gap_8} @ 8 iters"
    );

    // Sanity: rods actually deflected.
    let tip = parallel_2[10]; // tip of first rod
    assert!(tip.y < -1e-4, "cantilever never deflected: tip at {tip:?}");
}

#[test]
fn test_parallel_matches_serial_cloth_loosely() {
    // Bistable wrinkle modes: only loose agreement is physical here (see
    // module docs). Tight agreement is asserted on the unique-equilibrium
    // scenes above.
    let mut ws = make_cloth_world(false);
    let mut wp = make_cloth_world(true);
    ws.cfg.damping = 8.0;
    wp.cfg.damping = 8.0;
    let serial = run(&mut ws, 400);
    let parallel = run(&mut wp, 400);
    let dev = max_deviation(&serial, &parallel);
    assert!(dev < 1e-2, "cloth equilibria deviate by {dev} m (>1 cm)");

    // Both must be converged (settled, not drifting).
    let vel = wp
        .particles
        .vel
        .iter()
        .map(|v| v.length())
        .fold(0.0f64, f64::max);
    assert!(vel < 1e-2, "parallel cloth not settled: max vel {vel}");
}

#[test]
fn test_parallel_is_deterministic_run_to_run() {
    let a = run(&mut make_cloth_world(true), 50);
    let b = run(&mut make_cloth_world(true), 50);
    for (i, (pa, pb)) in a.iter().zip(&b).enumerate() {
        assert_eq!(
            pa, pb,
            "particle {i} differs between identical parallel runs"
        );
    }
}

#[test]
fn test_parallel_survives_constraint_addition_between_steps() {
    // The coloring cache must self-invalidate when constraints change.
    let mut world = make_cloth_world(true);
    run(&mut world, 5);

    let n = 24;
    let p1 = n - 1; // top-right (pinned row)
    let p2 = n * n - 1; // bottom-right
    let rest = (world.particles.pos[p1] - world.particles.pos[p2]).length();
    world
        .distance_constraints
        .push(DistanceConstraint::new(p1, p2, rest * 0.9, 1e-6));

    let positions = run(&mut world, 5);
    for (i, p) in positions.iter().enumerate() {
        assert!(
            p.is_finite(),
            "particle {i} exploded after constraint addition"
        );
    }
}
