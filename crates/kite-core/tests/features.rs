use glam::DVec3;
use kite_core::{
    build_kite_from_def, BridleLineDef, KiteDefinition, LashingDef, PanelDef, SparDef,
    StiffJunctionDef, World,
};

fn empty_kite_def(name: &str) -> KiteDefinition {
    KiteDefinition {
        name: name.to_string(),
        spars: Vec::new(),
        panels: Vec::new(),
        bridles: Vec::new(),
        bridle_junction: DVec3::ZERO,
        bridle_junction_pinned: false,
        pinned_points: Vec::new(),
        stiff_junctions: Vec::new(),
        lashings: Vec::new(),
    }
}

fn find_particle_near(world: &World, target: DVec3, tol: f64) -> usize {
    (0..world.particles.len())
        .find(|&i| (world.particles.pos[i] - target).length() < tol)
        .unwrap_or_else(|| panic!("no particle found near {:?}", target))
}

// ---------------------------------------------------------------------------
// Feature 1: panel mesh subdivision
// ---------------------------------------------------------------------------

#[test]
fn subdivision_produces_expected_mesh_and_mass() {
    let n = 3usize;
    let areal_density = 0.2;
    let mut def = empty_kite_def("subdiv");
    def.panels.push(PanelDef {
        name: "p0".to_string(),
        p1: DVec3::new(0.0, 0.0, 0.0),
        p2: DVec3::new(1.0, 0.0, 0.0),
        p3: DVec3::new(0.0, 0.0, 1.0),
        warp_compliance: 1e-6,
        weft_compliance: 1e-6,
        shear_compliance: 1e-6,
        bending_compliance: 1e-3,
        subdivisions: n,
        areal_density,
    });

    let mut world = World::new();
    build_kite_from_def(&mut world, &def);

    // (n+1)(n+2)/2 grid vertices, n^2 sub-triangles.
    assert_eq!(world.particles.len(), 10);
    assert_eq!(world.canopy_panels.len(), 9);

    // Total accumulated fabric mass == triangle area * areal_density, regardless of how
    // it got distributed/welded across sub-triangle corners.
    let total_mass: f64 = (0..world.particles.len())
        .map(|i| {
            if world.particles.inv_mass[i] > 0.0 {
                1.0 / world.particles.inv_mass[i]
            } else {
                0.0
            }
        })
        .sum();
    let triangle_area = 0.5; // right triangle, legs = 1.0
    let expected_mass = triangle_area * areal_density;
    assert!(
        (total_mass - expected_mass).abs() / expected_mass < 1e-9,
        "total_mass={} expected={}",
        total_mass,
        expected_mass
    );

    // Regular mesh: every sub-edge is either a "short" edge (length = leg/n) or the
    // hypotenuse-parallel "diagonal" edge (length = sqrt(2)/n).
    let expected_short = 1.0 / n as f64;
    let expected_diag = 2.0f64.sqrt() / n as f64;
    for c in &world.distance_constraints {
        let len = c.rest_length;
        let close_short = (len - expected_short).abs() < 1e-9;
        let close_diag = (len - expected_diag).abs() < 1e-9;
        assert!(close_short || close_diag, "unexpected sub-edge length {}", len);
    }
}

#[test]
fn fabric_mass_is_mesh_resolution_independent() {
    let areal_density = 0.2;
    let build = |n: usize| -> f64 {
        let mut def = empty_kite_def("mesh_indep");
        def.panels.push(PanelDef {
            name: "p0".to_string(),
            p1: DVec3::new(0.0, 0.0, 0.0),
            p2: DVec3::new(1.0, 0.0, 0.0),
            p3: DVec3::new(0.0, 0.0, 1.0),
            warp_compliance: 1e-6,
            weft_compliance: 1e-6,
            shear_compliance: 1e-6,
            bending_compliance: 1e-3,
            subdivisions: n,
            areal_density,
        });
        let mut world = World::new();
        build_kite_from_def(&mut world, &def);
        (0..world.particles.len())
            .map(|i| {
                if world.particles.inv_mass[i] > 0.0 {
                    1.0 / world.particles.inv_mass[i]
                } else {
                    0.0
                }
            })
            .sum()
    };

    let mass_n1 = build(1);
    let mass_n4 = build(4);
    assert!(
        (mass_n1 - mass_n4).abs() / mass_n1 < 1e-9,
        "mass_n1={} mass_n4={}",
        mass_n1,
        mass_n4
    );
}

// ---------------------------------------------------------------------------
// Feature 2: segmented bridle lines with mass and wind drag
// ---------------------------------------------------------------------------

fn bridle_chain_def() -> KiteDefinition {
    // Two anchor points closer together than the line's rest length, so the chain is
    // slack and free to sag/deflect like a hanging chain rather than pulled taut.
    let from = DVec3::new(-1.0, 0.0, 0.0);
    let to = DVec3::new(1.0, 0.0, 0.0);
    let mut def = empty_kite_def("bridle_chain");
    def.bridle_junction = to;
    def.bridle_junction_pinned = false; // pinned via pinned_points instead, see below
    def.bridles.push(BridleLineDef {
        name: "chain".to_string(),
        from,
        to,
        rest_length: 2.5, // > straight distance of 2.0 -> slack
        compliance: 1e-9,
        diameter: 0.003,
        num_segments: 8,
        density: 970.0,
    });
    def.pinned_points.push(from);
    def.pinned_points.push(to);
    def
}

#[test]
fn bridle_chain_sags_under_gravity() {
    let def = bridle_chain_def();
    let mut world = World::new();
    world.cfg.gravity = DVec3::new(0.0, -9.81, 0.0);
    build_kite_from_def(&mut world, &def);

    let mid_idx = find_particle_near(&world, DVec3::new(0.0, 0.0, 0.0), 0.2);

    let dt = 0.01;
    for _ in 0..500 {
        world.step(dt);
    }

    let mid_y = world.particles.pos[mid_idx].y;
    assert!(
        mid_y < -0.05,
        "expected midpoint to sag well below the straight chord (y=0), got y={}",
        mid_y
    );
}

#[test]
fn bridle_chain_deflects_downwind_under_crosswind() {
    let def = bridle_chain_def();
    let mut world = World::new();
    world.cfg.gravity = DVec3::ZERO; // isolate the wind-drag effect
    world.cfg.wind.v_ref = 20.0;
    world.cfg.wind.shear_exponent = 0.0; // uniform field, no height dependence
    world.cfg.wind.direction = DVec3::new(0.0, 0.0, 1.0); // crosswind, perpendicular to the line
    build_kite_from_def(&mut world, &def);

    let mid_idx = find_particle_near(&world, DVec3::new(0.0, 0.0, 0.0), 0.2);

    let dt = 0.01;
    for _ in 0..500 {
        world.step(dt);
    }

    let mid_z = world.particles.pos[mid_idx].z;
    assert!(
        mid_z > 0.05,
        "expected midpoint to deflect downwind (+Z) under crosswind, got z={}",
        mid_z
    );
}

// ---------------------------------------------------------------------------
// Feature 3: stiff spar junctions
// ---------------------------------------------------------------------------

fn t_joint_def(with_junction: bool) -> KiteDefinition {
    let mut def = empty_kite_def("t_joint");
    def.spars.push(SparDef {
        name: "vertical".to_string(),
        start: DVec3::new(0.0, -1.0, 0.0),
        end: DVec3::new(0.0, 1.0, 0.0),
        num_segments: 4,
        radius: 0.005,
        youngs_modulus: 4.0e10,
        shear_modulus: 4.0e9,
        density: 1950.0,
    });
    // Single-segment cantilever: keeps this a clean 2-body hinge test (one weld particle,
    // one free orientation DOF) rather than exercising the solver's separate, much slower
    // convergence for a multi-segment *unanchored* Cosserat chain settling under gravity
    // (a general solver characteristic, not specific to stiff junctions).
    def.spars.push(SparDef {
        name: "cantilever".to_string(),
        start: DVec3::new(0.0, 0.0, 0.0),
        end: DVec3::new(1.0, 0.0, 0.0),
        num_segments: 1,
        radius: 0.005,
        youngs_modulus: 4.0e10,
        shear_modulus: 4.0e9,
        density: 1950.0,
    });
    def.pinned_points.push(DVec3::new(0.0, -1.0, 0.0));
    def.pinned_points.push(DVec3::new(0.0, 1.0, 0.0));
    if with_junction {
        def.stiff_junctions.push(StiffJunctionDef {
            point: DVec3::new(0.0, 0.0, 0.0),
            compliance: 1e-10,
        });
    }
    def
}

#[test]
fn stiff_junction_holds_cantilever_near_build_height() {
    let tip_target = DVec3::new(1.0, 0.0, 0.0);

    let run = |with_junction: bool| -> f64 {
        let def = t_joint_def(with_junction);
        let mut world = World::new();
        world.cfg.gravity = DVec3::new(0.0, -9.81, 0.0);
        // The stiff junction and the cantilever's stretch-shear constraint both act
        // on the same orientation, and they pull against each other: the shear
        // constraint wants to rotate the frame to follow the segment tangent (it
        // gets w_rot/(w_rot + w_pos) = 80% of each correction here), while the
        // junction wants to hold that frame fixed. One Gauss-Seidel sweep per
        // substep does not resolve that coupling, and the unresolved residual shows
        // up as tip droop that has nothing to do with the junction's stiffness:
        // measured 0.0282, 0.0153, 0.0088, 0.0055, 0.0038, 0.0030 m for
        // iterations_per_substep = 1, 2, 4, 8, 16, 32. It is a solver convergence
        // rate, not a physical deflection — stiffening the supporting spar's E by
        // 10^4 moves it only from 0.0282 to 0.0258 m. So give the solve enough
        // sweeps to actually resolve the coupling before asserting a physical
        // property of the junction.
        world.cfg.iterations_per_substep = 4;
        build_kite_from_def(&mut world, &def);

        let tip_idx = find_particle_near(&world, tip_target, 0.05);

        let dt = 0.01;
        for _ in 0..500 {
            world.step(dt);
        }
        world.particles.pos[tip_idx].y
    };

    let tip_y_with = run(true).abs();
    let tip_y_without = run(false).abs();

    println!("tip droop with junction: {}, without: {}", tip_y_with, tip_y_without);

    assert!(
        tip_y_with < 0.01,
        "stiff junction should hold the cantilever tip near build height, drooped {}",
        tip_y_with
    );
    assert!(
        tip_y_without > tip_y_with * 10.0,
        "without the junction the cantilever should swing down substantially more: with={} without={}",
        tip_y_with,
        tip_y_without
    );
}

// ---------------------------------------------------------------------------
// Feature 4 (back-compat): existing scenario still parses and builds unchanged counts.
// ---------------------------------------------------------------------------

#[test]
fn simple_kite_v1_scenario_backcompat() {
    use std::fs::File;
    use std::io::Read;
    use serde::Deserialize;

    #[derive(Debug, Deserialize)]
    struct Scenario {
        kite: Option<KiteDefinition>,
    }

    let mut file = File::open("../../scenarios/simple_kite_v1.toml")
        .or_else(|_| File::open("scenarios/simple_kite_v1.toml"))
        .expect("failed to open scenario file");
    let mut toml_str = String::new();
    file.read_to_string(&mut toml_str).unwrap();

    let scenario: Scenario = toml::from_str(&toml_str).expect("old-format TOML must still parse");
    let kite_def = scenario.kite.expect("kite section present");

    // Confirm additive defaults kicked in (legacy scenario doesn't set these fields).
    assert!(kite_def.panels.iter().all(|p| p.subdivisions == 1));
    assert!(kite_def.bridles.iter().all(|b| b.num_segments == 1));
    assert!(kite_def.stiff_junctions.is_empty());

    let mut world = World::new();
    build_kite_from_def(&mut world, &kite_def);

    // Spine (5 nodes) + Cross-strut (5 nodes, sharing 1 weld at the origin) = 9,
    // plus the bridle junction = 10. All panel corners weld to existing spar nodes.
    assert_eq!(world.particles.len(), 10);
    assert_eq!(world.canopy_panels.len(), 12);
    assert_eq!(world.stretch_shear_constraints.len(), 8); // 2 spars * 4 segments
    assert_eq!(world.bend_twist_constraints.len(), 6); // 2 spars * 3 internal joints
    assert_eq!(world.unilateral_constraints.len(), 3); // 3 bridle lines, 1 segment each
}

// ---------------------------------------------------------------------------
// Feature 5: bridle-to-bridle attachments (cascaded bridle trees), order-independent
// ---------------------------------------------------------------------------

#[test]
fn bridle_to_bridle_knot_welds_regardless_of_listing_order() {
    let b_from = DVec3::new(0.0, 0.0, 0.0);
    let b_to = DVec3::new(0.0, -2.0, 0.0);
    let b_mid = b_from + (b_to - b_from) * 0.5; // (0, -1, 0), B's sole interior node
    let a_from = DVec3::new(-1.0, -1.0, 0.0);

    let mut def = empty_kite_def("cascade");
    def.bridle_junction = b_from;
    def.bridle_junction_pinned = true;

    // Attacher A is listed FIRST and targets B's not-yet-built interior knot.
    def.bridles.push(BridleLineDef {
        name: "A_attacher".to_string(),
        from: a_from,
        to: b_mid,
        rest_length: 1.0,
        compliance: 1e-9,
        diameter: 0.003,
        num_segments: 1,
        density: 970.0,
    });
    // B is listed SECOND; num_segments=2 gives it exactly one interior node, at its
    // midpoint, which is where A attaches.
    def.bridles.push(BridleLineDef {
        name: "B_base".to_string(),
        from: b_from,
        to: b_to,
        rest_length: 2.0,
        compliance: 1e-9,
        diameter: 0.003,
        num_segments: 2,
        density: 970.0,
    });

    def.pinned_points.push(a_from);
    def.pinned_points.push(b_to);

    let mut world = World::new();
    world.cfg.gravity = DVec3::new(0.0, -9.81, 0.0);
    build_kite_from_def(&mut world, &def);

    // Exactly one particle at the knot coordinate — no duplicate created because A was
    // built before B's interior node existed.
    let near_count = (0..world.particles.len())
        .filter(|&i| (world.particles.pos[i] - b_mid).length() < 1e-4)
        .count();
    assert_eq!(near_count, 1, "expected exactly one particle at the knot, found {}", near_count);

    let b_mid_idx = find_particle_near(&world, b_mid, 1e-4);
    let a_from_idx = find_particle_near(&world, a_from, 1e-4);

    // A's single-segment chain must terminate at B's interior node particle index.
    let a_chain_other = world
        .unilateral_constraints
        .iter()
        .find_map(|c| {
            if c.p1 == a_from_idx {
                Some(c.p2)
            } else if c.p2 == a_from_idx {
                Some(c.p1)
            } else {
                None
            }
        })
        .expect("expected a unilateral constraint touching A's anchor");
    assert_eq!(
        a_chain_other, b_mid_idx,
        "attacher's chain endpoint should weld to B's interior knot particle, not duplicate it"
    );

    // Small hanging arrangement: pinned at both outer ends, simulate and check stability.
    let dt = 0.01;
    for _ in 0..100 {
        world.step(dt);
    }
    for i in 0..world.particles.len() {
        assert!(world.particles.pos[i].is_finite(), "particle {} went non-finite", i);
    }
}

// ---------------------------------------------------------------------------
// Feature 6: spar-to-spar lashings
// ---------------------------------------------------------------------------

#[test]
fn lashing_holds_second_spar_at_fixed_separation() {
    let mut def = empty_kite_def("lashed_spars");
    def.spars.push(SparDef {
        name: "fixed".to_string(),
        start: DVec3::new(-0.5, 0.0, 0.0),
        end: DVec3::new(0.5, 0.0, 0.0),
        num_segments: 4,
        radius: 0.005,
        youngs_modulus: 4.0e10,
        shear_modulus: 4.0e9,
        density: 1950.0,
    });
    def.spars.push(SparDef {
        name: "hanging".to_string(),
        start: DVec3::new(-0.5, -0.05, 0.0),
        end: DVec3::new(0.5, -0.05, 0.0),
        num_segments: 4,
        radius: 0.005,
        youngs_modulus: 4.0e10,
        shear_modulus: 4.0e9,
        density: 1950.0,
    });
    def.lashings.push(LashingDef {
        point: DVec3::new(0.0, -0.025, 0.0),
        compliance: 1e-10,
    });
    def.pinned_points.push(DVec3::new(-0.5, 0.0, 0.0));
    def.pinned_points.push(DVec3::new(0.5, 0.0, 0.0));

    let mut world = World::new();
    world.cfg.gravity = DVec3::new(0.0, -9.81, 0.0);
    build_kite_from_def(&mut world, &def);

    let fixed_mid = find_particle_near(&world, DVec3::new(0.0, 0.0, 0.0), 1e-4);
    let hang_mid = find_particle_near(&world, DVec3::new(0.0, -0.05, 0.0), 1e-4);

    let dt = 0.01;
    for _ in 0..200 {
        world.step(dt);
    }

    let sep = (world.particles.pos[fixed_mid] - world.particles.pos[hang_mid]).length();
    assert!(
        (sep - 0.05).abs() / 0.05 < 0.10,
        "expected lashed nodes to stay ~5cm apart (free hang), got {}",
        sep
    );
}
