use glam::DVec3;
use kite_core::{
    build_kite_from_def, KiteDefinition, PanelDef, SparDef, UnilateralDistanceConstraint, World,
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

fn single_panel_kite_def() -> KiteDefinition {
    let mut def = empty_kite_def("panel_test");
    def.panels.push(PanelDef {
        name: "p0".to_string(),
        p1: DVec3::new(0.0, 0.0, 0.0),
        p2: DVec3::new(1.0, 0.0, 0.0),
        p3: DVec3::new(0.0, 0.0, 1.0),
        warp_compliance: 1e-8,
        weft_compliance: 1e-8,
        shear_compliance: 1e-8,
        bending_compliance: 1e-4,
        subdivisions: 1,
        areal_density: 0.05,
    });
    def
}

#[test]
fn last_h_set_after_step() {
    let mut world = World::new();
    let dt = 0.01;
    world.step(dt);
    let expected = dt / world.cfg.substeps as f64;
    assert!(
        (world.last_h - expected).abs() < 1e-15,
        "last_h = {}, expected {}",
        world.last_h,
        expected
    );
}

#[test]
fn aero_readback_matches_panels() {
    let mut world = World::new();
    world.cfg.gravity = DVec3::ZERO;
    world.cfg.wind.v_ref = 10.0;
    // 45 degree angle of attack on the flat panel in the XZ plane -> nonzero lift (alpha != 0, +-90deg)
    let angle: f64 = std::f64::consts::FRAC_PI_4;
    world.cfg.wind.direction = DVec3::new(angle.cos(), angle.sin(), 0.0);
    world.cfg.wind.shear_exponent = 0.0;

    let def = single_panel_kite_def();
    build_kite_from_def(&mut world, &def);

    for _ in 0..5 {
        world.step(0.01);
    }

    assert_eq!(world.aero_readback.len(), world.canopy_panels.len());

    let mut any_nonzero_lift = false;
    for panel_aero in &world.aero_readback {
        assert!(panel_aero.lift.is_finite());
        assert!(panel_aero.drag.is_finite());
        assert!(panel_aero.alpha.is_finite());
        if panel_aero.lift.length_squared() > 0.0 {
            any_nonzero_lift = true;
        }
    }
    assert!(any_nonzero_lift, "expected at least one panel with nonzero lift");
}

#[test]
fn pinned_point_welds_existing_node() {
    let mut world_baseline = World::new();
    let mut def = empty_kite_def("weld_test");
    def.spars.push(SparDef {
        name: "s0".to_string(),
        start: DVec3::new(0.0, 0.0, 0.0),
        end: DVec3::new(1.0, 0.0, 0.0),
        num_segments: 1,
        radius: 0.01,
        youngs_modulus: 1.0e9,
        shear_modulus: 5.0e8,
        density: 1000.0,
    });
    build_kite_from_def(&mut world_baseline, &def);
    let baseline_count = world_baseline.particles.len();

    let mut world_pinned = World::new();
    def.pinned_points.push(DVec3::new(0.0002, 0.0, 0.0)); // within 1mm of spar start
    build_kite_from_def(&mut world_pinned, &def);

    assert_eq!(world_pinned.particles.len(), baseline_count, "weld should not add a particle");

    let idx = (0..world_pinned.particles.len())
        .find(|&i| (world_pinned.particles.pos[i] - DVec3::new(0.0, 0.0, 0.0)).length() < 1e-3)
        .expect("welded particle not found");
    assert_eq!(world_pinned.particles.inv_mass[idx], 0.0);
}

#[test]
fn pinned_point_creates_isolated_anchor() {
    let mut def = empty_kite_def("anchor_test");
    def.spars.push(SparDef {
        name: "s0".to_string(),
        start: DVec3::new(0.0, 0.0, 0.0),
        end: DVec3::new(1.0, 0.0, 0.0),
        num_segments: 1,
        radius: 0.01,
        youngs_modulus: 1.0e9,
        shear_modulus: 5.0e8,
        density: 1000.0,
    });

    let mut world_baseline = World::new();
    build_kite_from_def(&mut world_baseline, &def);
    let baseline_count = world_baseline.particles.len();

    let far_point = DVec3::new(100.0, 100.0, 100.0);
    def.pinned_points.push(far_point);
    let mut world_pinned = World::new();
    build_kite_from_def(&mut world_pinned, &def);

    assert_eq!(world_pinned.particles.len(), baseline_count + 1);
    let idx = (0..world_pinned.particles.len())
        .find(|&i| (world_pinned.particles.pos[i] - far_point).length() < 1e-6)
        .expect("new anchor particle not found");
    assert_eq!(world_pinned.particles.inv_mass[idx], 0.0);
}

#[test]
fn kite_def_toml_backcompat() {
    let toml_no_pinned = r#"
        name = "backcompat"
        spars = []
        panels = []
        bridles = []
        bridle_junction = [0.0, 0.0, 0.0]
        bridle_junction_pinned = false
    "#;
    let def: KiteDefinition = toml::from_str(toml_no_pinned).expect("old-format TOML must still parse");
    assert!(def.pinned_points.is_empty());

    let serialized = toml::to_string(&def).expect("serialize");
    assert!(
        !serialized.contains("pinned_points"),
        "empty pinned_points must not be serialized: {}",
        serialized
    );
}

/// THE CRITICAL TEST: pins the sign convention of unilateral constraint tension empirically.
/// A pinned particle A at (0,1,0) holds a taut line to a 1kg particle B hanging at the origin
/// under gravity. At static equilibrium the line tension must equal exactly m*g = 9.81 N.
#[test]
fn bridle_tension_sign_and_magnitude() {
    let mut world = World::new();
    world.cfg.substeps = 24;
    world.cfg.iterations_per_substep = 1;
    world.cfg.gravity = DVec3::new(0.0, -9.81, 0.0);
    world.cfg.damping = 2.0;
    // no wind: default WindConfig has v_ref = 0.0

    let a = world.add_particle(DVec3::new(0.0, 1.0, 0.0), 0.0); // pinned anchor
    let b = world.add_particle(DVec3::new(0.0, 0.0, 0.0), 1.0); // 1 kg hanging mass

    world
        .unilateral_constraints
        .push(UnilateralDistanceConstraint::new(a, b, 0.9, 1e-9, 0.003));

    let dt = 0.01;
    for _ in 0..300 {
        world.step(dt);
    }

    let c = &world.unilateral_constraints[0];
    let h = world.last_h;
    let tension = -c.lambda / (h * h);

    println!(
        "lambda = {}, h = {}, tension = {} N (expected ~9.81 N)",
        c.lambda, h, tension
    );

    assert!(tension > 0.0, "tension must be positive, got {}", tension);
    let ratio = tension / 9.81;
    assert!(
        (ratio - 1.0).abs() < 0.10,
        "tension {} N not within 10% of expected 9.81 N (ratio {})",
        tension,
        ratio
    );
}
