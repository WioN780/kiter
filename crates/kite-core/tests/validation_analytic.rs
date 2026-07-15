use kite_core::{World, DistanceConstraint, BendingConstraint};
use glam::DVec3;

#[test]
fn test_smoke_empty_world() {
    let mut world = World::new();
    let dt = 0.01;
    for _ in 0..1000 {
        world.step(dt);
    }
    // Check that time advanced correctly (within floating point precision)
    let expected_time = 10.0;
    assert!(
        (world.time - expected_time).abs() < 1e-9,
        "Time did not advance correctly: expected {}, got {}",
        expected_time,
        world.time
    );
}

#[test]
fn test_hanging_chain_catenary() {
    let mut world = World::new();
    world.cfg.substeps = 50;
    world.cfg.iterations_per_substep = 1;
    world.cfg.gravity = DVec3::new(0.0, -9.81, 0.0);
    world.cfg.damping = 5.0; // High damping to settle quickly

    let span = 4.0;
    let length = 5.0;
    let n_segments = 20;
    let n_particles = n_segments + 1;
    let segment_rest_length = length / n_segments as f64;

    let x0 = span / 2.0;

    // Initialize particles along a simple sagging curve to avoid unstable vertical equilibrium
    for i in 0..n_particles {
        let t = i as f64 / n_segments as f64; // 0.0 to 1.0
        let x = -x0 + t * span;
        let y = -(1.0 - (x / x0).powi(2));
        let is_pinned = i == 0 || i == n_segments;
        let mass = if is_pinned { 0.0 } else { 1.0 };
        world.particles.add_particle(DVec3::new(x, y, 0.0), mass);
    }

    // Add distance constraints
    for i in 0..n_segments {
        world.distance_constraints.push(DistanceConstraint::new(
            i,
            i + 1,
            segment_rest_length,
            1e-8, // Almost inextensible
        ));
    }

    // Run the simulation to settle
    let dt = 0.01;
    for _ in 0..1000 {
        world.step(dt);
    }

    // Solve for analytic catenary parameter a: 2 * a * sinh(x0 / a) = length
    let a = {
        let mut low = 0.01;
        let mut high = 100.0;
        for _ in 0..100 {
            let mid = 0.5 * (low + high);
            let val = 2.0 * mid * (x0 / mid).sinh();
            if val > length {
                low = mid;
            } else {
                high = mid;
            }
        }
        0.5 * (low + high)
    };

    let y_offset = -a * (x0 / a).cosh();

    // Verify each free particle's y position matches the analytical catenary
    for i in 1..n_segments {
        let pos = world.particles.pos[i];
        let x = pos.x;
        let y_analytic = a * (x / a).cosh() + y_offset;
        let diff = (pos.y - y_analytic).abs();
        assert!(
            diff < 0.03,
            "Particle {} at x = {} has y = {}, expected analytic y = {} (diff = {})",
            i,
            x,
            pos.y,
            y_analytic,
            diff
        );
    }
}

#[test]
fn test_cantilever_rod_deflection() {
    let mut world = World::new();
    world.cfg.substeps = 30;
    world.cfg.iterations_per_substep = 2; // Extra iteration sweeps for faster convergence
    world.cfg.gravity = DVec3::new(0.0, -9.81, 0.0);
    world.cfg.damping = 6.0; // High damping to settle quickly

    let length = 2.0;
    let n_segments = 10;
    let n_particles = n_segments + 1;
    let dx = length / n_segments as f64;

    let total_mass = 1.0;
    let particle_mass = total_mass / n_segments as f64;

    let ei = 10.0; // Bending stiffness E * I
    let compliance_bending = dx.powi(3) / ei;

    // Initialize particles horizontally
    for i in 0..n_particles {
        let x = i as f64 * dx;
        // Pinned boundary condition at left end:
        // Pin both particle 0 and particle 1 horizontally to enforce horizontal clamped slope.
        let is_pinned = i == 0 || i == 1;
        let mass = if is_pinned { 0.0 } else { particle_mass };
        world.particles.add_particle(DVec3::new(x, 0.0, 0.0), mass);
    }

    // Add distance constraints (inextensible segments)
    for i in 0..n_segments {
        world.distance_constraints.push(DistanceConstraint::new(
            i,
            i + 1,
            dx,
            1e-10, // nearly rigid
        ));
    }

    // Add bending constraints
    for i in 1..n_segments {
        let p1 = i - 1;
        let p2 = i;
        let p3 = i + 1;
        // Rest state is straight horizontally
        let rest_pos1 = DVec3::new((i - 1) as f64 * dx, 0.0, 0.0);
        let rest_pos2 = DVec3::new(i as f64 * dx, 0.0, 0.0);
        let rest_pos3 = DVec3::new((i + 1) as f64 * dx, 0.0, 0.0);
        let rest_val = rest_pos1 - 2.0 * rest_pos2 + rest_pos3; // DVec3::ZERO
        world.bending_constraints.push(BendingConstraint::new(
            p1,
            p2,
            p3,
            rest_val,
            compliance_bending,
        ));
    }

    // Run simulation to settle
    let dt = 0.01;
    for _ in 0..1500 {
        world.step(dt);
    }

    // Tip deflection (y coordinate of the last particle)
    let tip_y = world.particles.pos[n_segments].y;
    let tip_deflection = -tip_y;

    // Euler-Bernoulli estimate: delta = (W * L^3) / (8 * E * I)
    // where W = total weight = M * g.
    let w_weight = total_mass * 9.81;
    let expected_deflection = (w_weight * length.powi(3)) / (8.0 * ei);

    // Allow some loose tolerance due to discretization of point-mass chain (e.g. 25% due to clamping 2 endpoints)
    let ratio = tip_deflection / expected_deflection;
    println!("Tip deflection: actual={}, expected={}, ratio={}", tip_deflection, expected_deflection, ratio);

    assert!(
        (ratio - 1.0).abs() < 0.25,
        "Tip deflection {} is not in the ballpark of expected {} (ratio {})",
        tip_deflection,
        expected_deflection,
        ratio
    );
}

#[test]
fn test_energy_conservation() {
    let mut world = World::new();
    world.cfg.substeps = 100;
    world.cfg.iterations_per_substep = 1;
    world.cfg.gravity = DVec3::new(0.0, -9.81, 0.0);
    world.cfg.damping = 0.0; // NO damping

    // Double pendulum-like setup
    // P0 pinned, P1 and P2 free
    world.particles.add_particle(DVec3::new(0.0, 0.0, 0.0), 0.0); // Pinned
    world.particles.add_particle(DVec3::new(1.0, 0.0, 0.0), 1.0); // Free
    world.particles.add_particle(DVec3::new(2.0, 0.0, 0.0), 1.0); // Free

    // Distance constraints
    world.distance_constraints.push(DistanceConstraint::new(0, 1, 1.0, 1e-5));
    world.distance_constraints.push(DistanceConstraint::new(1, 2, 1.0, 1e-5));

    // Bending constraint
    world.bending_constraints.push(BendingConstraint::new(
        0,
        1,
        2,
        DVec3::ZERO,
        1e-4,
    ));

    let initial_energy = world.compute_total_energy();
    // At rest, initial energy should be 0.0
    assert!((initial_energy - 0.0).abs() < 1e-5);

    let dt = 0.01;
    for step in 0..300 {
        world.step(dt);
        let energy = world.compute_total_energy();
        let drift = (energy - initial_energy).abs();

        // Assert that energy is conserved within a small tolerance
        assert!(
            drift < 0.20,
            "Step {}: Energy drifted to {}, initial = {} (drift = {})",
            step,
            energy,
            initial_energy,
            drift
        );
    }
}

fn test_helper_scaled_axis(q: glam::DQuat) -> DVec3 {
    let xyz = q.xyz();
    let len = xyz.length();
    if len < 1e-12 {
        DVec3::ZERO
    } else {
        let theta = 2.0 * len.atan2(q.w);
        let theta = if theta > std::f64::consts::PI {
            theta - 2.0 * std::f64::consts::PI
        } else if theta < -std::f64::consts::PI {
            theta + 2.0 * std::f64::consts::PI
        } else {
            theta
        };
        (xyz / len) * theta
    }
}

#[test]
fn test_cantilever_cosserat_rod() {
    use kite_core::{StretchShearConstraint, BendTwistConstraint, Material, SectionGeometry};
    use kite_core::materials::{stretch_shear_compliance, bend_twist_compliance};

    let mut world = World::new();
    world.cfg.substeps = 30;
    world.cfg.iterations_per_substep = 2;
    world.cfg.gravity = DVec3::new(0.0, -9.81, 0.0);
    world.cfg.damping = 4.0;

    let length = 2.0;
    let n_segments = 10;
    let n_particles = n_segments + 1;
    let dx = length / n_segments as f64;

    let radius = 0.02;
    let density = 1000.0;
    let geom = SectionGeometry::SolidRound { radius };
    let area = geom.area();
    let segment_mass = density * area * dx; // ~0.2513 kg

    let material = Material {
        youngs_modulus: 1.0e9,
        shear_modulus: 5.0e8,
        density,
        tensile_strength: 1.0e9,
    };

    // 1. Initialize particles extending along Z-axis
    for i in 0..n_particles {
        let z = i as f64 * dx;
        let is_pinned = i == 0; // pin root position
        let mass = if is_pinned { 0.0 } else { segment_mass };
        world.particles.add_particle(DVec3::new(0.0, 0.0, z), mass);
    }

    // 2. Initialize orientations
    let inertia = geom.compute_inertia(dx, segment_mass);
    for i in 0..n_segments {
        let is_pinned = i == 0; // clamp root orientation
        let inv_inertia = if is_pinned { DVec3::ZERO } else { DVec3::new(1.0 / inertia.x, 1.0 / inertia.y, 1.0 / inertia.z) };
        world.add_segment(glam::DQuat::IDENTITY, inv_inertia);
    }

    // 3. Add stretch-shear constraints
    let comp_ss = stretch_shear_compliance(&material, &geom, dx);
    for i in 0..n_segments {
        world.stretch_shear_constraints.push(StretchShearConstraint::new(
            i,
            i + 1,
            i,
            dx,
            comp_ss,
            2.0 * radius,
        ));
    }

    // 4. Add bend-twist constraints
    let comp_bt = bend_twist_compliance(&material, &geom, dx);
    for i in 0..n_segments - 1 {
        world.bend_twist_constraints.push(BendTwistConstraint::new(
            i,
            i + 1,
            DVec3::ZERO, // straight rest state
            comp_bt,
            f64::MAX, // no yield
            false,    // solid round
        ));
    }

    // Run simulation to settle
    let dt = 0.01;
    for _ in 0..1200 {
        world.step(dt);
    }

    let tip_y = world.particles.pos[n_segments].y;
    let tip_deflection = -tip_y;

    // Euler-Bernoulli deflection = (W * L^3) / (8 * E * I)
    let total_mass = segment_mass * n_segments as f64;
    let w_weight = total_mass * 9.81;
    let ei = material.youngs_modulus * geom.area_moment_of_inertia();
    let expected_deflection = (w_weight * length.powi(3)) / (8.0 * ei);

    let ratio = tip_deflection / expected_deflection;
    println!("Cosserat tip deflection: actual={}, expected={}, ratio={}", tip_deflection, expected_deflection, ratio);

    // With 10 segment Cosserat rod, deflection is extremely close to analytical (within 9% in both sequential and parallel modes)
    assert!(
        (ratio - 1.0).abs() < 0.09,
        "Cosserat tip deflection {} is not close to expected {} (ratio {})",
        tip_deflection,
        expected_deflection,
        ratio
    );
}

#[test]
fn test_torsion_cosserat_rod() {
    use kite_core::{StretchShearConstraint, BendTwistConstraint, Material, SectionGeometry};
    use kite_core::materials::{stretch_shear_compliance, bend_twist_compliance};

    let mut world = World::new();
    world.cfg.substeps = 40;
    world.cfg.iterations_per_substep = 2;
    world.cfg.gravity = DVec3::ZERO; // no gravity
    world.cfg.damping = 5.0;

    let length = 1.0;
    let n_segments = 5;
    let n_particles = n_segments + 1;
    let dx = length / n_segments as f64;

    let radius = 0.02;
    let density = 1000.0;
    let geom = SectionGeometry::SolidRound { radius };
    let area = geom.area();
    let segment_mass = density * area * dx;

    let material = Material {
        youngs_modulus: 1.0e9,
        shear_modulus: 5.0e8,
        density,
        tensile_strength: 1.0e9,
    };

    // Initialize particles along Z
    for i in 0..n_particles {
        let z = i as f64 * dx;
        let is_pinned = i == 0;
        let mass = if is_pinned { 0.0 } else { segment_mass };
        world.particles.add_particle(DVec3::new(0.0, 0.0, z), mass);
    }

    // Initialize orientations (clamp segment 0)
    let inertia = geom.compute_inertia(dx, segment_mass);
    for i in 0..n_segments {
        let is_pinned = i == 0;
        let inv_inertia = if is_pinned { DVec3::ZERO } else { DVec3::new(1.0 / inertia.x, 1.0 / inertia.y, 1.0 / inertia.z) };
        world.add_segment(glam::DQuat::IDENTITY, inv_inertia);
    }

    // Constraints
    let comp_ss = stretch_shear_compliance(&material, &geom, dx);
    for i in 0..n_segments {
        world.stretch_shear_constraints.push(StretchShearConstraint::new(i, i + 1, i, dx, comp_ss, 2.0 * radius));
    }

    let comp_bt = bend_twist_compliance(&material, &geom, dx);
    for i in 0..n_segments - 1 {
        world.bend_twist_constraints.push(BendTwistConstraint::new(
            i,
            i + 1,
            DVec3::ZERO,
            comp_bt,
            f64::MAX,
            false,
        ));
    }

    // Apply known torsion torque to the last segment
    let torque_z = 100.0;
    world.torques[n_segments - 1] = DVec3::new(0.0, 0.0, torque_z);

    // Step simulation to settle
    let dt = 0.01;
    for _ in 0..500 {
        world.step(dt);
    }

    // Resulting twist angle on the last segment
    let last_q = world.orientations.quat[n_segments - 1];
    let twist_angle = test_helper_scaled_axis(last_q).z;

    // Theoretical twist: theta = T * L / (G * J)
    let j_polar = geom.polar_moment_of_inertia();
    let expected_twist = (torque_z * length) / (material.shear_modulus * j_polar);

    let ratio = twist_angle / expected_twist;
    println!("Twist angle: actual={}, expected={}, ratio={}", twist_angle, expected_twist, ratio);

    assert!(
        (ratio - 1.0).abs() < 0.10,
        "Twist angle {} is not close to expected {} (ratio {})",
        twist_angle,
        expected_twist,
        ratio
    );
}

#[test]
fn test_breaking_cosserat_rod() {
    use kite_core::{StretchShearConstraint, BendTwistConstraint, Material, SectionGeometry, ConstraintState, Event};
    use kite_core::materials::{stretch_shear_compliance, bend_twist_compliance};

    // Case 1: Solid rod snaps
    {
        let mut world = World::new();
        world.cfg.substeps = 20;
        world.cfg.iterations_per_substep = 2;
        world.cfg.gravity = DVec3::ZERO;
        world.cfg.damping = 2.0;

        world.particles.add_particle(DVec3::new(0.0, 0.0, 0.0), 0.0);
        world.particles.add_particle(DVec3::new(0.0, 0.0, 0.5), 0.1);
        world.particles.add_particle(DVec3::new(0.0, 0.0, 1.0), 0.1);

        world.add_segment(glam::DQuat::IDENTITY, DVec3::ZERO); // clamp segment 0
        world.add_segment(glam::DQuat::IDENTITY, DVec3::ONE);

        let material = Material::fiberglass();
        let geom = SectionGeometry::SolidRound { radius: 0.01 };
        let comp_ss = stretch_shear_compliance(&material, &geom, 0.5);
        world.stretch_shear_constraints.push(StretchShearConstraint::new(0, 1, 0, 0.5, comp_ss, 0.02));
        world.stretch_shear_constraints.push(StretchShearConstraint::new(1, 2, 1, 0.5, comp_ss, 0.02));

        let comp_bt = bend_twist_compliance(&material, &geom, 0.5);
        // Low yield threshold
        world.bend_twist_constraints.push(BendTwistConstraint::new(
            0,
            1,
            DVec3::ZERO,
            comp_bt,
            0.01,  // yield threshold
            false, // is_inflatable = false (solid)
        ));

        // Apply a huge torque to exceed the threshold
        world.torques[1] = DVec3::new(100.0, 0.0, 0.0);

        world.step(0.01);

        // Check state
        assert_eq!(world.bend_twist_constraints[0].state, ConstraintState::Broken);
        assert!(world.events.contains(&Event::SparBroken { joint_index: 0 }));
    }

    // Case 2: Inflatable tube folds
    {
        let mut world = World::new();
        world.cfg.substeps = 20;
        world.cfg.iterations_per_substep = 2;
        world.cfg.gravity = DVec3::ZERO;
        world.cfg.damping = 2.0;

        world.particles.add_particle(DVec3::new(0.0, 0.0, 0.0), 0.0);
        world.particles.add_particle(DVec3::new(0.0, 0.0, 0.5), 0.1);
        world.particles.add_particle(DVec3::new(0.0, 0.0, 1.0), 0.1);

        world.add_segment(glam::DQuat::IDENTITY, DVec3::ZERO); // clamp segment 0
        world.add_segment(glam::DQuat::IDENTITY, DVec3::ONE);

        let material = Material::fiberglass();
        let geom = SectionGeometry::SolidRound { radius: 0.01 };
        let comp_ss = stretch_shear_compliance(&material, &geom, 0.5);
        world.stretch_shear_constraints.push(StretchShearConstraint::new(0, 1, 0, 0.5, comp_ss, 0.02));
        world.stretch_shear_constraints.push(StretchShearConstraint::new(1, 2, 1, 0.5, comp_ss, 0.02));

        let comp_bt = bend_twist_compliance(&material, &geom, 0.5);
        // Low yield threshold
        world.bend_twist_constraints.push(BendTwistConstraint::new(
            0,
            1,
            DVec3::ZERO,
            comp_bt,
            0.01,  // yield threshold
            true,  // is_inflatable = true (should fold)
        ));

        // Apply a huge torque to exceed the threshold
        world.torques[1] = DVec3::new(100.0, 0.0, 0.0);

        world.step(0.01);

        // Check state
        assert_eq!(world.bend_twist_constraints[0].state, ConstraintState::Folded);
        assert!(world.events.contains(&Event::LeadingEdgeFolded { joint_index: 0 }));
    }
}

#[test]
fn test_cloth_stretch_equilibrium() {
    use kite_core::DistanceConstraint;

    let mut world = World::new();
    world.cfg.substeps = 30;
    world.cfg.iterations_per_substep = 2;
    world.cfg.gravity = DVec3::ZERO;
    world.cfg.damping = 5.0;

    // 1D warp-aligned strip: p0 (pinned) -> p1 -> p2
    world.add_particle(DVec3::new(0.0, 0.0, 0.0), 0.0);
    world.add_particle(DVec3::new(1.0, 0.0, 0.0), 1.0);
    world.add_particle(DVec3::new(2.0, 0.0, 0.0), 1.0);

    let compliance = 0.05;
    world.distance_constraints.push(DistanceConstraint::new(0, 1, 1.0, compliance));
    world.distance_constraints.push(DistanceConstraint::new(1, 2, 1.0, compliance));

    // Apply known force on the end
    let force_x = 10.0;
    world.forces[2] = DVec3::new(force_x, 0.0, 0.0);

    // Run to settle
    let dt = 0.01;
    for _ in 0..1000 {
        world.step(dt);
    }

    let p1_x = world.particles.pos[1].x;
    let p2_x = world.particles.pos[2].x;

    let ext1 = p1_x - 1.0;
    let ext2 = p2_x - p1_x - 1.0;

    let expected_ext = compliance * force_x; // 0.5

    println!("Extension 1: actual={}, expected={}", ext1, expected_ext);
    println!("Extension 2: actual={}, expected={}", ext2, expected_ext);

    assert!((ext1 - expected_ext).abs() < 1e-3);
    assert!((ext2 - expected_ext).abs() < 1e-3);
}

#[test]
fn test_flat_sheet_drop() {
    use kite_core::{DistanceConstraint, DihedralBendingConstraint, compute_dihedral_angle};

    let mut world = World::new();
    world.cfg.substeps = 30;
    world.cfg.iterations_per_substep = 2;
    world.cfg.gravity = DVec3::new(0.0, -9.81, 0.0);
    world.cfg.damping = 4.0;

    let m = 4;
    let n = 4;
    let dx = 0.2;
    let dz = 0.2;
    let get_idx = |c: usize, r: usize| c + r * m;

    // Add particles in a horizontal grid
    for r in 0..n {
        for c in 0..m {
            let pos = DVec3::new(c as f64 * dx, 0.0, r as f64 * dz);
            // Pin the row r = 0
            let is_pinned = r == 0;
            let mass = if is_pinned { 0.0 } else { 0.1 };
            world.add_particle(pos, mass);
        }
    }

    // Add warp constraints (horizontal)
    for r in 0..n {
        for c in 0..m - 1 {
            let p1 = get_idx(c, r);
            let p2 = get_idx(c + 1, r);
            world.distance_constraints.push(DistanceConstraint::new(p1, p2, dx, 0.01));
        }
    }

    // Add weft constraints (vertical)
    for r in 0..n - 1 {
        for c in 0..m {
            let p1 = get_idx(c, r);
            let p2 = get_idx(c, r + 1);
            world.distance_constraints.push(DistanceConstraint::new(p1, p2, dz, 0.01));
        }
    }

    // Add shear constraints (diagonals)
    let diag_len = (dx * dx + dz * dz).sqrt();
    for r in 0..n - 1 {
        for c in 0..m - 1 {
            let p00 = get_idx(c, r);
            let p10 = get_idx(c + 1, r);
            let p01 = get_idx(c, r + 1);
            let p11 = get_idx(c + 1, r + 1);

            world.distance_constraints.push(DistanceConstraint::new(p00, p11, diag_len, 0.05));
            world.distance_constraints.push(DistanceConstraint::new(p10, p01, diag_len, 0.05));
        }
    }

    // Add dihedral bending constraints
    // 1. Diagonal shared edge inside each quad
    for r in 0..n - 1 {
        for c in 0..m - 1 {
            let p00 = get_idx(c, r);
            let p10 = get_idx(c + 1, r);
            let p01 = get_idx(c, r + 1);
            let p11 = get_idx(c + 1, r + 1);

            let p1_pos = world.particles.pos[p00];
            let p2_pos = world.particles.pos[p11];
            let p3_pos = world.particles.pos[p10];
            let p4_pos = world.particles.pos[p01];
            let rest = compute_dihedral_angle(p1_pos, p2_pos, p3_pos, p4_pos);
            world.dihedral_bending_constraints.push(DihedralBendingConstraint::new(p00, p11, p10, p01, rest, 0.1));
        }
    }

    // 2. Vertical shared edge between adjacent quads in X
    for r in 0..n - 1 {
        for c in 0..m - 2 {
            let p10 = get_idx(c + 1, r);
            let p11 = get_idx(c + 1, r + 1);
            let p00 = get_idx(c, r);
            let p21 = get_idx(c + 2, r + 1);

            let p1_pos = world.particles.pos[p10];
            let p2_pos = world.particles.pos[p11];
            let p3_pos = world.particles.pos[p00];
            let p4_pos = world.particles.pos[p21];
            let rest = compute_dihedral_angle(p1_pos, p2_pos, p3_pos, p4_pos);
            world.dihedral_bending_constraints.push(DihedralBendingConstraint::new(p10, p11, p00, p21, rest, 0.1));
        }
    }

    // 3. Horizontal shared edge between adjacent quads in Y
    for r in 0..n - 2 {
        for c in 0..m - 1 {
            let p01 = get_idx(c, r + 1);
            let p11 = get_idx(c + 1, r + 1);
            let p00 = get_idx(c, r);
            let p12 = get_idx(c + 1, r + 2);

            let p1_pos = world.particles.pos[p01];
            let p2_pos = world.particles.pos[p11];
            let p3_pos = world.particles.pos[p00];
            let p4_pos = world.particles.pos[p12];
            let rest = compute_dihedral_angle(p1_pos, p2_pos, p3_pos, p4_pos);
            world.dihedral_bending_constraints.push(DihedralBendingConstraint::new(p01, p11, p00, p12, rest, 0.1));
        }
    }

    // Verify initial energy
    let initial_energy = world.compute_total_energy();
    println!("Initial energy: {}", initial_energy);

    // Run simulation to settle
    let dt = 0.01;
    for _ in 0..500 {
        world.step(dt);
    }

    let final_energy = world.compute_total_energy();
    println!("Final energy: {}", final_energy);

    // Verify sheet hangs down and did not explode (no NaN, finite positions)
    for i in 0..world.particles.len() {
        let pos = world.particles.pos[i];
        assert!(pos.is_finite(), "Particle {} position is not finite: {:?}", i, pos);
        if i >= m {
            // Non-pinned particles should have dropped downwards (y < 0.0)
            assert!(pos.y < 0.0, "Particle {} did not drop: {:?}", i, pos);
        }
    }
}

#[test]
fn test_dihedral_gradient_numerical() {
    use kite_core::compute_dihedral_angle;

    // Define 4 points not coplanar
    let p1 = DVec3::new(0.0, 0.0, 0.0);
    let p2 = DVec3::new(1.0, 0.0, 0.0);
    let p3 = DVec3::new(0.0, 0.1, 1.0);
    let p4 = DVec3::new(0.0, 0.2, -1.0);

    let e = p2 - p1;
    let e_len = e.length();

    let e1 = e.cross(p3 - p1);
    let e2 = (p4 - p1).cross(e);

    let n1 = e1.normalize();
    let n2 = e2.normalize();

    let h1 = e1.length() / e_len;
    let h2 = e2.length() / e_len;

    // Let's print different sign options for q4
    let q3_analytical = n1 / h1;
    let q4_analytical_pos = n2 / h2;
    let q4_analytical_neg = -n2 / h2;

    // Finite difference gradients
    let eps = 1e-6;
    let mut grad_p3_fd = DVec3::ZERO;
    for c in 0..3 {
        let mut p3_plus = p3;
        p3_plus[c] += eps;
        let angle_plus = compute_dihedral_angle(p1, p2, p3_plus, p4);
        let mut p3_minus = p3;
        p3_minus[c] -= eps;
        let angle_minus = compute_dihedral_angle(p1, p2, p3_minus, p4);
        grad_p3_fd[c] = (angle_plus - angle_minus) / (2.0 * eps);
    }

    let mut grad_p4_fd = DVec3::ZERO;
    for c in 0..3 {
        let mut p4_plus = p4;
        p4_plus[c] += eps;
        let angle_plus = compute_dihedral_angle(p1, p2, p3, p4_plus);
        let mut p4_minus = p4;
        p4_minus[c] -= eps;
        let angle_minus = compute_dihedral_angle(p1, p2, p3, p4_minus);
        grad_p4_fd[c] = (angle_plus - angle_minus) / (2.0 * eps);
    }

    println!("--- DIHEDRAL GRADIENT DEBUG ---");
    println!("q3 analytical: {:?}", q3_analytical);
    println!("q3 numerical (FD): {:?}", grad_p3_fd);
    println!("q4 analytical (pos): {:?}", q4_analytical_pos);
    println!("q4 analytical (neg): {:?}", q4_analytical_neg);
    println!("q4 numerical (FD): {:?}", grad_p4_fd);
    println!("--------------------------------");

    // Check matching
    assert!((q3_analytical - grad_p3_fd).length() < 1e-3 || (-q3_analytical - grad_p3_fd).length() < 1e-3);
}

#[test]
fn test_unilateral_bridle_slack() {
    use kite_core::UnilateralDistanceConstraint;

    let mut world = World::new();
    world.cfg.substeps = 20;
    world.cfg.iterations_per_substep = 2;
    world.cfg.gravity = DVec3::ZERO;
    world.cfg.damping = 2.0;

    world.add_particle(DVec3::new(0.0, 0.0, 0.0), 0.0); // pinned
    world.add_particle(DVec3::new(1.0, 0.0, 0.0), 1.0); // free

    world.unilateral_constraints.push(UnilateralDistanceConstraint::new(0, 1, 1.5, 0.001, 0.002));

    // 1. Slack test: initial distance 1.0 < rest_length 1.5
    world.step(0.01);
    assert_eq!(world.unilateral_constraints[0].lambda, 0.0);

    // 2. Compress test: push towards anchor
    world.forces[1] = DVec3::new(-10.0, 0.0, 0.0);
    world.step(0.01);
    assert_eq!(world.unilateral_constraints[0].lambda, 0.0);

    // 3. Tension test: pull away from anchor past 1.5m
    world.forces[1] = DVec3::new(100.0, 0.0, 0.0);
    for _ in 0..50 {
        world.step(0.01);
    }
    assert!(world.unilateral_constraints[0].lambda < 0.0); // must be in tension (negative lambda)
}

#[test]
fn test_branching_bridle_tension() {
    use kite_core::UnilateralDistanceConstraint;

    let mut world = World::new();
    world.cfg.substeps = 40;
    world.cfg.iterations_per_substep = 10; // high iterations to resolve branching network precisely
    world.cfg.gravity = DVec3::ZERO;
    world.cfg.damping = 10.0; // high damping to settle quickly
    world.cfg.wind.v_ref = 10.0;
    world.cfg.wind.direction = DVec3::new(1.0, 0.0, 0.0);
    world.cfg.wind.shear_exponent = 0.0;

    // Anchors
    world.add_particle(DVec3::new(-1.0, 1.0, 0.0), 0.0); // 0: anchor 1
    world.add_particle(DVec3::new(1.0, 1.0, 0.0), 0.0);  // 1: anchor 2
    // Junction
    world.add_particle(DVec3::new(0.0, 0.0, 0.0), 1.0);  // 2: junction
    // End load point
    world.add_particle(DVec3::new(0.0, -1.0, 0.0), 1.0); // 3: load point

    let sqrt_2 = 2.0_f64.sqrt();
    let comp = 1.0e-6; // stiff Dyneema-like compliance
    let diameter = 0.003; // 3mm line

    // Branching bridle: two legs converging to a main line
    world.unilateral_constraints.push(UnilateralDistanceConstraint::new(0, 2, sqrt_2, comp, diameter)); // left leg (index 0)
    world.unilateral_constraints.push(UnilateralDistanceConstraint::new(1, 2, sqrt_2, comp, diameter)); // right leg (index 1)
    world.unilateral_constraints.push(UnilateralDistanceConstraint::new(2, 3, 1.0, comp, diameter));    // main line (index 2)

    // Apply downward load to Particle 3
    world.forces[3] = DVec3::new(0.0, -100.0, 0.0);

    // Settle
    let dt = 0.01;
    for _ in 0..400 {
        world.step(dt);
    }

    let lambda_left = world.unilateral_constraints[0].lambda;
    let lambda_right = world.unilateral_constraints[1].lambda;
    let lambda_main = world.unilateral_constraints[2].lambda;

    println!("Tension lambdas: left={}, right={}, main={}", lambda_left, lambda_right, lambda_main);

    // Verify left and right carry equal tension
    assert!((lambda_left - lambda_right).abs() < 1e-4);

    // Verify tension distribution ratio: T_leg / T_main = 1 / sqrt(2) ≈ 0.7071
    let ratio = lambda_left / lambda_main;
    let expected_ratio = 1.0 / sqrt_2;
    println!("Tension distribution ratio: actual={}, expected={}", ratio, expected_ratio);

    assert!(
        (ratio - expected_ratio).abs() < 0.01,
        "Tension distribution ratio {} did not match expected {}",
        ratio,
        expected_ratio
    );

    // Verify that the junction particle 2 is settled near (0, 0, 0)
    let pos_junction = world.particles.pos[2];
    assert!(pos_junction.length() < 0.02);

    // Check drag acceleration is active on load particle 3 (wind is along X, line is along Y)
    // Relative wind is along X, so drag force is along X, making particle 3 sway slightly in X
    let pos_load = world.particles.pos[3];
    assert!(pos_load.x > 0.0, "Line drag did not push load particle in wind direction: {:?}", pos_load);
}

#[test]
fn test_canopy_panel_aerodynamics() {
    use kite_core::CanopyPanel;

    // Test Case 1: Alpha = 0 (wind is along X, flat plate in XZ plane)
    {
        let mut world = World::new();
        world.cfg.substeps = 1;
        world.cfg.gravity = DVec3::ZERO;
        world.cfg.damping = 0.0;
        world.cfg.wind.v_ref = 10.0;
        world.cfg.wind.direction = DVec3::new(1.0, 0.0, 0.0);
        world.cfg.wind.shear_exponent = 0.0;

        world.add_particle(DVec3::new(0.0, 0.0, 0.0), 1.0);
        world.add_particle(DVec3::new(1.0, 0.0, 0.0), 1.0);
        world.add_particle(DVec3::new(0.0, 0.0, 1.0), 1.0);

        world.canopy_panels.push(CanopyPanel::new(0, 1, 2));

        let dt = 0.0001;
        world.step(dt);

        // Compute accumulated force from velocity change: F = m * v / dt
        let f_total = (world.particles.vel[0] + world.particles.vel[1] + world.particles.vel[2]) * 1.0 / dt;

        // Expected force: q * Area * CD0
        // q = 0.5 * 1.225 * 100 = 61.25 Pa
        // Area = 0.5 * 1.0 * 1.0 = 0.5 m^2
        // CD0 = 0.04
        // F_x = 61.25 * 0.5 * 0.04 = 1.225 N
        // F_y = 0.0
        println!("Aero force (alpha=0): {:?}", f_total);
        assert!((f_total.x - 1.225).abs() < 1e-4);
        assert!(f_total.y.abs() < 1e-4);
    }

    // Test Case 2: Alpha = 45 deg (alpha = pi/4)
    {
        let mut world = World::new();
        world.cfg.substeps = 1;
        world.cfg.gravity = DVec3::ZERO;
        world.cfg.damping = 0.0;
        
        let wind_speed = 10.0;
        let angle = std::f64::consts::FRAC_PI_4; // 45 deg
        world.cfg.wind.v_ref = wind_speed;
        world.cfg.wind.direction = DVec3::new(angle.cos(), angle.sin(), 0.0);
        world.cfg.wind.shear_exponent = 0.0;

        world.add_particle(DVec3::new(0.0, 0.0, 0.0), 1.0);
        world.add_particle(DVec3::new(1.0, 0.0, 0.0), 1.0);
        world.add_particle(DVec3::new(0.0, 0.0, 1.0), 1.0);

        world.canopy_panels.push(CanopyPanel::new(0, 1, 2));

        let dt = 0.0001;
        world.step(dt);

        let f_total = (world.particles.vel[0] + world.particles.vel[1] + world.particles.vel[2]) * 1.0 / dt;

        // Expected force:
        // F_normal = q * Area * CN * n = 61.25 * 0.5 * (1.1 * sin(90)) * (0, 1, 0) = (0.0, 33.6875, 0.0) N
        // F_drag0 = q * Area * CD0 * v_rel_unit = 61.25 * 0.5 * 0.04 * (cos(45), sin(45), 0.0) = (0.8662, 0.8662, 0.0) N
        // F_total = F_normal + F_drag0 = (0.8662, 34.5537, 0.0) N
        println!("Aero force (alpha=45): {:?}", f_total);
        assert!((f_total.x - 0.8662).abs() < 1e-3);
        assert!((f_total.y - 34.5537).abs() < 1e-3);
    }

    // Test Case 3: Alpha = 90 deg (alpha = pi/2, wind along Y)
    {
        let mut world = World::new();
        world.cfg.substeps = 1;
        world.cfg.gravity = DVec3::ZERO;
        world.cfg.damping = 0.0;
        world.cfg.wind.v_ref = 10.0;
        world.cfg.wind.direction = DVec3::new(0.0, 1.0, 0.0);
        world.cfg.wind.shear_exponent = 0.0;

        world.add_particle(DVec3::new(0.0, 0.0, 0.0), 1.0);
        world.add_particle(DVec3::new(1.0, 0.0, 0.0), 1.0);
        world.add_particle(DVec3::new(0.0, 0.0, 1.0), 1.0);

        world.canopy_panels.push(CanopyPanel::new(0, 1, 2));

        let dt = 0.0001;
        world.step(dt);

        let f_total = (world.particles.vel[0] + world.particles.vel[1] + world.particles.vel[2]) * 1.0 / dt;

        // Expected force:
        // alpha = 90 deg, sin(2alpha) = 0. C_N = 0.
        // F_total = F_drag0 = q * Area * CD0 * v_rel_unit = 61.25 * 0.5 * 0.04 * (0, 1, 0) = (0.0, 1.225, 0.0) N
        println!("Aero force (alpha=90): {:?}", f_total);
        assert!(f_total.x.abs() < 1e-4);
        assert!((f_total.y - 1.225).abs() < 1e-4);
    }
}

#[test]
fn test_spar_drag() {
    use kite_core::StretchShearConstraint;

    let mut world = World::new();
    world.cfg.substeps = 1;
    world.cfg.gravity = DVec3::ZERO;
    world.cfg.damping = 0.0;
    world.cfg.wind.v_ref = 10.0;
    world.cfg.wind.direction = DVec3::new(1.0, 0.0, 0.0);
    world.cfg.wind.shear_exponent = 0.0;

    world.add_particle(DVec3::new(0.0, 0.0, 0.0), 1.0);
    world.add_particle(DVec3::new(0.0, 0.0, 1.0), 1.0);

    // Add a segment orientation
    world.add_segment(glam::DQuat::IDENTITY, DVec3::ONE);

    // Spar segment: connects particle 0 and 1, orientation 0, rest_length 1.0, diameter 0.05m
    world.stretch_shear_constraints.push(StretchShearConstraint::new(
        0,
        1,
        0,
        1.0,
        DVec3::ZERO,
        0.05,
    ));

    let dt = 0.0001;
    world.step(dt);

    let f_total = (world.particles.vel[0] + world.particles.vel[1]) * 1.0 / dt;

    // Expected force:
    // F_cross = 0.5 * rho * CD_cyl * V_cross^2 * diameter * length
    // F_cross = 0.5 * 1.225 * 1.2 * 100 * 0.05 * 1.0 = 3.675 N along X
    println!("Spar drag force: {:?}", f_total);
    assert!((f_total.x - 3.675).abs() < 1e-4);
    assert!(f_total.y.abs() < 1e-4);
    assert!(f_total.z.abs() < 1e-4);
}

#[test]
fn test_simple_kite_v1_flight() {
    use std::fs::File;
    use std::io::Read;
    use serde::Deserialize;
    use kite_core::{KiteDefinition, build_kite_from_def};
    use kite_core::wind::WindConfig;

    #[derive(Debug, Deserialize)]
    struct Scenario {
        gravity: DVec3,
        duration: f64,
        wind: Option<WindConfig>,
        kite: Option<KiteDefinition>,
    }

    let mut file = File::open("../../scenarios/simple_kite_v1.toml")
        .or_else(|_| File::open("scenarios/simple_kite_v1.toml"))
        .expect("Failed to open scenario file");
    let mut toml_str = String::new();
    file.read_to_string(&mut toml_str).unwrap();

    let scenario: Scenario = toml::from_str(&toml_str).unwrap();

    let mut world = World::new();
    world.cfg.gravity = scenario.gravity;
    if let Some(wind_cfg) = scenario.wind {
        world.cfg.wind = wind_cfg;
    }
    let kite_def = scenario.kite.unwrap();
    build_kite_from_def(&mut world, &kite_def);

    // Step simulation
    let dt = 0.01;
    let steps = (scenario.duration / dt).round() as usize;
    for _ in 0..steps {
        world.step(dt);
    }

    let p_nose = 4;
    let p_tail = 0;
    let final_nose = world.particles.pos[p_nose];
    let final_tail = world.particles.pos[p_tail];
    let attitude_vector = (final_nose - final_tail).normalize();

    println!("TEST KITE final nose: {:?}", final_nose);
    println!("TEST KITE final tail: {:?}", final_tail);
    println!("TEST KITE final attitude: {:?}", attitude_vector);

    // Assert stable bounded positions (no NaNs or infinite drift)
    assert!(!final_nose.x.is_nan());
    assert!(final_nose.y > -2.0 && final_nose.y < -1.0);
    assert!(final_nose.z > -1.5 && final_nose.z < -0.5);

    // Assert attitude is aligned downwind (Z-direction)
    assert!((attitude_vector.z - (-1.0)).abs() < 0.05);

    // Assert bit-exact/near-exact regression check against golden snapshot
    // Final attitude from CLI: DVec3(8.9976e-7, 0.000996, -0.9999995)
    let expected_attitude = DVec3::new(0.00000089976, 0.00099609, -0.9999995);
    assert!((attitude_vector - expected_attitude).length() < 1e-4);
}

#[test]
fn test_ground_collision() {
    let mut world = World::new();
    world.cfg.ground_collision_enabled = true;
    world.cfg.gravity = DVec3::new(0.0, -9.81, 0.0);
    world.cfg.damping = 5.0; // high damping to settle

    // Add falling particle
    world.add_particle(DVec3::new(0.0, 5.0, 0.0), 1.0);

    let dt = 0.01;
    for _ in 0..300 {
        world.step(dt);
    }

    let pos = world.particles.pos[0];
    let vel = world.particles.vel[0];
    println!("Ground collision final pos: {:?}, vel: {:?}", pos, vel);

    // Should settle exactly at the ground plane thickness (0.005)
    assert!((pos.y - 0.005).abs() < 1e-4);
    assert!(pos.x.abs() < 1e-4);
    assert!(pos.z.abs() < 1e-4);
    assert!(vel.length() < 1e-2); // settled, no jitter
}

#[test]
fn test_self_collision_folding() {
    let mut world = World::new();
    world.cfg.self_collision_enabled = true;
    world.cfg.gravity = DVec3::new(0.0, -9.81, 0.0);
    world.cfg.damping = 5.0;

    // Create a pinned triangle at y = 0.05
    let p0 = world.add_particle(DVec3::new(0.0, 0.05, 0.0), 0.0); // pinned
    let p1 = world.add_particle(DVec3::new(1.0, 0.05, 0.0), 0.0); // pinned
    let p2 = world.add_particle(DVec3::new(0.5, 0.05, 0.5), 0.0); // pinned

    world.canopy_panels.push(kite_core::CanopyPanel::new(p0, p1, p2));

    // Add falling particle directly above the triangle's centroid
    let p3 = world.add_particle(DVec3::new(0.5, 1.0, 0.25), 1.0);

    let dt = 0.01;
    for _ in 0..300 {
        world.step(dt);
    }

    let pos_fall = world.particles.pos[p3];
    let vel_fall = world.particles.vel[p3];
    println!("Triangle collision final pos: {:?}, vel: {:?}", pos_fall, vel_fall);

    // Pinned triangle is at y = 0.05. Self-collision thickness is 0.01.
    // Falling particle should settle exactly at y = 0.05 + 0.01 = 0.06.
    assert!((pos_fall.y - 0.06).abs() < 1.0e-3);
    assert!(vel_fall.length() < 1.0e-2); // settled, no jitter
}

#[test]
fn test_pendulum_period() {
    let mut world = World::new();
    world.cfg.substeps = 50;
    world.cfg.iterations_per_substep = 4;
    world.cfg.gravity = DVec3::new(0.0, -9.81, 0.0);
    world.cfg.damping = 0.0; // no damping - period must stay honest, not decay

    let length = 1.0;
    let small_angle: f64 = 0.05; // rad (~2.9 deg); small-angle error ~0.06%, negligible vs 2% tolerance below

    world.add_particle(DVec3::ZERO, 0.0); // pinned anchor
    world.add_particle(
        DVec3::new(length * small_angle.sin(), -length * small_angle.cos(), 0.0),
        1.0,
    );
    world.distance_constraints.push(DistanceConstraint::new(0, 1, length, 1e-10)); // near-rigid

    // Track x(t) of the bob and find successive same-direction zero-crossings
    let dt = 0.001;
    let mut prev_x = world.particles.pos[1].x;
    let mut crossing_times = Vec::new();
    for _ in 0..20_000 {
        world.step(dt);
        let x = world.particles.pos[1].x;
        if prev_x > 0.0 && x <= 0.0 {
            crossing_times.push(world.time);
        }
        prev_x = x;
    }

    assert!(
        crossing_times.len() >= 2,
        "not enough oscillations captured: {}",
        crossing_times.len()
    );
    let measured_period = crossing_times[1] - crossing_times[0];
    let expected_period = 2.0 * std::f64::consts::PI * (length / 9.81).sqrt();
    let ratio = measured_period / expected_period;
    println!(
        "Pendulum period: actual={}, expected={}, ratio={}",
        measured_period, expected_period, ratio
    );

    assert!(
        (ratio - 1.0).abs() < 0.02,
        "Measured period {} not close to small-angle formula {} (ratio {})",
        measured_period,
        expected_period,
        ratio
    );
}


