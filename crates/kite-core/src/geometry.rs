use crate::aero::CanopyPanel;
use crate::constraints::{
    BendTwistConstraint, DihedralBendingConstraint, DistanceConstraint, StretchShearConstraint,
    UnilateralDistanceConstraint,
};
use crate::materials::{
    bend_twist_compliance, stretch_shear_compliance, Material, SectionGeometry,
};
use crate::world::World;
use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};

/// Definition of a spar (rod) in the kite.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SparDef {
    pub name: String,
    pub start: DVec3,
    pub end: DVec3,
    pub num_segments: usize,
    pub radius: f64,
    pub youngs_modulus: f64,
    pub shear_modulus: f64,
    pub density: f64,
}

/// Definition of a canopy panel (triangle) in the kite.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PanelDef {
    pub name: String,
    pub p1: DVec3,
    pub p2: DVec3,
    pub p3: DVec3,
    pub warp_compliance: f64,
    pub weft_compliance: f64,
    pub shear_compliance: f64,
    pub bending_compliance: f64,
}

/// Definition of a bridle line in the kite.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BridleLineDef {
    pub name: String,
    pub from: DVec3,
    pub to: DVec3,
    pub rest_length: f64,
    pub compliance: f64,
    pub diameter: f64,
}

/// A complete parametric/structural definition of a kite.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct KiteDefinition {
    pub name: String,
    pub spars: Vec<SparDef>,
    pub panels: Vec<PanelDef>,
    pub bridles: Vec<BridleLineDef>,
    pub bridle_junction: DVec3,
    pub bridle_junction_pinned: bool,
}

/// Geometry importer pipeline: builds all particles and constraints in the World
/// from a parsed KiteDefinition, welding adjacent nodes within a tolerance.
pub fn build_kite_from_def(world: &mut World, def: &KiteDefinition) {
    let weld_tol = 1.0e-3; // 1mm weld tolerance

    // Helper to find or add a particle in the world at a given position.
    // If a particle already exists within weld_tol, returns its index.
    let find_or_add_particle = |world: &mut World, pos: DVec3, mass: f64| -> usize {
        for i in 0..world.particles.len() {
            if (world.particles.pos[i] - pos).length() < weld_tol {
                // If the new request has a non-pinned mass and the existing particle is pinned (mass=0),
                // we keep it pinned or update. But usually anchors are pinned, and we want to keep them pinned.
                return i;
            }
        }
        world.add_particle(pos, mass)
    };

    // 1. Build Spars
    for spar in &def.spars {
        let delta = (spar.end - spar.start) / (spar.num_segments as f64);
        let segment_len = delta.length();

        // Material properties
        let mat = Material {
            youngs_modulus: spar.youngs_modulus,
            shear_modulus: spar.shear_modulus,
            density: spar.density,
            tensile_strength: 1.0e9, // large default
        };
        let geom = SectionGeometry::SolidRound {
            radius: spar.radius,
        };
        let cross_area = geom.area();
        let segment_mass = spar.density * cross_area * segment_len;

        // Create particles along the spar
        let mut spar_particles = Vec::new();
        for i in 0..=spar.num_segments {
            let p_pos = spar.start + delta * (i as f64);
            // Welds intersections automatically (e.g. cross-strut intersection)
            let idx = find_or_add_particle(world, p_pos, segment_mass);
            spar_particles.push(idx);
        }

        // Add orientation segments and stretch-shear constraints
        let mut spar_orientations = Vec::new();
        let u_tangent = delta.normalize();

        // Find rest orientation: rotates world Z (local tangent) to spar tangent direction
        let q_rot = if u_tangent.dot(DVec3::NEG_Z).abs() > 0.999 {
            DQuat::from_rotation_y(std::f64::consts::PI)
        } else {
            DQuat::from_rotation_arc(DVec3::Z, u_tangent)
        };

        let comp_ss = stretch_shear_compliance(&mat, &geom, segment_len);

        for i in 0..spar.num_segments {
            let q_idx = world.add_segment(q_rot, geom.compute_inertia(segment_len, segment_mass));
            spar_orientations.push(q_idx);

            world
                .stretch_shear_constraints
                .push(StretchShearConstraint::new(
                    spar_particles[i],
                    spar_particles[i + 1],
                    q_idx,
                    segment_len,
                    comp_ss,
                    2.0 * spar.radius,
                ));
        }

        // Add bend-twist constraints between adjacent segments in this spar
        let comp_bt = bend_twist_compliance(&mat, &geom, segment_len);
        for i in 0..spar.num_segments - 1 {
            world.bend_twist_constraints.push(BendTwistConstraint::new(
                spar_orientations[i],
                spar_orientations[i + 1],
                DVec3::ZERO, // zero rest curvature/torsion (straight spar)
                comp_bt,
                f64::MAX, // no yield limit for this simple setup
                false,
            ));
        }
    }

    // 2. Build Canopy Panels and Cloth Constraints
    let mut panel_particle_indices = Vec::new();
    let fabric_mass = 0.02; // 20g default per vertex

    for panel in &def.panels {
        let i1 = find_or_add_particle(world, panel.p1, fabric_mass);
        let i2 = find_or_add_particle(world, panel.p2, fabric_mass);
        let i3 = find_or_add_particle(world, panel.p3, fabric_mass);

        panel_particle_indices.push((i1, i2, i3, panel.clone()));

        world.canopy_panels.push(CanopyPanel::new(i1, i2, i3));

        // Helper to add unique unique distance constraints for triangle edges
        let mut add_cloth_edge = |p_a: usize, p_b: usize, panel_def: &PanelDef| {
            // Check if constraint already exists
            for c in &world.distance_constraints {
                if (c.p1 == p_a && c.p2 == p_b) || (c.p1 == p_b && c.p2 == p_a) {
                    return;
                }
            }

            let pos_a = world.particles.pos[p_a];
            let pos_b = world.particles.pos[p_b];
            let edge_vec = pos_b - pos_a;
            let rest_len = edge_vec.length();
            if rest_len < weld_tol {
                return;
            }

            // Determine if edge is warp, weft, or shear based on grain orientation
            // Here, we check the alignment with X (warp) and Z (weft)
            let u_edge = edge_vec.normalize();
            let cos_warp = u_edge.x.abs();
            let cos_weft = u_edge.z.abs();

            let comp = if cos_warp > 0.866 {
                panel_def.warp_compliance // close to X-axis
            } else if cos_weft > 0.866 {
                panel_def.weft_compliance // close to Z-axis
            } else {
                panel_def.shear_compliance // diagonal
            };

            world
                .distance_constraints
                .push(DistanceConstraint::new(p_a, p_b, rest_len, comp));
        };

        add_cloth_edge(i1, i2, panel);
        add_cloth_edge(i2, i3, panel);
        add_cloth_edge(i3, i1, panel);
    }

    // 3. Build Dihedral Bending Constraints (detect shared edges)
    let n_panels = panel_particle_indices.len();
    for i in 0..n_panels {
        for j in i + 1..n_panels {
            let (pi1, pi2, pi3, ref p_def) = panel_particle_indices[i];
            let (pj1, pj2, pj3, _) = panel_particle_indices[j];

            let set_i = [pi1, pi2, pi3];
            let set_j = [pj1, pj2, pj3];

            // Find shared particles
            let mut shared = Vec::new();
            for &idx in &set_i {
                if set_j.contains(&idx) {
                    shared.push(idx);
                }
            }

            if shared.len() == 2 {
                // Shared edge between shared[0] and shared[1]
                let p_shared1 = shared[0];
                let p_shared2 = shared[1];

                // Find unshared opposite vertices
                let p_opp_i = *set_i
                    .iter()
                    .find(|&&x| x != p_shared1 && x != p_shared2)
                    .unwrap();
                let p_opp_j = *set_j
                    .iter()
                    .find(|&&x| x != p_shared1 && x != p_shared2)
                    .unwrap();

                // Compute rest dihedral angle
                let pos1 = world.particles.pos[p_shared1];
                let pos2 = world.particles.pos[p_shared2];
                let pos3 = world.particles.pos[p_opp_i];
                let pos4 = world.particles.pos[p_opp_j];

                let rest_angle = crate::world::compute_dihedral_angle(pos1, pos2, pos3, pos4);

                world
                    .dihedral_bending_constraints
                    .push(DihedralBendingConstraint::new(
                        p_shared1,
                        p_shared2,
                        p_opp_i,
                        p_opp_j,
                        rest_angle,
                        p_def.bending_compliance,
                    ));
            }
        }
    }

    // 4. Build Bridle Junction and Lines
    let j_mass = if def.bridle_junction_pinned {
        0.0
    } else {
        0.05
    };
    let j_idx = find_or_add_particle(world, def.bridle_junction, j_mass);

    for line in &def.bridles {
        // Find closest particles to "from" and "to"
        let from_idx = find_or_add_particle(world, line.from, fabric_mass);
        let to_idx = if (line.to - def.bridle_junction).length() < weld_tol {
            j_idx
        } else {
            find_or_add_particle(world, line.to, fabric_mass)
        };

        world
            .unilateral_constraints
            .push(UnilateralDistanceConstraint::new(
                from_idx,
                to_idx,
                line.rest_length,
                line.compliance,
                line.diameter,
            ));
    }
}

/// Builds a pinned rectangular cloth grid of `n × n` particles: warp/weft/shear
/// distance constraints plus dihedral bending across interior weft edges.
/// Used by benchmarks and the parallel-solver equivalence tests; returns the
/// index of the first particle added.
pub fn build_cloth_grid(world: &mut World, n: usize, spacing: f64, mass: f64) -> usize {
    let first = world.particles.len();
    let idx = |c: usize, r: usize| first + c + r * n;

    for r in 0..n {
        for c in 0..n {
            let pos = DVec3::new(c as f64 * spacing, 0.0, r as f64 * spacing);
            let m = if r == 0 { 0.0 } else { mass }; // pin the top row
            world.add_particle(pos, m);
        }
    }

    let stretch_compliance = 1e-6;
    let shear_compliance = 1e-4;
    let bend_compliance = 1e-2;
    let diag = spacing * std::f64::consts::SQRT_2;

    for r in 0..n {
        for c in 0..n {
            if c + 1 < n {
                world.distance_constraints.push(DistanceConstraint::new(
                    idx(c, r),
                    idx(c + 1, r),
                    spacing,
                    stretch_compliance,
                ));
            }
            if r + 1 < n {
                world.distance_constraints.push(DistanceConstraint::new(
                    idx(c, r),
                    idx(c, r + 1),
                    spacing,
                    stretch_compliance,
                ));
            }
            if c + 1 < n && r + 1 < n {
                world.distance_constraints.push(DistanceConstraint::new(
                    idx(c, r),
                    idx(c + 1, r + 1),
                    diag,
                    shear_compliance,
                ));
                world.distance_constraints.push(DistanceConstraint::new(
                    idx(c + 1, r),
                    idx(c, r + 1),
                    diag,
                    shear_compliance,
                ));
            }
        }
    }

    // Dihedral bending across each interior weft edge (flat rest angle):
    // edge (c,r)-(c+1,r) shared by wing particles (c or c+1, r-1) and (c, r+1).
    for r in 1..n - 1 {
        for c in 0..n - 1 {
            world
                .dihedral_bending_constraints
                .push(DihedralBendingConstraint::new(
                    idx(c, r),
                    idx(c + 1, r),
                    idx(c, r - 1),
                    idx(c, r + 1),
                    0.0,
                    bend_compliance,
                ));
        }
    }

    first
}
