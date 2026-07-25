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
use std::collections::HashMap;

fn default_subdivisions() -> usize { 1 }
fn default_areal_density() -> f64 { 0.05 }
fn default_num_segments() -> usize { 1 }
fn default_line_density() -> f64 { 970.0 } // Dyneema kg/m^3

/// Adds `extra_mass` (kg) to an existing particle's inverse mass, skipping pinned
/// particles (inv_mass == 0), which must stay pinned.
fn add_particle_mass(world: &mut World, idx: usize, extra_mass: f64) {
    if world.particles.inv_mass[idx] <= 0.0 {
        return;
    }
    let cur_mass = 1.0 / world.particles.inv_mass[idx];
    world.particles.inv_mass[idx] = 1.0 / (cur_mass + extra_mass);
}

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
    /// Number of congruent sub-triangles per edge (n); the panel is built as an n^2
    /// regular barycentric-grid mesh. 1 = untouched single triangle (legacy behavior).
    #[serde(default = "default_subdivisions")]
    pub subdivisions: usize,
    /// Fabric areal density in kg/m^2 (typical ripstop nylon ~0.05).
    #[serde(default = "default_areal_density")]
    pub areal_density: f64,
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
    /// Number of chained unilateral segments (1 = legacy single-segment line).
    #[serde(default = "default_num_segments")]
    pub num_segments: usize,
    /// Line material density in kg/m^3 (default ~970, Dyneema).
    #[serde(default = "default_line_density")]
    pub density: f64,
}

/// A stiff bend-twist junction locking two different spars' segments together at their
/// as-built relative angle (e.g. a T or cross intersection), rather than letting them
/// hinge freely at the shared weld particle.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StiffJunctionDef {
    pub point: DVec3,
    pub compliance: f64,
}

/// Ties two different spars together at a point (like a zip-tie/lashing): a stiff
/// distance constraint between the nearest node of each spar, holding their as-built
/// separation. Position-only (free pivot); use `stiff_junctions` on a welded node for
/// full bend-twist locking.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LashingDef {
    pub point: DVec3,
    pub compliance: f64,
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
    /// Extra points to pin (inv_mass = 0) after the kite is built. Welds to an existing
    /// node within 1mm, or creates a new isolated anchor particle (e.g. for tethers).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub pinned_points: Vec<DVec3>,
    /// Stiff bend-twist junctions locking together segments of different spars that meet
    /// at a welded particle (e.g. a T-joint cross-strut), holding their as-built angle.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub stiff_junctions: Vec<StiffJunctionDef>,
    /// Lashings tying two different spars together at a point with a stiff distance
    /// constraint (position-only, free pivot).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub lashings: Vec<LashingDef>,
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
    // Tracks, per particle, which (spar_id, orientation q_index) pairs touch it. Used by
    // stiff junctions (step 1.5) to find segments from different spars sharing a weld point.
    let mut particle_segments: HashMap<usize, Vec<(usize, usize)>> = HashMap::new();

    for (spar_id, spar) in def.spars.iter().enumerate() {
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

        // Create particles along the spar.
        //
        // Lumped-mass discretization: a segment's mass is split half to each of
        // its two end nodes, so interior nodes carry a full segment mass and the
        // two tips carry half. The nodal masses then sum to exactly the spar's
        // mass (density * area * length) and the mass distribution is symmetric
        // about the spar's midpoint. Giving every one of the n+1 nodes a full
        // segment mass instead overstates the spar by (n+1)/n (25% at n = 4,
        // 50% at n = 2) and doubles the weighting of the tips, which inflates
        // the kite's moment of inertia about its own centre of mass and so
        // wrongly slows every rotational response.
        //
        // Welded intersections (e.g. a cross-strut crossing the spine) accumulate
        // a share from each spar that meets there rather than keeping only the
        // first spar's, which is what `find_or_add_particle` would do.
        let mut spar_particles = Vec::new();
        for i in 0..=spar.num_segments {
            let p_pos = spar.start + delta * (i as f64);
            let share = if i == 0 || i == spar.num_segments {
                0.5 * segment_mass
            } else {
                segment_mass
            };
            let existing =
                (0..world.particles.len()).find(|&j| (world.particles.pos[j] - p_pos).length() < weld_tol);
            let idx = match existing {
                Some(j) => {
                    add_particle_mass(world, j, share);
                    j
                }
                None => world.add_particle(p_pos, share),
            };
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
            // `add_segment` takes the *inverse* inertia (generalized rotational inverse
            // mass); passing the inertia itself makes every rod frame ~I^-2 times too
            // heavy to rotate, which welds each spar segment's director frame to its
            // as-built world orientation and freezes the whole kite's attitude.
            let q_idx =
                world.add_segment(q_rot, geom.compute_inv_inertia(segment_len, segment_mass));
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

            // Record that this segment's orientation touches both of its endpoint particles.
            particle_segments.entry(spar_particles[i]).or_default().push((spar_id, q_idx));
            particle_segments.entry(spar_particles[i + 1]).or_default().push((spar_id, q_idx));
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

    // 1.5 Build stiff bend-twist junctions between segments of DIFFERENT spars that share
    // a welded particle, locking them at their as-built (rest) relative orientation.
    for junction in &def.stiff_junctions {
        let found = (0..world.particles.len())
            .find(|&i| (world.particles.pos[i] - junction.point).length() < weld_tol);
        let Some(p_idx) = found else {
            // No welded particle at this point, nothing to stiffen. Not a hard error since
            // junctions are declarative and may reference a not-yet-built point; this should
            // not happen in practice for spar intersections, hence the debug_assert.
            debug_assert!(false, "stiff_junction point {:?} did not weld to any particle", junction.point);
            continue;
        };
        let Some(touching) = particle_segments.get(&p_idx) else {
            continue;
        };

        // Rule 3 (AGENTS.md): compliance is never literally 0.0 in a division.
        let comp = junction.compliance.max(1e-12);

        for a in 0..touching.len() {
            for b in (a + 1)..touching.len() {
                let (spar_a, qa) = touching[a];
                let (spar_b, qb) = touching[b];
                if spar_a == spar_b {
                    continue; // only cross-spar pairs get a stiff junction
                }

                // UNVERIFIED: the Darboux vector here is read directly as solver.rs's
                // bend-twist solve does: rest_value = (q1.conjugate() * q2).xyz(). This raw
                // quaternion-vector-part convention is only an exact small-angle Darboux
                // vector; at a 90-degree as-built cross it is a large-angle value. Because we
                // read it directly from the as-built quaternions (same formula, same instant),
                // the constraint still reads C = 0 exactly at the build pose and stiffly
                // resists deviation, but the *shape* of the restoring "stiffness" away from
                // that pose is not verified to match true rotational elasticity at large
                // deflection. Flagging per AGENTS.md rule 7.
                let q_a = world.orientations.quat[qa];
                let q_b = world.orientations.quat[qb];
                let rest_value = (q_a.conjugate() * q_b).xyz();

                world.bend_twist_constraints.push(BendTwistConstraint::new(
                    qa,
                    qb,
                    rest_value,
                    DVec3::splat(comp),
                    f64::MAX,
                    false,
                ));
            }
        }
    }

    // 1.6 Build spar-to-spar lashings: a stiff distance constraint between the nearest
    // node of each of two different spars, holding their as-built separation
    // (position-only tie/free pivot, unlike stiff_junctions, which also locks
    // bend/twist and requires the two spars to already share a welded particle).
    for lashing in &def.lashings {
        // For each spar with a node within 0.25m of `point`, keep only its closest node.
        let mut best_per_spar: HashMap<usize, (usize, f64)> = HashMap::new();
        for (&p_idx, touching) in &particle_segments {
            let dist = (world.particles.pos[p_idx] - lashing.point).length();
            if dist > 0.25 {
                continue;
            }
            for &(spar_id, _) in touching {
                let entry = best_per_spar.entry(spar_id).or_insert((p_idx, dist));
                if dist < entry.1 || (dist == entry.1 && p_idx < entry.0) {
                    *entry = (p_idx, dist);
                }
            }
        }

        // Pick the two closest-to-point spars (each already reduced to its closest node);
        // sort with explicit tie-breaks for determinism (AGENTS.md rule 6), since
        // particle_segments iteration order is a HashMap and not stable.
        let mut candidates: Vec<(usize, usize, f64)> = best_per_spar
            .iter()
            .map(|(&spar_id, &(p_idx, dist))| (spar_id, p_idx, dist))
            .collect();
        candidates.sort_unstable_by(|a, b| {
            a.2.total_cmp(&b.2).then_with(|| a.0.cmp(&b.0)).then_with(|| a.1.cmp(&b.1))
        });

        if candidates.len() < 2 {
            // Declarative like stiff_junctions: not a hard error, but shouldn't happen for
            // a well-formed scenario.
            debug_assert!(
                false,
                "lashing point {:?} found nodes from fewer than two spars within 0.25m",
                lashing.point
            );
            continue;
        }

        let p_a = candidates[0].1;
        let p_b = candidates[1].1;
        if p_a == p_b {
            continue; // already welded to the same particle, nothing to lash
        }

        // Rule 3 (AGENTS.md): compliance is never literally 0.0 in a division.
        let comp = lashing.compliance.max(1e-12);
        let as_built_dist = (world.particles.pos[p_a] - world.particles.pos[p_b]).length();
        world
            .distance_constraints
            .push(DistanceConstraint::new(p_a, p_b, as_built_dist, comp));
    }

    // 2. Build Canopy Panels and Cloth Constraints
    // Each PanelDef triangle is subdivided into subdivisions^2 congruent sub-triangles on a
    // regular barycentric grid; fabric mass (area * areal_density) is distributed to the
    // corner particles so total canopy mass is mesh-resolution-independent.
    let mut panel_particle_indices = Vec::new();
    let bridle_anchor_mass = 0.01; // small default mass for newly-created bridle endpoints

    for panel in &def.panels {
        let n = panel.subdivisions.max(1);
        let n_f = n as f64;

        let grid_pos = |i: usize, j: usize| -> DVec3 {
            panel.p1
                + (panel.p2 - panel.p1) * (i as f64 / n_f)
                + (panel.p3 - panel.p1) * (j as f64 / n_f)
        };

        // Enumerate the n^2 sub-triangles (upward + downward) on the (i,j) grid, i+j <= n.
        let mut tris: Vec<[(usize, usize); 3]> = Vec::new();
        for j in 0..n {
            for i in 0..(n - j) {
                tris.push([(i, j), (i + 1, j), (i, j + 1)]);
            }
        }
        for j in 0..n.saturating_sub(1) {
            for i in 0..(n - 1 - j) {
                tris.push([(i + 1, j), (i + 1, j + 1), (i, j + 1)]);
            }
        }

        // Accumulate each grid vertex's total fabric mass share (area * areal_density / 3
        // per incident sub-triangle) before touching the world, so each vertex is
        // created/welded exactly once with its full share.
        let mut vertex_mass: HashMap<(usize, usize), f64> = HashMap::new();
        for tri in &tris {
            let p0 = grid_pos(tri[0].0, tri[0].1);
            let p1 = grid_pos(tri[1].0, tri[1].1);
            let p2 = grid_pos(tri[2].0, tri[2].1);
            let area = 0.5 * (p1 - p0).cross(p2 - p0).length();
            let share = area * panel.areal_density / 3.0;
            for &v in tri {
                *vertex_mass.entry(v).or_insert(0.0) += share;
            }
        }

        let mut grid_idx: HashMap<(usize, usize), usize> = HashMap::new();
        let mut sorted_verts: Vec<_> = vertex_mass.keys().copied().collect();
        sorted_verts.sort_unstable();
        for v in sorted_verts {
            let mass = vertex_mass[&v];
            let pos = grid_pos(v.0, v.1);
            // Weld to an existing particle (accumulating mass) or create a new one.
            let idx = {
                let mut found = None;
                for i in 0..world.particles.len() {
                    if (world.particles.pos[i] - pos).length() < weld_tol {
                        found = Some(i);
                        break;
                    }
                }
                match found {
                    Some(i) => {
                        add_particle_mass(world, i, mass);
                        i
                    }
                    None => world.add_particle(pos, mass),
                }
            };
            grid_idx.insert(v, idx);
        }

        // Helper to add unique distance constraints for sub-triangle edges (reuses the
        // existing warp/weft/shear classification).
        let add_cloth_edge = |world: &mut World, p_a: usize, p_b: usize, panel_def: &PanelDef| {
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

        for tri in &tris {
            let i1 = grid_idx[&tri[0]];
            let i2 = grid_idx[&tri[1]];
            let i3 = grid_idx[&tri[2]];

            panel_particle_indices.push((i1, i2, i3, panel.clone()));
            world.canopy_panels.push(CanopyPanel::new(i1, i2, i3));

            add_cloth_edge(world, i1, i2, panel);
            add_cloth_edge(world, i2, i3, panel);
            add_cloth_edge(world, i3, i1, panel);
        }
    }

    // 3. Build Dihedral Bending Constraints (detect shared edges via a shared-edge map,
    // O(n) instead of the old O(n^2) all-pairs scan, needed since subdivision multiplies
    // panel count).
    let mut edge_map: HashMap<(usize, usize), Vec<(usize, usize)>> = HashMap::new();
    for (panel_idx, &(pi1, pi2, pi3, _)) in panel_particle_indices.iter().enumerate() {
        for &(a, b, opp) in &[(pi1, pi2, pi3), (pi2, pi3, pi1), (pi3, pi1, pi2)] {
            let key = (a.min(b), a.max(b));
            edge_map.entry(key).or_default().push((panel_idx, opp));
        }
    }
    // Sort for determinism (AGENTS.md rule 6): HashMap iteration order is not stable.
    let mut sorted_edges: Vec<_> = edge_map.into_iter().collect();
    sorted_edges.sort_unstable_by_key(|(k, _)| *k);

    for ((p_shared1, p_shared2), entries) in sorted_edges {
        if entries.len() != 2 {
            continue;
        }
        let (panel_i_idx, p_opp_i) = entries[0];
        let (_panel_j_idx, p_opp_j) = entries[1];
        let (_, _, _, ref p_def) = panel_particle_indices[panel_i_idx];

        let pos1 = world.particles.pos[p_shared1];
        let pos2 = world.particles.pos[p_shared2];
        let pos3 = world.particles.pos[p_opp_i];
        let pos4 = world.particles.pos[p_opp_j];

        let rest_angle = crate::world::compute_dihedral_angle(pos1, pos2, pos3, pos4);

        world.dihedral_bending_constraints.push(DihedralBendingConstraint::new(
            p_shared1,
            p_shared2,
            p_opp_i,
            p_opp_j,
            rest_angle,
            p_def.bending_compliance,
        ));
    }

    // 4. Build Bridle Junction and Lines
    let j_mass = if def.bridle_junction_pinned {
        0.0
    } else {
        0.05
    };
    let j_idx = find_or_add_particle(world, def.bridle_junction, j_mass);

    // Bridle-to-bridle knots: a bridle's `from`/`to` may coincide with an interior node
    // of ANOTHER bridle line rather than a spar/panel/junction point. Interior nodes only
    // exist once that other bridle has actually been built (its chain particles created),
    // so build order matters, unlike every other weld in this format, which is
    // order-independent because both sides compute the same coordinate. Resolve a build
    // order here so this works regardless of `def.bridles` listing order.
    //
    // Precompute each bridle's prospective interior-node positions purely from the def
    // (straight-line k/n interpolation, identical math to the chain-building loop below).
    let n_bridles = def.bridles.len();
    let bridle_interior_positions: Vec<Vec<DVec3>> = def
        .bridles
        .iter()
        .map(|line| {
            let n_seg = line.num_segments.max(1);
            (1..n_seg)
                .map(|k| {
                    let t = k as f64 / n_seg as f64;
                    line.from + (line.to - line.from) * t
                })
                .collect()
        })
        .collect();

    // Bridle i depends on bridle j (i != j) if i's `from` or `to` lands on one of j's
    // interior nodes, i.e. i must be built after j.
    let mut deps: Vec<Vec<usize>> = vec![Vec::new(); n_bridles];
    for (i, dep_i) in deps.iter_mut().enumerate() {
        let bridle_i = &def.bridles[i];
        for (j, interior_j) in bridle_interior_positions.iter().enumerate() {
            if i == j {
                continue;
            }
            let attaches = interior_j.iter().any(|&node| {
                (bridle_i.from - node).length() < weld_tol
                    || (bridle_i.to - node).length() < weld_tol
            });
            if attaches {
                dep_i.push(j);
            }
        }
    }

    // Deferred-rounds scheduling: repeatedly build any bridle whose dependencies are
    // already built, until a pass makes no progress. A knot cycle (degenerate, but must
    // not hang or panic) leaves some bridles unbuilt; append those in def order to break
    // the cycle deterministically.
    let mut built = vec![false; n_bridles];
    let mut build_order: Vec<usize> = Vec::with_capacity(n_bridles);
    loop {
        let mut progressed = false;
        for i in 0..n_bridles {
            if !built[i] && deps[i].iter().all(|&d| built[d]) {
                built[i] = true;
                build_order.push(i);
                progressed = true;
            }
        }
        if !progressed {
            break;
        }
    }
    for (i, &b) in built.iter().enumerate() {
        if !b {
            build_order.push(i);
        }
    }

    for &bridle_idx in &build_order {
        let line = &def.bridles[bridle_idx];
        // Find closest particles to "from" and "to"
        let from_idx = find_or_add_particle(world, line.from, bridle_anchor_mass);
        let to_idx = if (line.to - def.bridle_junction).length() < weld_tol {
            j_idx
        } else {
            find_or_add_particle(world, line.to, bridle_anchor_mass)
        };

        let n_seg = line.num_segments.max(1);
        let seg_rest_len = line.rest_length / n_seg as f64;

        // Total line mass, distributed evenly over the interior (newly-created) particles
        // only. Endpoints keep their existing (welded kite/junction) mass.
        let line_cross_area = std::f64::consts::PI * (line.diameter * 0.5).powi(2);
        let total_line_mass = line.density * line_cross_area * line.rest_length;
        let interior_count = n_seg.saturating_sub(1);
        let interior_mass = if interior_count > 0 {
            total_line_mass / interior_count as f64
        } else {
            0.0
        };

        let mut chain = Vec::with_capacity(n_seg + 1);
        chain.push(from_idx);
        for k in 1..n_seg {
            let t = k as f64 / n_seg as f64;
            let pos = line.from + (line.to - line.from) * t;
            let idx = world.add_particle(pos, interior_mass);
            chain.push(idx);
        }
        chain.push(to_idx);

        for k in 0..n_seg {
            world.unilateral_constraints.push(UnilateralDistanceConstraint::new(
                chain[k],
                chain[k + 1],
                seg_rest_len,
                line.compliance,
                line.diameter,
            ));
        }
    }

    // 5. Pin extra points (weld to existing node within tolerance, or create a new anchor).
    for &pt in &def.pinned_points {
        let idx = find_or_add_particle(world, pt, 0.0);
        world.particles.inv_mass[idx] = 0.0;
        world.particles.vel[idx] = DVec3::ZERO;
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
