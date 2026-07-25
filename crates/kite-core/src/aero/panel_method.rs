use crate::aero::coefficients::flat_plate_coefficients;
use crate::world::World;
use glam::DVec3;

/// A triangular canopy panel defined by 3 particle indices.
#[derive(Clone, Copy, Debug)]
pub struct CanopyPanel {
    /// Index of the first particle.
    pub p1: usize,
    /// Index of the second particle.
    pub p2: usize,
    /// Index of the third particle.
    pub p3: usize,
    /// Mean vertex velocity after the previous substep's added-mass
    /// correction (`None` until first sampled). See `aero::unsteady`.
    pub(crate) prev_mean_vel: Option<DVec3>,
}

impl CanopyPanel {
    /// Creates a new CanopyPanel.
    pub fn new(p1: usize, p2: usize, p3: usize) -> Self {
        Self {
            p1,
            p2,
            p3,
            prev_mean_vel: None,
        }
    }
}

/// Per-panel aerodynamic readback (lift, drag force and angle of attack) recorded during
/// `apply_canopy_aerodynamics` for debug visualization. Not used by the solver itself.
#[derive(Clone, Copy, Debug, Default)]
pub struct PanelAero {
    /// Lift force applied to this panel (world frame, Newtons).
    pub lift: DVec3,
    /// Drag force applied to this panel (world frame, Newtons).
    pub drag: DVec3,
    /// Angle of attack (radians).
    pub alpha: f64,
}

/// Centroid, area and unit normal of a panel from its current vertex positions.
/// `None` if the triangle is degenerate (zero area).
fn panel_geom(world: &World, panel: &CanopyPanel) -> Option<(DVec3, f64, DVec3)> {
    let p1 = world.particles.pos[panel.p1];
    let p2 = world.particles.pos[panel.p2];
    let p3 = world.particles.pos[panel.p3];
    let cross = (p2 - p1).cross(p3 - p1);
    let double_area = cross.length();
    if double_area < 1e-12 {
        return None;
    }
    Some((
        (p1 + p2 + p3) / 3.0,
        0.5 * double_area,
        cross / double_area,
    ))
}

/// Chordwise load-distribution shape, normalized so $\int_0^1 w\,d\xi = 1$:
///
/// $$w(\xi) = (p+1)(1-\xi)^p$$
///
/// Its centroid is $\int_0^1 \xi w \, d\xi = 1/(p+2)$, so requesting a
/// center-of-pressure at chord fraction $\bar{x}$ means $p = 1/\bar{x} - 2$.
/// $\bar{x} = 1/4$ gives $p = 2$ (the thin-airfoil flat-plate quarter-chord
/// result); $\bar{x} = 1/2$ gives $p = 0$, i.e. uniform pressure. The family is
/// non-negative and bounded on $[0, 1]$ for $p \ge 0$, and monotonically
/// decreasing from leading to trailing edge as attached-flow plate loading is.
fn chordwise_weight(xi: f64, p: f64) -> f64 {
    (p + 1.0) * (1.0 - xi.clamp(0.0, 1.0)).powf(p)
}

/// Computes panel-method aerodynamic forces for all canopy panels (triangles)
/// and applies them as velocity updates to their vertices.
///
/// The pitching moment of a sail comes from the *chordwise* distribution of its
/// pressure load, so that is what is modeled: each panel's quasi-steady
/// flat-plate force is scaled by the sail-level chordwise loading law
/// `chordwise_weight` evaluated at that panel's own station along the sail
/// chord, then applied at the panel's own centroid.
///
/// Two earlier formulations of this are both wrong, in ways worth recording:
///
/// * Splitting each panel's force equally over its 3 vertices applies the
///   resultant at the panel's area centroid — a uniform-pressure assumption.
///   For a *flat* panel that is mathematically guaranteed to produce zero
///   pitching moment about that panel, so a single-panel sail can never trim.
/// * Offsetting each panel's application point to a center of pressure derived
///   from that panel's *own* chordwise extent is not mesh-convergent: the offset
///   scales with the sub-triangle, and the up/down-pointing triangles of a
///   subdivided mesh have opposite centroid-to-CoP offsets which cancel
///   incoherently. Measured on a diamond kite at 25 deg, the whole-sail pitching
///   moment ran +0.41, -0.59, -1.36, -1.83, -2.35, -2.64 N.m for 4..256 panels —
///   it changes sign under refinement and never settles.
///
/// Scaling per-panel loads by a distribution referenced to the *sail* chord is
/// mesh-convergent by construction: it is a Riemann sum of a bounded chordwise
/// load density, and the discrete renormalization below keeps the total force
/// equal to the (already convergent) quasi-steady panel-method resultant.
pub fn apply_canopy_aerodynamics(world: &mut World, h: f64) {
    let rho = 1.225; // standard air density kg/m^3

    world.aero_readback.clear();
    world
        .aero_readback
        .resize(world.canopy_panels.len(), PanelAero::default());

    // --- Pass 1: per-panel quasi-steady force, plus the sail-level flow reference.
    let mut area_sum = 0.0;
    let mut area_normal = DVec3::ZERO;
    let mut area_v_rel = DVec3::ZERO;
    let mut area_abs_alpha = 0.0;

    for (panel_idx, panel) in world.canopy_panels.iter().enumerate() {
        let Some((centroid, area, normal)) = panel_geom(world, panel) else {
            continue;
        };

        let v_triangle = (world.particles.vel[panel.p1]
            + world.particles.vel[panel.p2]
            + world.particles.vel[panel.p3])
            / 3.0;
        let wind_vel = crate::wind::wind_at(centroid, world.time, &world.cfg.wind);
        let v_rel = wind_vel - v_triangle;

        // Panel area and orientation define the sail reference frame whether or
        // not this panel happens to be carrying load this substep.
        area_sum += area;
        area_normal += area * normal;
        area_v_rel += area * v_rel;

        let v_rel_mag = v_rel.length();
        if v_rel_mag < 1e-6 {
            continue;
        }
        let v_rel_unit = v_rel / v_rel_mag;

        // Angle of attack alpha: sin(alpha) = v_rel_unit . normal
        let sin_alpha = v_rel_unit.dot(normal).clamp(-1.0, 1.0);
        let alpha = sin_alpha.asin();
        area_abs_alpha += area * alpha.abs();

        // Dynamic pressure q = 0.5 * rho * |v_rel|^2
        let q = 0.5 * rho * v_rel_mag * v_rel_mag;
        let (_cn, cl, cd) = flat_plate_coefficients(alpha);

        // Drag is parallel to the relative wind; lift is perpendicular to it, in
        // the plane spanned by the normal and the relative wind.
        let f_drag = q * area * cd * v_rel_unit;
        let f_lift = if alpha.cos().abs() > 1e-6 {
            let lift_dir = (normal - sin_alpha * v_rel_unit).normalize();
            q * area * cl * lift_dir
        } else {
            DVec3::ZERO
        };

        world.aero_readback[panel_idx] = PanelAero {
            lift: f_lift,
            drag: f_drag,
            alpha,
        };
    }

    if area_sum <= 0.0 {
        return;
    }

    // --- Sail chordwise reference direction: the mean relative wind projected
    // into the mean sail plane, pointing leading edge -> trailing edge.
    let n_bar = area_normal.normalize_or_zero();
    let v_bar = area_v_rel / area_sum;
    let chord_dir = (v_bar - v_bar.dot(n_bar) * n_bar).normalize_or_zero();

    // Center-of-pressure chord fraction, and the loading exponent realizing it.
    // UNVERIFIED: flat-plate CoP travel — quarter-chord at small |alpha| (exact
    // thin-airfoil-theory result) blending to mid-chord (uniform pressure) at
    // 90 deg. The 90-deg endpoint is exact for a bluff normal plate; the cosine
    // blend between the two is a standard flight-sim interpolation, not taken
    // from a cited kite-aerodynamics source.
    let alpha_bar = area_abs_alpha / area_sum;
    let x_bar = (0.25 + 0.25 * (1.0 - alpha_bar.cos())).clamp(0.25, 0.5);
    let p_load = 1.0 / x_bar - 2.0;

    // --- Pass 2: the sail's chordwise extent, over every panel vertex (not just
    // centroids) so xi spans the full chord.
    let mut s_min = f64::MAX;
    let mut s_max = f64::MIN;
    if chord_dir != DVec3::ZERO {
        for panel in &world.canopy_panels {
            for idx in [panel.p1, panel.p2, panel.p3] {
                let s = world.particles.pos[idx].dot(chord_dir);
                s_min = s_min.min(s);
                s_max = s_max.max(s);
            }
        }
    }
    let c_sail = s_max - s_min;

    // Degenerate flow reference (flow exactly normal to the sail, or no chordwise
    // extent): the load is chordwise-symmetric, so fall back to uniform loading.
    let uniform = chord_dir == DVec3::ZERO || !c_sail.is_finite() || c_sail < 1e-9;

    // --- Pass 3: area-weighted normalization, so the redistribution preserves
    // the sail's total load rather than rescaling it.
    let weight_of = |centroid: DVec3| -> f64 {
        if uniform {
            1.0
        } else {
            chordwise_weight((centroid.dot(chord_dir) - s_min) / c_sail, p_load)
        }
    };
    let mut norm_acc = 0.0;
    for panel in &world.canopy_panels {
        if let Some((centroid, area, _)) = panel_geom(world, panel) {
            norm_acc += area * weight_of(centroid);
        }
    }
    let k_norm = if norm_acc > 1e-12 {
        norm_acc / area_sum
    } else {
        1.0
    };

    // --- Pass 4: apply each panel's scaled load at its own centroid (equal split
    // over the 3 vertices). The pitching moment now arises from how the load
    // varies across the sail, which is where a real sail's moment comes from.
    for panel_idx in 0..world.canopy_panels.len() {
        let panel = world.canopy_panels[panel_idx];
        let Some((centroid, _, _)) = panel_geom(world, &panel) else {
            continue;
        };
        let scale = weight_of(centroid) / k_norm;

        // The scale multiplies the whole resultant, including the small
        // parasitic-friction part of C_D, which is really uniform over the sail
        // rather than chordwise-distributed. It is ~10% of C_D at working
        // incidence and acts nearly in-plane, so it carries almost no pitching
        // moment; separating it is not worth a second coefficient path.
        let readback = &mut world.aero_readback[panel_idx];
        readback.lift *= scale;
        readback.drag *= scale;
        let f_total = readback.lift + readback.drag;

        for idx in [panel.p1, panel.p2, panel.p3] {
            let w = world.particles.inv_mass[idx];
            if w > 0.0 {
                world.particles.vel[idx] += h * (f_total / 3.0) * w;
            }
        }
    }
}
