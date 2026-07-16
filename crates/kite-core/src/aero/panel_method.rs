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

/// Computes panel-method aerodynamic forces for all canopy panels (triangles)
/// and applies them as velocity updates to their vertices.
pub fn apply_canopy_aerodynamics(world: &mut World, h: f64) {
    let rho = 1.225; // standard air density kg/m^3

    world.aero_readback.clear();
    world.aero_readback.resize(world.canopy_panels.len(), PanelAero::default());

    for (panel_idx, panel) in world.canopy_panels.iter().enumerate() {
        let p1 = world.particles.pos[panel.p1];
        let p2 = world.particles.pos[panel.p2];
        let p3 = world.particles.pos[panel.p3];

        let v1 = world.particles.vel[panel.p1];
        let v2 = world.particles.vel[panel.p2];
        let v3 = world.particles.vel[panel.p3];

        // Centroid and centroid velocity
        let centroid = (p1 + p2 + p3) / 3.0;
        let v_triangle = (v1 + v2 + v3) / 3.0;

        // Relative wind at centroid
        let wind_vel = crate::wind::wind_at(centroid, world.time, &world.cfg.wind);
        let v_rel = wind_vel - v_triangle;
        let v_rel_mag = v_rel.length();
        if v_rel_mag < 1e-6 {
            continue;
        }
        let v_rel_unit = v_rel / v_rel_mag;

        // Triangle geometry
        let edge1 = p2 - p1;
        let edge2 = p3 - p1;
        let cross = edge1.cross(edge2);
        let double_area = cross.length();
        if double_area < 1e-12 {
            continue;
        }
        let area = 0.5 * double_area;
        let normal = cross / double_area; // unit normal

        // Angle of attack alpha: sin(alpha) = v_rel_unit . normal
        let sin_alpha = v_rel_unit.dot(normal).clamp(-1.0, 1.0);
        let alpha = sin_alpha.asin();

        // Dynamic pressure q = 0.5 * rho * |v_rel|^2
        let q = 0.5 * rho * v_rel_mag * v_rel_mag;

        // Aerodynamic coefficients
        let (_cn, cl, cd) = flat_plate_coefficients(alpha);

        // Drag force is parallel to relative wind
        let f_drag = q * area * cd * v_rel_unit;

        // Lift force is perpendicular to relative wind, in the plane of normal and relative wind
        let cos_alpha = alpha.cos();
        let f_lift = if cos_alpha.abs() > 1e-6 {
            let lift_dir = (normal - sin_alpha * v_rel_unit).normalize();
            q * area * cl * lift_dir
        } else {
            DVec3::ZERO
        };

        world.aero_readback[panel_idx] = PanelAero { lift: f_lift, drag: f_drag, alpha };

        let f_total = f_lift + f_drag;

        // Split force evenly to its 3 vertices
        let f_third = f_total / 3.0;

        let w1 = world.particles.inv_mass[panel.p1];
        let w2 = world.particles.inv_mass[panel.p2];
        let w3 = world.particles.inv_mass[panel.p3];

        if w1 > 0.0 {
            world.particles.vel[panel.p1] += h * f_third * w1;
        }
        if w2 > 0.0 {
            world.particles.vel[panel.p2] += h * f_third * w2;
        }
        if w3 > 0.0 {
            world.particles.vel[panel.p3] += h * f_third * w3;
        }
    }
}
