//! Unsteady aerodynamic corrections (masterplan §6.4): the `aero_correction`
//! seam called after the base panel-method forces each substep.
//!
//! Currently implements **added mass**: a light fabric panel accelerating
//! normal to its plane also accelerates the surrounding air. Per panel the
//! entrained mass is modeled as the broadside added mass of the
//! equivalent-area flat disk (Lamb, *Hydrodynamics* §102):
//!
//!   m_a = (8/3) · ρ · a³,  with  a = sqrt(A/π)
//!
//! Implementation is *implicit* rather than a finite-difference force: after
//! all force kicks, the change in the panel's mean normal velocity since the
//! previous substep is rescaled by M/(M + m_a) (M = summed vertex mass). An
//! explicit F = -m_a·dv/dt force diverges whenever m_a exceeds the fabric
//! mass, the typical case for kite canopy (displaced air is heavier than
//! the fabric). This form is unconditionally stable instead and reproduces
//! the exact steady dynamics a = F/(M + m_a).
//!
//! Dynamic-stall lag remains deferred: the base model
//! (`flat_plate_coefficients`) has no stall curve to lag yet.

use crate::world::World;

const RHO_AIR: f64 = 1.225; // kg/m^3, matches panel_method

/// Equivalent-disk broadside added mass for a panel of area `area`.
pub fn panel_added_mass(area: f64) -> f64 {
    let a = (area / std::f64::consts::PI).sqrt();
    (8.0 / 3.0) * RHO_AIR * a * a * a
}

/// Applies all unsteady aero corrections. Called once per substep, after the
/// quasi-steady panel forces; gated by `Config::added_mass_enabled`.
pub fn apply_aero_corrections(world: &mut World, _h: f64) {
    if !world.cfg.added_mass_enabled {
        return;
    }

    for i in 0..world.canopy_panels.len() {
        let panel = world.canopy_panels[i];

        let w1 = world.particles.inv_mass[panel.p1];
        let w2 = world.particles.inv_mass[panel.p2];
        let w3 = world.particles.inv_mass[panel.p3];
        // A pinned vertex anchors the panel; rigid normal-velocity rescaling
        // would fight the pin, so skip (the pin dominates the dynamics anyway).
        if w1 <= 0.0 || w2 <= 0.0 || w3 <= 0.0 {
            continue;
        }
        let m_total = 1.0 / w1 + 1.0 / w2 + 1.0 / w3;

        let p1 = world.particles.pos[panel.p1];
        let p2 = world.particles.pos[panel.p2];
        let p3 = world.particles.pos[panel.p3];
        let cross = (p2 - p1).cross(p3 - p1);
        let double_area = cross.length();
        if double_area < 1e-12 {
            continue;
        }
        let normal = cross / double_area;
        let m_a = panel_added_mass(0.5 * double_area);

        let mean_vel = (world.particles.vel[panel.p1]
            + world.particles.vel[panel.p2]
            + world.particles.vel[panel.p3])
            / 3.0;

        if let Some(prev) = panel.prev_mean_vel {
            // Normal component of the velocity gained since last substep,
            // rescaled so the panel + entrained air accelerate together.
            let dv_n = (mean_vel - prev).dot(normal);
            let correction = -dv_n * m_a / (m_total + m_a) * normal;
            world.particles.vel[panel.p1] += correction;
            world.particles.vel[panel.p2] += correction;
            world.particles.vel[panel.p3] += correction;
            world.canopy_panels[i].prev_mean_vel = Some(mean_vel + correction);
        } else {
            world.canopy_panels[i].prev_mean_vel = Some(mean_vel);
        }
    }
}
