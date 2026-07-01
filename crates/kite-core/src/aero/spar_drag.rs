use crate::world::World;

/// Computes bluff cylinder cross-flow drag for all spar/tube segments
/// and applies the resulting forces to their end particles.
pub fn apply_spar_drag(world: &mut World, h: f64) {
    let rho = 1.225; // standard air density kg/m^3
    let cd_cyl = 1.2; // cylinder drag coefficient

    for b in &world.stretch_shear_constraints {
        let w1 = world.particles.inv_mass[b.p1];
        let w2 = world.particles.inv_mass[b.p2];
        if w1 <= 0.0 && w2 <= 0.0 {
            continue;
        }

        let p1 = world.particles.pos[b.p1];
        let p2 = world.particles.pos[b.p2];
        let diff = p2 - p1;
        let len = diff.length();
        if len < 1e-6 {
            continue;
        }
        let u = diff / len; // segment unit tangent

        // Midpoint velocity and position
        let v1 = world.particles.vel[b.p1];
        let v2 = world.particles.vel[b.p2];
        let v_mid = 0.5 * (v1 + v2);
        let p_mid = 0.5 * (p1 + p2);

        // Relative wind at midpoint
        let wind_vel = crate::wind::wind_at(p_mid, world.time, &world.cfg.wind);
        let v_rel = wind_vel - v_mid;

        // Decompose into axial and cross components
        let v_axial = v_rel.dot(u) * u;
        let v_cross = v_rel - v_axial;
        let v_cross_mag = v_cross.length();

        if v_cross_mag < 1e-6 {
            continue;
        }

        // Cross-flow drag force: F = 0.5 * rho * cd * V_cross^2 * diameter * length
        let force_mag = 0.5 * rho * cd_cyl * v_cross_mag * v_cross_mag * b.diameter * len;
        let force_vector = force_mag * (v_cross / v_cross_mag);

        // Split force equally between the two particles
        let f_half = 0.5 * force_vector;

        if w1 > 0.0 {
            world.particles.vel[b.p1] += h * f_half * w1;
        }
        if w2 > 0.0 {
            world.particles.vel[b.p2] += h * f_half * w2;
        }
    }
}
