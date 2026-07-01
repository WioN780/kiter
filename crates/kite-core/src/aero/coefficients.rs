/// Computes the aerodynamic coefficients for a flat plate / thin membrane
/// over the full angle of attack $\alpha$ (in radians).
/// Returns (C_N, C_L, C_D) where:
/// - C_N is the normal force coefficient.
/// - C_L is the lift coefficient.
/// - C_D is the drag coefficient.
pub fn flat_plate_coefficients(alpha: f64) -> (f64, f64, f64) {
    let cn_max = 1.1;
    let cd0 = 0.04; // parasitic friction drag

    let cn = cn_max * (2.0 * alpha).sin();
    let cl = cn * alpha.cos();
    let cd = cn * alpha.sin() + cd0;

    (cn, cl, cd)
}
