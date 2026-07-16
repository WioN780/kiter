use glam::DVec3;

/// Computes the mean shear wind speed profile using the power-law relation:
/// V_mean(h) = V_ref * (h / h_ref)^p
/// Returns the wind velocity vector along the wind direction.
pub fn mean_wind(pos: DVec3, _t: f64, v_ref: f64, h_ref: f64, p: f64, direction: DVec3) -> DVec3 {
    let h = pos.y.max(0.1); // clamp to avoid division by zero or negative heights
    let speed = v_ref * (h / h_ref).powf(p);
    speed * direction.normalize()
}
