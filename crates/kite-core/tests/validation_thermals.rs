//! Milestone 10b acceptance: thermal/convective cells (masterplan §7.5).
//!
//! The cell field is vertical-only with no y-dependence, so it must be
//! divergence-free to numerical precision, obey its sin² grow/dissipate
//! envelope, and drift with the configured velocity.

use glam::DVec3;
use kite_core::wind::{wind_at, ThermalCell, WindConfig};

fn test_cell() -> ThermalCell {
    ThermalCell {
        center: DVec3::new(5.0, 0.0, -3.0),
        radius: 8.0,
        strength: 2.5,
        start_time: 1.0,
        lifetime: 60.0,
        drift: DVec3::new(0.4, 0.0, 0.1),
    }
}

#[test]
fn test_thermal_divergence_free() {
    let cell = test_cell();
    let t = 20.0;
    let eps = 1e-4;

    // Central-difference divergence at a grid of sample points.
    for &x in &[-10.0, 0.0, 5.0, 12.0] {
        for &y in &[0.5, 10.0, 40.0] {
            for &z in &[-9.0, -3.0, 4.0] {
                let p = DVec3::new(x, y, z);
                let dvx = (cell.velocity_at(p + DVec3::X * eps, t).x
                    - cell.velocity_at(p - DVec3::X * eps, t).x)
                    / (2.0 * eps);
                let dvy = (cell.velocity_at(p + DVec3::Y * eps, t).y
                    - cell.velocity_at(p - DVec3::Y * eps, t).y)
                    / (2.0 * eps);
                let dvz = (cell.velocity_at(p + DVec3::Z * eps, t).z
                    - cell.velocity_at(p - DVec3::Z * eps, t).z)
                    / (2.0 * eps);
                let div = dvx + dvy + dvz;
                assert!(
                    div.abs() < 1e-9,
                    "divergence {div} at {p:?} (should be exactly zero)"
                );
            }
        }
    }
}

#[test]
fn test_thermal_lifecycle_envelope() {
    let cell = test_cell();
    let axis_at = |t: f64| cell.center + cell.drift * (t - cell.start_time);

    // Dead before birth and after death.
    assert_eq!(cell.velocity_at(axis_at(1.0), 0.5), DVec3::ZERO);
    assert_eq!(cell.velocity_at(axis_at(1.0), 1.0), DVec3::ZERO);
    assert_eq!(cell.velocity_at(axis_at(61.0), 61.0), DVec3::ZERO);
    assert_eq!(cell.velocity_at(axis_at(61.0), 100.0), DVec3::ZERO);

    // Peak strength on the axis at mid-life.
    let mid = 1.0 + 30.0;
    let v_mid = cell.velocity_at(axis_at(mid), mid);
    assert!((v_mid.y - cell.strength).abs() < 1e-12);
    assert_eq!(v_mid.x, 0.0);
    assert_eq!(v_mid.z, 0.0);

    // Weaker early in life, still rising.
    let early = 1.0 + 3.0;
    let v_early = cell.velocity_at(axis_at(early), early);
    assert!(v_early.y > 0.0 && v_early.y < 0.3 * cell.strength);

    // Gaussian radial falloff: exp(-d²/r²) = exp(-4) at d = 2·radius.
    let off = axis_at(mid) + DVec3::new(2.0 * cell.radius, 0.0, 0.0);
    let v_off = cell.velocity_at(off, mid);
    assert!((v_off.y - cell.strength * (-4.0f64).exp()).abs() < 1e-12);
}

#[test]
fn test_thermal_drifts_with_configured_velocity() {
    let cell = test_cell();
    let t1 = 21.0; // age 20
    let t2 = 41.0; // age 40, symmetric envelope value (sin²(π/3) each side of mid)

    // The peak (axis) location must move by drift × Δage.
    let axis1 = cell.center + cell.drift * 20.0;
    let axis2 = cell.center + cell.drift * 40.0;

    let v1 = cell.velocity_at(axis1, t1).y;
    let v2 = cell.velocity_at(axis2, t2).y;
    // sin²(π·1/3) == sin²(π·2/3): same envelope, so identical peak values.
    assert!((v1 - v2).abs() < 1e-12);

    // The old axis position must now be off-peak.
    let v_stale = cell.velocity_at(axis1, t2).y;
    assert!(v_stale < v2);
}

#[test]
fn test_thermals_summed_into_wind_at() {
    let mut cfg = WindConfig {
        v_ref: 0.0, // no mean wind, no turbulence: isolate the thermal
        ..WindConfig::default()
    };
    cfg.thermals.push(test_cell());

    let cell = test_cell();
    let mid = 31.0;
    let axis = cell.center + cell.drift * 30.0;
    let v = wind_at(axis, mid, &cfg);
    assert!((v.y - cell.strength).abs() < 1e-12);

    // Sink cell superposes.
    cfg.thermals.push(ThermalCell {
        strength: -1.0,
        ..test_cell()
    });
    let v_sum = wind_at(axis, mid, &cfg);
    assert!((v_sum.y - (cell.strength - 1.0)).abs() < 1e-12);
}
