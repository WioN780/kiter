pub mod curl_noise;
pub mod gust_events;
pub mod mean_profile;
pub mod thermal_cells;

use glam::DVec3;
use serde::{Deserialize, Serialize};
use std::cell::RefCell;

pub use curl_noise::CurlNoiseField;
pub use gust_events::GustEvent;
pub use mean_profile::mean_wind;
pub use thermal_cells::ThermalCell;

thread_local! {
    // `CurlNoiseField::new` rebuilds 3 Perlin permutation tables, and
    // `wind_at` is called from hot loops (per canopy panel / bridle segment /
    // spar segment / substep). The field is fully determined by
    // (seed, length_scale, octaves), so cache the last-built one per thread
    // instead of reconstructing it every call. This is a pure memoization of
    // a deterministic function of `cfg`: it changes no observable output and
    // introduces no cross-call state dependence, so it preserves `World::step`
    // determinism/purity despite the thread-local storage.
    static FIELD_CACHE: RefCell<Option<(u64, usize, CurlNoiseField)>> = const { RefCell::new(None) };
}

/// Configuration for the wind field, including shear, turbulence, and gusts.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct WindConfig {
    /// Reference wind speed (m/s) at reference height.
    pub v_ref: f64,
    /// Reference height (meters) at which v_ref is defined.
    pub h_ref: f64,
    /// Wind shear power-law exponent.
    pub shear_exponent: f64,
    /// Wind direction vector (unit vector in horizontal plane).
    pub direction: DVec3,
    /// Target turbulence intensity ratio (standard deviation sigma / V_mean).
    pub turbulence_intensity: f64,
    /// Integral length scale L of the turbulence (meters).
    pub length_scale: f64,
    /// Number of fractal octaves.
    pub octaves: usize,
    /// Active gust events.
    pub gusts: Vec<GustEvent>,
    /// Active thermal/convective cells (§7.5).
    pub thermals: Vec<ThermalCell>,
    /// Seed for the turbulence field RNG.
    pub seed: u64,
}

impl Default for WindConfig {
    fn default() -> Self {
        Self {
            v_ref: 0.0,
            h_ref: 10.0,
            shear_exponent: 0.15,
            direction: DVec3::new(1.0, 0.0, 0.0),
            turbulence_intensity: 0.0, // defaults to zero (constant mean wind)
            length_scale: 20.0,
            octaves: 3,
            gusts: Vec::new(),
            thermals: Vec::new(),
            seed: 42,
        }
    }
}

/// Computes the total wind velocity vector at a given position and time.
/// Combines the mean shear profile, curl-noise turbulence, and discrete gusts.
pub fn wind_at(pos: DVec3, t: f64, cfg: &WindConfig) -> DVec3 {
    // A diverged solver can hand back a non-finite position mid-step, before
    // `World::step` returns and callers get a chance to notice. The Perlin
    // noise crate `unwrap()`s internally on NaN/Inf input and would hard
    // panic the process; short-circuit to zero here so divergence surfaces
    // as the intended "positions went non-finite" state instead of a crash.
    if !pos.is_finite() {
        return DVec3::ZERO;
    }

    // 1. Mean profile component
    let v_mean = mean_wind(
        pos,
        t,
        cfg.v_ref,
        cfg.h_ref,
        cfg.shear_exponent,
        cfg.direction,
    );

    // 2. Turbulence component (divergence-free curl-noise)
    let v_turb = if cfg.turbulence_intensity > 0.0 {
        FIELD_CACHE.with(|cache| {
            let mut cache = cache.borrow_mut();
            let stale = match &*cache {
                Some((seed, octaves, field)) => {
                    *seed != cfg.seed
                        || *octaves != cfg.octaves
                        || field.length_scale.to_bits() != cfg.length_scale.to_bits()
                }
                None => true,
            };
            if stale {
                *cache = Some((
                    cfg.seed,
                    cfg.octaves,
                    CurlNoiseField::new(cfg.seed, cfg.length_scale, cfg.octaves),
                ));
            }
            let (_, _, field) = cache.as_ref().expect("just populated above");
            field.turbulence_at(
                pos,
                t,
                cfg.v_ref,
                cfg.h_ref,
                cfg.shear_exponent,
                cfg.direction,
                cfg.turbulence_intensity,
            )
        })
    } else {
        DVec3::ZERO
    };

    // 3. Discrete gust events component
    let mut v_gust = DVec3::ZERO;
    for gust in &cfg.gusts {
        v_gust += gust.velocity_at(pos, t);
    }

    // 4. Thermal/convective cells (§7.5)
    let mut v_thermal = DVec3::ZERO;
    for cell in &cfg.thermals {
        v_thermal += cell.velocity_at(pos, t);
    }

    v_mean + v_turb + v_gust + v_thermal
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn non_finite_position_does_not_panic() {
        // A diverged solver step can hand this a NaN/Inf position; it must
        // short-circuit before reaching the Perlin noise sampling, which
        // panics on non-finite input (see the guard's comment).
        let cfg = WindConfig { turbulence_intensity: 0.3, ..WindConfig::default() };
        let v = wind_at(DVec3::new(f64::NAN, 0.0, 0.0), 0.0, &cfg);
        assert_eq!(v, DVec3::ZERO);
        let v = wind_at(DVec3::new(f64::INFINITY, 0.0, 0.0), 0.0, &cfg);
        assert_eq!(v, DVec3::ZERO);
    }
}
