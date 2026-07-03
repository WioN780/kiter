pub mod curl_noise;
pub mod gust_events;
pub mod mean_profile;
pub mod thermal_cells;

use glam::DVec3;
use serde::{Deserialize, Serialize};

pub use curl_noise::CurlNoiseField;
pub use gust_events::GustEvent;
pub use mean_profile::mean_wind;
pub use thermal_cells::ThermalCell;

/// Configuration for the wind field, including shear, turbulence, and gusts.
#[derive(Clone, Debug, Serialize, Deserialize)]
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
        let field = CurlNoiseField::new(cfg.seed, cfg.length_scale, cfg.octaves);
        field.turbulence_at(
            pos,
            t,
            cfg.v_ref,
            cfg.h_ref,
            cfg.shear_exponent,
            cfg.direction,
            cfg.turbulence_intensity,
        )
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
