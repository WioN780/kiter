//! Thermal/convective cells (masterplan §7.5): localized rising/sinking air
//! columns modeled as drifting radial-basis blobs of vertical velocity that
//! grow and dissipate over their lifetime.
//!
//! Each cell contributes only a vertical velocity whose magnitude depends on
//! horizontal distance from the (drifting) column axis — so the field is
//! *analytically divergence-free*: v_x = v_z = 0 and ∂v_y/∂y = 0.

use glam::DVec3;
use serde::{Deserialize, Serialize};

/// A drifting thermal column of rising (or sinking, negative strength) air.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ThermalCell {
    /// Column axis position (at `start_time`), on the ground plane.
    pub center: DVec3,
    /// Gaussian core radius of the column (meters).
    pub radius: f64,
    /// Peak vertical velocity at the axis at mid-life (m/s); negative for
    /// sink columns.
    pub strength: f64,
    /// Time the cell is born (seconds).
    pub start_time: f64,
    /// Total lifetime (seconds); the cell grows then dissipates over this
    /// span with a smooth sin² envelope.
    pub lifetime: f64,
    /// Horizontal drift velocity of the column axis (m/s), typically a
    /// fraction of the mean wind.
    pub drift: DVec3,
}

impl ThermalCell {
    /// Evaluates this cell's wind contribution at a position and time.
    pub fn velocity_at(&self, pos: DVec3, t: f64) -> DVec3 {
        let age = t - self.start_time;
        if age <= 0.0 || age >= self.lifetime || self.lifetime <= 0.0 {
            return DVec3::ZERO;
        }

        // Smooth grow/dissipate envelope: 0 at birth and death, 1 at mid-life.
        let tau = age / self.lifetime;
        let envelope = (std::f64::consts::PI * tau).sin().powi(2);

        // Horizontal distance from the drifted column axis.
        let axis = self.center + self.drift * age;
        let dx = pos.x - axis.x;
        let dz = pos.z - axis.z;
        let d2 = dx * dx + dz * dz;

        let r2 = self.radius * self.radius;
        if r2 <= 0.0 {
            return DVec3::ZERO;
        }
        let radial = (-d2 / r2).exp();

        DVec3::new(0.0, self.strength * envelope * radial, 0.0)
    }
}
