use glam::DVec3;
use serde::{Serialize, Deserialize};

/// A discrete spatial 1-cosine gust event.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GustEvent {
    /// Time when the gust front passes the origin (t_start in seconds).
    pub start_time: f64,
    /// Spatial width of the gust front (length in meters).
    pub length: f64,
    /// Peak amplitude increase of the gust (m/s).
    pub amplitude: f64,
    /// Direction the gust travels and blows.
    pub direction: DVec3,
    /// Propagation velocity of the gust front (m/s, usually mean wind speed).
    pub propagation_velocity: f64,
}

impl GustEvent {
    /// Creates a new GustEvent.
    pub fn new(
        start_time: f64,
        length: f64,
        amplitude: f64,
        direction: DVec3,
        propagation_velocity: f64,
    ) -> Self {
        Self {
            start_time,
            length,
            amplitude,
            direction: direction.normalize(),
            propagation_velocity,
        }
    }

    /// Evaluates the gust velocity at a given position and time.
    pub fn velocity_at(&self, pos: DVec3, t: f64) -> DVec3 {
        if t < self.start_time {
            return DVec3::ZERO;
        }

        // Distance the gust front has traveled since start_time
        let d_front = self.propagation_velocity * (t - self.start_time);
        
        // Position along the gust direction
        let x = pos.dot(self.direction);
        
        // Coordinate within the gust front (0 at the front, increasing behind it)
        let xi = d_front - x;

        if xi >= 0.0 && xi <= self.length {
            let factor = 0.5 * (1.0 - (2.0 * std::f64::consts::PI * xi / self.length).cos());
            (self.amplitude * factor) * self.direction
        } else {
            DVec3::ZERO
        }
    }
}
