use glam::DVec3;
use noise::{NoiseFn, Perlin};

/// A divergence-free curl-noise turbulence field generator.
pub struct CurlNoiseField {
    perlin_x: Perlin,
    perlin_y: Perlin,
    perlin_z: Perlin,
    /// Integral length scale L of the turbulence (meters).
    pub length_scale: f64,
    /// Number of fractal octaves.
    pub octaves: usize,
}

impl CurlNoiseField {
    /// Creates a new CurlNoiseField from a seed.
    pub fn new(seed: u64, length_scale: f64, octaves: usize) -> Self {
        Self {
            perlin_x: Perlin::new(seed as u32),
            perlin_y: Perlin::new((seed + 1) as u32),
            perlin_z: Perlin::new((seed + 2) as u32),
            length_scale,
            octaves,
        }
    }

    /// Evaluates the raw vector potential at a given spatial position.
    pub fn potential_raw(&self, p: DVec3) -> DVec3 {
        let f0 = 1.0 / self.length_scale;
        let mut pot = DVec3::ZERO;
        let mut amp = 1.0;
        let mut freq = f0;

        for _ in 0..self.octaves {
            // Sample Perlin noise in [-1, 1] range
            let px = self.perlin_x.get([p.x * freq, p.y * freq, p.z * freq]) * amp;
            let py = self.perlin_y.get([p.x * freq, p.y * freq, p.z * freq]) * amp;
            let pz = self.perlin_z.get([p.x * freq, p.y * freq, p.z * freq]) * amp;

            pot.x += px;
            pot.y += py;
            pot.z += pz;

            amp *= 0.5;
            freq *= 2.0;
        }

        pot
    }

    /// Computes the divergence-free turbulence velocity vector at a given position and time.
    /// Uses central finite differences to take the curl of the vector potential.
    #[allow(clippy::too_many_arguments)]
    pub fn turbulence_at(
        &self,
        pos: DVec3,
        t: f64,
        v_ref: f64,
        h_ref: f64,
        p_exponent: f64,
        wind_dir: DVec3,
        turbulence_intensity: f64,
    ) -> DVec3 {
        let eps = 1e-5;
        let l_scale = self.length_scale;

        let pot_at = |p: DVec3| {
            let y = p.y.max(0.1);
            let v_mean = v_ref * (y / h_ref).powf(p_exponent);
            let sigma = turbulence_intensity * v_mean;

            // Advect coordinates downwind (Taylor's frozen turbulence hypothesis)
            let v_advect = v_ref * wind_dir.normalize();
            let p_advect = p - v_advect * t;

            // Scale potential by L * sigma to make curl derivative independent of L
            (l_scale * sigma) * self.potential_raw(p_advect)
        };

        let p_xp = pot_at(pos + DVec3::new(eps, 0.0, 0.0));
        let p_xm = pot_at(pos - DVec3::new(eps, 0.0, 0.0));
        let p_yp = pot_at(pos + DVec3::new(0.0, eps, 0.0));
        let p_ym = pot_at(pos - DVec3::new(0.0, eps, 0.0));
        let p_zp = pot_at(pos + DVec3::new(0.0, 0.0, eps));
        let p_zm = pot_at(pos - DVec3::new(0.0, 0.0, eps));

        // Curl components: v = curl(Psi)
        // v_x = dPsi_z/dy - dPsi_y/dz
        let curl_x = (p_yp.z - p_ym.z) / (2.0 * eps) - (p_zp.y - p_zm.y) / (2.0 * eps);
        // v_y = dPsi_x/dz - dPsi_z/dx
        let curl_y = (p_zp.x - p_zm.x) / (2.0 * eps) - (p_xp.z - p_xm.z) / (2.0 * eps);
        // v_z = dPsi_y/dx - dPsi_x/dy
        let curl_z = (p_xp.y - p_xm.y) / (2.0 * eps) - (p_yp.x - p_ym.x) / (2.0 * eps);

        // Empirically calibrated factor to make standard deviation match target turbulence intensity (sigma)
        let calibration_factor = 2.15;

        DVec3::new(curl_x, curl_y, curl_z) / calibration_factor
    }
}
