use glam::DVec3;

/// Physical material properties.
#[derive(Clone, Copy, Debug)]
pub struct Material {
    /// Young's modulus $E$ in Pascals ($\text{Pa}$).
    pub youngs_modulus: f64,
    /// Shear modulus $G$ in Pascals ($\text{Pa}$).
    pub shear_modulus: f64,
    /// Density $\rho$ in $\text{kg/m}^3$.
    pub density: f64,
    /// Tensile/yield strength in Pascals ($\text{Pa}$).
    pub tensile_strength: f64,
}

impl Material {
    /// Pultruded Carbon Fiber Rod ballpark values.
    pub fn carbon_fiber() -> Self {
        Self {
            youngs_modulus: 1.0e11, // 100 GPa
            shear_modulus: 4.5e9,   // 4.5 GPa
            density: 1550.0,
            tensile_strength: 9.0e8, // 900 MPa
        }
    }

    /// Fiberglass Rod ballpark values.
    pub fn fiberglass() -> Self {
        Self {
            youngs_modulus: 4.0e10, // 40 GPa
            shear_modulus: 4.0e9,   // 4.0 GPa
            density: 1950.0,
            tensile_strength: 8.0e8, // 800 MPa
        }
    }
}

/// Geometric properties of the cross-section of a rod segment.
#[derive(Clone, Copy, Debug)]
pub enum SectionGeometry {
    /// Solid round rod with a given radius.
    SolidRound { radius: f64 },
    /// Hollow tube with outer radius and wall thickness.
    Tube { radius: f64, thickness: f64 },
}

impl SectionGeometry {
    /// Computes the cross-sectional area $A$ in $\text{m}^2$.
    pub fn area(&self) -> f64 {
        match *self {
            SectionGeometry::SolidRound { radius } => std::f64::consts::PI * radius.powi(2),
            SectionGeometry::Tube { radius, thickness } => {
                let inner_r = radius - thickness;
                std::f64::consts::PI * (radius.powi(2) - inner_r.powi(2))
            }
        }
    }

    /// Computes the second moment of area $I$ in $\text{m}^4$ (bending).
    pub fn area_moment_of_inertia(&self) -> f64 {
        match *self {
            SectionGeometry::SolidRound { radius } => std::f64::consts::PI * radius.powi(4) / 4.0,
            SectionGeometry::Tube { radius, thickness } => {
                let inner_r = radius - thickness;
                std::f64::consts::PI * (radius.powi(4) - inner_r.powi(4)) / 4.0
            }
        }
    }

    /// Computes the polar moment of area $J$ in $\text{m}^4$ (torsion).
    pub fn polar_moment_of_inertia(&self) -> f64 {
        match *self {
            SectionGeometry::SolidRound { radius } => std::f64::consts::PI * radius.powi(4) / 2.0,
            SectionGeometry::Tube { radius, thickness } => {
                let inner_r = radius - thickness;
                std::f64::consts::PI * (radius.powi(4) - inner_r.powi(4)) / 2.0
            }
        }
    }

    /// Computes the diagonal inertia tensor ($I_x, I_y, I_z$) in $\text{kg}\cdot\text{m}^2$ for a segment.
    pub fn compute_inertia(&self, length: f64, mass: f64) -> DVec3 {
        let r = match *self {
            SectionGeometry::SolidRound { radius } => radius,
            SectionGeometry::Tube { radius, .. } => radius,
        };
        // Longitudinal inertia (twist-z)
        let i_z = match *self {
            SectionGeometry::SolidRound { .. } => 0.5 * mass * r.powi(2),
            SectionGeometry::Tube { thickness, .. } => {
                let inner_r = r - thickness;
                0.5 * mass * (r.powi(2) + inner_r.powi(2))
            }
        };
        // Transverse inertia (bend-x, bend-y)
        let i_xy = (1.0 / 12.0) * mass * length.powi(2) + 0.25 * mass * r.powi(2);
        DVec3::new(i_xy, i_xy, i_z)
    }

    /// Diagonal *inverse* inertia ($I_x^{-1}, I_y^{-1}, I_z^{-1}$) of a segment: the
    /// generalized rotational inverse mass the XPBD solver expects in
    /// `OrientationSet::inv_inertia`. A zero/degenerate principal moment maps to a
    /// zero inverse (rotationally pinned about that axis), matching how the solver
    /// reads `inv_inertia == 0` as "infinitely heavy".
    pub fn compute_inv_inertia(&self, length: f64, mass: f64) -> DVec3 {
        let i = self.compute_inertia(length, mass);
        let inv = |x: f64| if x > 0.0 { 1.0 / x } else { 0.0 };
        DVec3::new(inv(i.x), inv(i.y), inv(i.z))
    }
}

/// Maps material properties and section geometry to stretch-shear compliance.
/// Returns a vector containing (shear_x, shear_y, stretch_z) compliances.
pub fn stretch_shear_compliance(material: &Material, geom: &SectionGeometry, length: f64) -> DVec3 {
    let a = geom.area();
    let comp_stretch = length / (material.youngs_modulus * a);
    // Deliberate simplification: omits the Timoshenko shear shape factor kappa
    // (~0.9 solid round, ~0.5 thin tube), so shear reads up to ~2x too stiff.
    // Add kappa to SectionGeometry if shear-dominated deflection ever matters.
    let comp_shear = length / (material.shear_modulus * a);
    DVec3::new(comp_shear, comp_shear, comp_stretch)
}

/// Maps material properties and section geometry to bend-twist compliance.
/// Returns a vector containing (bend_x, bend_y, twist_z) compliances.
pub fn bend_twist_compliance(
    material: &Material,
    geom: &SectionGeometry,
    average_length: f64,
) -> DVec3 {
    let i = geom.area_moment_of_inertia();
    let j = geom.polar_moment_of_inertia();
    let comp_bend = average_length / (4.0 * material.youngs_modulus * i);
    let comp_twist = average_length / (4.0 * material.shear_modulus * j);
    DVec3::new(comp_bend, comp_bend, comp_twist)
}
