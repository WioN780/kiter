/// A position-based compliant unilateral (tension-only) distance constraint between two particles.
/// Used for bridle lines that go slack when compressed.
#[derive(Clone, Debug)]
pub struct UnilateralDistanceConstraint {
    /// Index of the first particle.
    pub p1: usize,
    /// Index of the second particle.
    pub p2: usize,
    /// The maximum length of the line (at rest/taut).
    pub rest_length: f64,
    /// Line compliance $\alpha$ (Dyneema is near-zero).
    pub compliance: f64,
    /// Accumulated Lagrange multiplier $\lambda$ for this substep (always $\le 0$).
    pub lambda: f64,
    /// Line diameter (for drag calculations).
    pub diameter: f64,
}

impl UnilateralDistanceConstraint {
    /// Creates a new UnilateralDistanceConstraint.
    pub fn new(p1: usize, p2: usize, rest_length: f64, compliance: f64, diameter: f64) -> Self {
        Self {
            p1,
            p2,
            rest_length,
            compliance,
            lambda: 0.0,
            diameter,
        }
    }
}
