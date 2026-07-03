/// A position-based compliant dihedral angle bending constraint between two triangles sharing an edge.
/// Couples 4 particles: p1, p2 (shared edge), p3, p4 (opposite vertices).
#[derive(Clone, Debug)]
pub struct DihedralBendingConstraint {
    /// Index of the first edge particle.
    pub p1: usize,
    /// Index of the second edge particle.
    pub p2: usize,
    /// Index of the first triangle's opposite particle.
    pub p3: usize,
    /// Index of the second triangle's opposite particle.
    pub p4: usize,
    /// The target rest dihedral angle (in radians).
    pub rest_angle: f64,
    /// Bending compliance $\alpha$.
    pub compliance: f64,
    /// Accumulated Lagrange multiplier $\lambda$ for this substep.
    pub lambda: f64,
}

impl DihedralBendingConstraint {
    /// Creates a new DihedralBendingConstraint.
    pub fn new(
        p1: usize,
        p2: usize,
        p3: usize,
        p4: usize,
        rest_angle: f64,
        compliance: f64,
    ) -> Self {
        Self {
            p1,
            p2,
            p3,
            p4,
            rest_angle,
            compliance,
            lambda: 0.0,
        }
    }
}
