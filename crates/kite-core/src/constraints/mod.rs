pub mod anchor;
pub mod bridle_unilateral;
pub mod cloth_bending;
pub mod cloth_stretch_shear;
pub mod ground_contact;
pub mod rod_cosserat;
pub mod self_collision;

use glam::DVec3;

pub use bridle_unilateral::UnilateralDistanceConstraint;
pub use cloth_bending::DihedralBendingConstraint;
pub use rod_cosserat::{BendTwistConstraint, ConstraintState, StretchShearConstraint};

/// A position-based compliant distance constraint between two particles.
#[derive(Clone, Debug)]
pub struct DistanceConstraint {
    /// Index of the first particle.
    pub p1: usize,
    /// Index of the second particle.
    pub p2: usize,
    /// Rest length of the connection.
    pub rest_length: f64,
    /// Constraint compliance $\alpha$ (inverse stiffness, $m/N$).
    pub compliance: f64,
    /// Accumulated Lagrange multiplier $\lambda$ for this substep.
    pub lambda: f64,
}

impl DistanceConstraint {
    /// Creates a new DistanceConstraint.
    pub fn new(p1: usize, p2: usize, rest_length: f64, compliance: f64) -> Self {
        Self {
            p1,
            p2,
            rest_length,
            compliance,
            lambda: 0.0,
        }
    }
}

/// A position-based compliant linear bending constraint between three particles.
/// Constrains $(p_1 - 2*p_2 + p_3) \approx \mathbf{rest\_value}$.
#[derive(Clone, Debug)]
pub struct BendingConstraint {
    /// Index of the first particle.
    pub p1: usize,
    /// Index of the middle particle.
    pub p2: usize,
    /// Index of the third particle.
    pub p3: usize,
    /// Rest bending vector value (precomputed from rest positions).
    pub rest_value: DVec3,
    /// Bending compliance $\alpha$ (inverse stiffness).
    pub compliance: f64,
    /// Accumulated vector Lagrange multiplier $\boldsymbol{\lambda}$ for this substep.
    pub lambda: DVec3,
}

impl BendingConstraint {
    /// Creates a new BendingConstraint with a given rest curvature vector.
    pub fn new(p1: usize, p2: usize, p3: usize, rest_value: DVec3, compliance: f64) -> Self {
        Self {
            p1,
            p2,
            p3,
            rest_value,
            compliance,
            lambda: DVec3::ZERO,
        }
    }
}
