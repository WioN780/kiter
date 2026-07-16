use glam::DVec3;

/// State of a physical constraint, representing failure modes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ConstraintState {
    /// Fully functioning elastic/rigid behavior.
    Active,
    /// Folded behavior (specific to underinflated leading edge bladders).
    Folded,
    /// Completely severed or snapped constraint.
    Broken,
}

/// A position-based compliant stretch and shear constraint for a Cosserat rod segment.
/// Couples two particles (endpoints) and one segment orientation.
#[derive(Clone, Debug)]
pub struct StretchShearConstraint {
    /// Index of the first particle.
    pub p1: usize,
    /// Index of the second particle.
    pub p2: usize,
    /// Index of the segment orientation in the `OrientationSet`.
    pub q_index: usize,
    /// Segment rest length.
    pub rest_length: f64,
    /// Compliance $\alpha$ along the local frame: (shear_x, shear_y, stretch_z).
    pub compliance: DVec3,
    /// Accumulated Lagrange multipliers $\boldsymbol{\lambda}$ for this substep.
    pub lambda: DVec3,
    /// Segment diameter (for drag calculations).
    pub diameter: f64,
}

impl StretchShearConstraint {
    /// Creates a new StretchShearConstraint.
    pub fn new(
        p1: usize,
        p2: usize,
        q_index: usize,
        rest_length: f64,
        compliance: DVec3,
        diameter: f64,
    ) -> Self {
        Self {
            p1,
            p2,
            q_index,
            rest_length,
            compliance,
            lambda: DVec3::ZERO,
            diameter,
        }
    }
}

/// A position-based compliant bending and twisting constraint between two adjacent segments.
/// Couples two segment orientations.
#[derive(Clone, Debug)]
pub struct BendTwistConstraint {
    /// Index of the first segment orientation in the `OrientationSet`.
    pub q1_index: usize,
    /// Index of the second segment orientation in the `OrientationSet`.
    pub q2_index: usize,
    /// Rest Darboux vector value.
    pub rest_value: DVec3,
    /// Compliance $\alpha$ along the local frame: (bend_x, bend_y, twist_z).
    pub compliance: DVec3,
    /// Accumulated Lagrange multipliers $\boldsymbol{\lambda}$ for this substep.
    pub lambda: DVec3,
    /// Maximum generalized bending/torsional impulse before yield occurs.
    pub yield_threshold: f64,
    /// True if this represents an inflatable leading edge (LE) tube.
    pub is_inflatable: bool,
    /// Current structural state of the joint.
    pub state: ConstraintState,
}

impl BendTwistConstraint {
    /// Creates a new BendTwistConstraint.
    pub fn new(
        q1_index: usize,
        q2_index: usize,
        rest_value: DVec3,
        compliance: DVec3,
        yield_threshold: f64,
        is_inflatable: bool,
    ) -> Self {
        Self {
            q1_index,
            q2_index,
            rest_value,
            compliance,
            lambda: DVec3::ZERO,
            yield_threshold,
            is_inflatable,
            state: ConstraintState::Active,
        }
    }
}
