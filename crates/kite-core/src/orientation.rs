use glam::{DQuat, DVec3};

/// Struct-of-arrays representation of segment orientations.
#[derive(Clone, Debug)]
pub struct OrientationSet {
    /// Current orientation quaternion of each segment.
    pub quat: Vec<DQuat>,
    /// Orientation of each segment at the beginning of the current substep.
    pub prev_quat: Vec<DQuat>,
    /// Angular velocity of each segment in the local material frame.
    pub omega: Vec<DVec3>,
    /// Diagonal inverse inertia tensor ($I_x^{-1}, I_y^{-1}, I_z^{-1}$) of each segment in the local frame.
    pub inv_inertia: Vec<DVec3>,
}

impl OrientationSet {
    /// Creates a new, empty OrientationSet.
    pub fn new() -> Self {
        Self {
            quat: Vec::new(),
            prev_quat: Vec::new(),
            omega: Vec::new(),
            inv_inertia: Vec::new(),
        }
    }

    /// Adds a segment orientation.
    pub fn add_segment(&mut self, quat: DQuat, inv_inertia: DVec3) -> usize {
        self.quat.push(quat);
        self.prev_quat.push(quat);
        self.omega.push(DVec3::ZERO);
        self.inv_inertia.push(inv_inertia);
        self.quat.len() - 1
    }

    /// Returns the number of segment orientations.
    pub fn len(&self) -> usize {
        self.quat.len()
    }

    /// Returns true if the set is empty.
    pub fn is_empty(&self) -> bool {
        self.quat.is_empty()
    }

    /// Clears all segments.
    pub fn clear(&mut self) {
        self.quat.clear();
        self.prev_quat.clear();
        self.omega.clear();
        self.inv_inertia.clear();
    }
}

impl Default for OrientationSet {
    fn default() -> Self {
        Self::new()
    }
}
