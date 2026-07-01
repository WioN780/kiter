use glam::DVec3;

/// Struct-of-arrays representation of particles.
#[derive(Clone, Debug)]
pub struct ParticleSet {
    /// Current position of each particle.
    pub pos: Vec<DVec3>,
    /// Position of each particle at the beginning of the current substep.
    pub prev_pos: Vec<DVec3>,
    /// Predicted position of each particle during constraint solving.
    pub pred_pos: Vec<DVec3>,
    /// Velocity of each particle.
    pub vel: Vec<DVec3>,
    /// Inverse mass ($1/m$) of each particle. Pinned particles have inverse mass 0.0.
    pub inv_mass: Vec<f64>,
}

impl ParticleSet {
    /// Creates a new, empty ParticleSet.
    pub fn new() -> Self {
        Self {
            pos: Vec::new(),
            prev_pos: Vec::new(),
            pred_pos: Vec::new(),
            vel: Vec::new(),
            inv_mass: Vec::new(),
        }
    }

    /// Adds a particle to the set with a given position and mass.
    /// Returns the particle index.
    pub fn add_particle(&mut self, pos: DVec3, mass: f64) -> usize {
        let inv_mass = if mass <= 0.0 { 0.0 } else { 1.0 / mass };
        self.pos.push(pos);
        self.prev_pos.push(pos);
        self.pred_pos.push(pos);
        self.vel.push(DVec3::ZERO);
        self.inv_mass.push(inv_mass);
        self.pos.len() - 1
    }

    /// Returns the number of particles in the set.
    pub fn len(&self) -> usize {
        self.pos.len()
    }

    /// Returns true if the set contains no particles.
    pub fn is_empty(&self) -> bool {
        self.pos.is_empty()
    }

    /// Clears all particles.
    pub fn clear(&mut self) {
        self.pos.clear();
        self.prev_pos.clear();
        self.pred_pos.clear();
        self.vel.clear();
        self.inv_mass.clear();
    }
}

impl Default for ParticleSet {
    fn default() -> Self {
        Self::new()
    }
}
