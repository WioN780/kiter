use glam::DVec3;
use crate::particles::ParticleSet;
use crate::orientation::OrientationSet;
use crate::constraints::{DistanceConstraint, BendingConstraint, StretchShearConstraint, BendTwistConstraint, ConstraintState, DihedralBendingConstraint, UnilateralDistanceConstraint};
use crate::solver;
use crate::aero::CanopyPanel;

/// Event emitted by the physics engine during simulation.
#[derive(Clone, Debug, PartialEq)]
pub enum Event {
    /// A solid rod segment has yielded past its tensile/bending limit and snapped.
    SparBroken { joint_index: usize },
    /// An inflatable tube joint has kinked/folded due to underpressure or excessive load.
    LeadingEdgeFolded { joint_index: usize },
}

/// Configuration parameters for the physics simulation.
#[derive(Clone, Debug)]
pub struct Config {
    /// Number of substeps per step (default 24).
    pub substeps: usize,
    /// Solver iterations per substep (default 1).
    pub iterations_per_substep: usize,
    /// Gravity acceleration vector (Y-up, right-handed).
    pub gravity: DVec3,
    /// Velocity damping factor (per second, e.g. 2.0).
    pub damping: f64,
    /// Bladder pressure of inflatable leading edge (LE) tubes (relative bar or Pa).
    pub bladder_pressure: f64,
    /// Pressure-stiffness coupling constant $k$ (stiffness $\propto 1 + k \cdot p$).
    pub k_pressure: f64,
    /// Wind field configuration.
    pub wind: crate::wind::WindConfig,
    /// Whether ground-plane collision is enabled.
    pub ground_collision_enabled: bool,
    /// Whether self-collision detection is enabled.
    pub self_collision_enabled: bool,
    /// Whether to solve constraints in parallel using graph coloring and Rayon.
    pub parallel_solve: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            substeps: 24,
            iterations_per_substep: 1,
            gravity: DVec3::new(0.0, -9.81, 0.0),
            damping: 2.0,
            bladder_pressure: 0.0,
            k_pressure: 1.0,
            wind: crate::wind::WindConfig::default(),
            ground_collision_enabled: false,
            self_collision_enabled: false,
            parallel_solve: std::env::var("KITER_PARALLEL_SOLVE").is_ok(),
        }
    }
}

/// The top-level state of the physical simulation.
pub struct World {
    /// Current simulation time in seconds.
    pub time: f64,
    /// Solver configuration parameters.
    pub cfg: Config,
    /// Set of particles in the simulation.
    pub particles: ParticleSet,
    /// Set of segment orientations in the simulation.
    pub orientations: OrientationSet,
    /// Active simple distance constraints.
    pub distance_constraints: Vec<DistanceConstraint>,
    /// Active simple bending constraints.
    pub bending_constraints: Vec<BendingConstraint>,
    /// Active Cosserat rod stretch-shear constraints.
    pub stretch_shear_constraints: Vec<StretchShearConstraint>,
    /// Active Cosserat rod bend-twist constraints.
    pub bend_twist_constraints: Vec<BendTwistConstraint>,
    /// Active cloth dihedral bending constraints.
    pub dihedral_bending_constraints: Vec<DihedralBendingConstraint>,
    /// Active unilateral (bridle) distance constraints.
    pub unilateral_constraints: Vec<UnilateralDistanceConstraint>,
    /// Canopy panels (triangles) for aerodynamics.
    pub canopy_panels: Vec<CanopyPanel>,
    /// External torques applied to each segment (world frame).
    pub torques: Vec<DVec3>,
    /// External forces applied to each particle.
    pub forces: Vec<DVec3>,
    /// Accumulated physics events since the last step.
    pub events: Vec<Event>,
    /// Cached graph coloring for parallel constraint solving.
    pub coloring: Option<crate::solver::SolverColoring>,
}

/// Helper to compute the dihedral angle between two triangles sharing an edge (p1, p2).
pub fn compute_dihedral_angle(p1: DVec3, p2: DVec3, p3: DVec3, p4: DVec3) -> f64 {
    let e = p2 - p1;
    let e_len = e.length();
    if e_len < 1e-12 {
        return 0.0;
    }
    let e_unit = e / e_len;

    let v1 = p3 - p1;
    let v2 = p4 - p1;

    let n1_raw = e.cross(v1);
    let n2_raw = v2.cross(e);

    let n1_len = n1_raw.length();
    let n2_len = n2_raw.length();

    if n1_len < 1e-12 || n2_len < 1e-12 {
        return 0.0;
    }

    let n1 = n1_raw / n1_len;
    let n2 = n2_raw / n2_len;

    let cos_theta = n1.dot(n2).clamp(-1.0, 1.0);
    let sin_theta = n1.cross(n2).dot(e_unit);

    sin_theta.atan2(cos_theta)
}

impl World {
    /// Creates a new, empty simulation world.
    pub fn new() -> Self {
        Self {
            time: 0.0,
            cfg: Config::default(),
            particles: ParticleSet::new(),
            orientations: OrientationSet::new(),
            distance_constraints: Vec::new(),
            bending_constraints: Vec::new(),
            stretch_shear_constraints: Vec::new(),
            bend_twist_constraints: Vec::new(),
            dihedral_bending_constraints: Vec::new(),
            unilateral_constraints: Vec::new(),
            canopy_panels: Vec::new(),
            torques: Vec::new(),
            forces: Vec::new(),
            events: Vec::new(),
            coloring: None,
        }
    }

    /// Adds a segment to the simulation, returning its index.
    pub fn add_segment(&mut self, quat: glam::DQuat, inv_inertia: DVec3) -> usize {
        self.torques.push(DVec3::ZERO);
        self.orientations.add_segment(quat, inv_inertia)
    }

    /// Adds a particle to the simulation, returning its index.
    pub fn add_particle(&mut self, pos: DVec3, mass: f64) -> usize {
        self.forces.push(DVec3::ZERO);
        self.particles.add_particle(pos, mass)
    }

    /// Advances the simulation by a time step `dt` using the XPBD solver.
    pub fn step(&mut self, dt: f64) {
        solver::step_simulation(self, dt);
        self.time += dt;
    }

    /// Computes the total mechanical energy of the system (translational + rotational kinetic, gravitational, and elastic).
    pub fn compute_total_energy(&self) -> f64 {
        let mut kinetic_trans = 0.0;
        let mut potential_grav = 0.0;

        for i in 0..self.particles.len() {
            let w = self.particles.inv_mass[i];
            if w > 0.0 {
                let m = 1.0 / w;
                kinetic_trans += 0.5 * m * self.particles.vel[i].length_squared();
                potential_grav -= m * self.cfg.gravity.dot(self.particles.pos[i]);
            }
        }

        let mut kinetic_rot = 0.0;
        for i in 0..self.orientations.len() {
            let inv_i = self.orientations.inv_inertia[i];
            let omega = self.orientations.omega[i];
            // Kinetic energy of rotated segment: E_rot = 0.5 * (I_x * omega_x^2 + I_y * omega_y^2 + I_z * omega_z^2)
            if inv_i.x > 0.0 {
                kinetic_rot += 0.5 * omega.x.powi(2) / inv_i.x;
            }
            if inv_i.y > 0.0 {
                kinetic_rot += 0.5 * omega.y.powi(2) / inv_i.y;
            }
            if inv_i.z > 0.0 {
                kinetic_rot += 0.5 * omega.z.powi(2) / inv_i.z;
            }
        }

        let mut elastic = 0.0;
        for c in &self.distance_constraints {
            let p1 = self.particles.pos[c.p1];
            let p2 = self.particles.pos[c.p2];
            let dist = (p1 - p2).length();
            let delta = dist - c.rest_length;
            let comp = c.compliance.max(1e-12);
            elastic += 0.5 * delta * delta / comp;
        }

        for b in &self.bending_constraints {
            let p1 = self.particles.pos[b.p1];
            let p2 = self.particles.pos[b.p2];
            let p3 = self.particles.pos[b.p3];
            let delta = (p1 - 2.0 * p2 + p3) - b.rest_value;
            let comp = b.compliance.max(1e-12);
            elastic += 0.5 * delta.length_squared() / comp;
        }

        // Elastic energy of Cosserat rod stretch-shear constraints
        for c in &self.stretch_shear_constraints {
            let p1 = self.particles.pos[c.p1];
            let p2 = self.particles.pos[c.p2];
            let q = self.orientations.quat[c.q_index];

            let d1 = q * DVec3::X;
            let d2 = q * DVec3::Y;
            let d3 = q * DVec3::Z;

            let diff = (p2 - p1) / c.rest_length;
            let c1 = diff.dot(d1);
            let c2 = diff.dot(d2);
            let c3 = diff.dot(d3) - 1.0;

            let comp = c.compliance;
            elastic += 0.5 * c1 * c1 / comp.x.max(1e-12);
            elastic += 0.5 * c2 * c2 / comp.y.max(1e-12);
            elastic += 0.5 * c3 * c3 / comp.z.max(1e-12);
        }

        // Elastic energy of Cosserat rod bend-twist constraints
        let pressure_stiffness_factor = 1.0 + self.cfg.k_pressure * self.cfg.bladder_pressure;
        for b in &self.bend_twist_constraints {
            if b.state == ConstraintState::Broken {
                continue;
            }

            let q1 = self.orientations.quat[b.q1_index];
            let q2 = self.orientations.quat[b.q2_index];

            // Local relative rotation vector: vec(conj(q1) * q2)
            let relative = q1.conjugate() * q2;
            let val = relative.xyz();

            // Note that Darboux vector has scaling term: let's match the constraint C
            // For energy we compute delta = C = (relative_vec - rest_value)
            let delta = val - b.rest_value;

            // Compliance mapping (pressure dependent for LE, folded factor if luffed)
            let mut comp = b.compliance;
            if b.is_inflatable {
                comp.x /= pressure_stiffness_factor;
                comp.y /= pressure_stiffness_factor;
            }

            if b.state == ConstraintState::Folded {
                // soft hinge: compliance multiplied by 1000
                comp.x *= 1000.0;
                comp.y *= 1000.0;
            }

            elastic += 0.5 * delta.x.powi(2) / comp.x.max(1e-12);
            elastic += 0.5 * delta.y.powi(2) / comp.y.max(1e-12);
            elastic += 0.5 * delta.z.powi(2) / comp.z.max(1e-12);
        }

        for b in &self.dihedral_bending_constraints {
            let p1 = self.particles.pos[b.p1];
            let p2 = self.particles.pos[b.p2];
            let p3 = self.particles.pos[b.p3];
            let p4 = self.particles.pos[b.p4];
            let angle = compute_dihedral_angle(p1, p2, p3, p4);
            let delta = angle - b.rest_angle;
            let comp = b.compliance.max(1e-12);
            elastic += 0.5 * delta * delta / comp;
        }

        for b in &self.unilateral_constraints {
            let p1 = self.particles.pos[b.p1];
            let p2 = self.particles.pos[b.p2];
            let dist = (p1 - p2).length();
            if dist > b.rest_length {
                let delta = dist - b.rest_length;
                let comp = b.compliance.max(1e-12);
                elastic += 0.5 * delta * delta / comp;
            }
        }

        kinetic_trans + kinetic_rot + potential_grav + elastic
    }
}

impl Default for World {
    fn default() -> Self {
        Self::new()
    }
}
