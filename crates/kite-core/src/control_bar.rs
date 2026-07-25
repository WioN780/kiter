//! Milestone 10c: dynamic (non-kinematic) control bar (masterplan §5.5).
//!
//! The bar is a genuinely rigid accessory, so it is modeled with Rapier's
//! *real* dynamics (`RigidBodySet` + a spherical joint pinning the bar
//! center to the pilot's hands), the one place Rapier dynamics is allowed.
//! Coupling to the XPBD world happens **only at the two anchor particles**
//! (masterplan §5.4/§5.5 boundary-exchange pattern), once per substep:
//!
//! - *pre-substep*: the bar tip positions/velocities are written into the
//!   anchor particles (which are kinematic on the XPBD side, `inv_mass = 0`);
//! - *post-substep*: the net line force on each anchor is recovered from the
//!   accumulated XPBD Lagrange multipliers (F = λ/h² along the constraint
//!   direction) of unilateral (line) constraints incident to the anchor, and
//!   applied to the bar at the tip; then the Rapier world advances by `h`.
//!
//! Rapier is **never** used for rods, cloth, or bridle lines themselves.
//!
//! Stability: this is an *explicit* staggered coupling. A slack line that
//! snaps taut against a fast-moving tip resolves its whole position violation
//! in one substep, and F = λ/h² feeds that back as an impulsive kick. That
//! can positively feed back and diverge if the line-side inertia rivals the
//! bar's. Keep the bar heavy relative to per-substep line impulses (true for
//! real kite rigs: aero damping keeps line tension smooth).
//! ponytail: explicit exchange; upgrade path is an implicit tip/line solve if
//! snap loads ever show up in real scenarios.

use glam::DVec3;
use rapier3d_f64::prelude::*;

fn to_vec(v: DVec3) -> Vector<Real> {
    vector![v.x, v.y, v.z]
}

fn to_dvec(v: Vector<Real>) -> DVec3 {
    DVec3::new(v.x, v.y, v.z)
}

/// A rigid control bar held at its center by a spherical joint (free to
/// rotate, translation locked to the pilot anchor), with kite lines attached
/// at its two tips.
pub struct ControlBar {
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    islands: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    ccd: CCDSolver,
    pipeline: PhysicsPipeline,
    params: IntegrationParameters,
    bar_handle: RigidBodyHandle,
    /// Bar tip offsets in bar-local coordinates (±half_span along local X).
    half_span: f64,
    /// XPBD anchor particle driven by the -X bar tip.
    pub left_anchor: usize,
    /// XPBD anchor particle driven by the +X bar tip.
    pub right_anchor: usize,
}

impl ControlBar {
    /// Creates a bar of `mass` kg and total span `2·half_span`, centered at
    /// `pilot_pos`, coupled to the two given XPBD particles (which must be
    /// kinematic: `inv_mass = 0`).
    pub fn new(
        pilot_pos: DVec3,
        half_span: f64,
        mass: f64,
        left_anchor: usize,
        right_anchor: usize,
    ) -> Self {
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let mut impulse_joints = ImpulseJointSet::new();

        let pilot = bodies.insert(RigidBodyBuilder::fixed().translation(to_vec(pilot_pos)));

        let bar = bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(to_vec(pilot_pos))
                .can_sleep(false),
        );
        // Thin capsule along local X carries the bar's mass properties.
        colliders.insert_with_parent(
            ColliderBuilder::capsule_x(half_span, 0.015)
                .mass(mass)
                // The bar never participates in kite collision detection.
                .collision_groups(InteractionGroups::none()),
            bar,
            &mut bodies,
        );

        let joint = SphericalJointBuilder::new()
            .local_anchor1(point![0.0, 0.0, 0.0])
            .local_anchor2(point![0.0, 0.0, 0.0]);
        impulse_joints.insert(pilot, bar, joint, true);

        Self {
            bodies,
            colliders,
            impulse_joints,
            multibody_joints: MultibodyJointSet::new(),
            islands: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            ccd: CCDSolver::new(),
            pipeline: PhysicsPipeline::new(),
            params: IntegrationParameters::default(),
            bar_handle: bar,
            half_span,
            left_anchor,
            right_anchor,
        }
    }

    fn tip_local(&self, left: bool) -> Point<Real> {
        let x = if left {
            -self.half_span
        } else {
            self.half_span
        };
        point![x, 0.0, 0.0]
    }

    /// World-space position of a bar tip.
    pub fn tip_position(&self, left: bool) -> DVec3 {
        let body = &self.bodies[self.bar_handle];
        let p = body.position() * self.tip_local(left);
        DVec3::new(p.x, p.y, p.z)
    }

    /// World-space velocity of a bar tip.
    pub fn tip_velocity(&self, left: bool) -> DVec3 {
        let body = &self.bodies[self.bar_handle];
        let p = body.position() * self.tip_local(left);
        to_dvec(body.velocity_at_point(&p))
    }

    /// Bar orientation as a glam quaternion (for logging/inspection).
    pub fn orientation(&self) -> glam::DQuat {
        let q = self.bodies[self.bar_handle].position().rotation;
        glam::DQuat::from_xyzw(q.i, q.j, q.k, q.w)
    }

    /// Writes bar tip states into the XPBD anchor particles. Called at the
    /// top of every substep.
    pub(crate) fn sync_anchors_to_world(&self, world: &mut crate::world::World) {
        for (left, anchor) in [(true, self.left_anchor), (false, self.right_anchor)] {
            let pos = self.tip_position(left);
            let vel = self.tip_velocity(left);
            world.particles.pos[anchor] = pos;
            world.particles.prev_pos[anchor] = pos;
            world.particles.pred_pos[anchor] = pos;
            world.particles.vel[anchor] = vel;
        }
    }

    /// Recovers the net line force on each anchor from the XPBD multipliers
    /// (F = λ/h² along the line), applies it to the bar tips, and advances
    /// the Rapier world by `h`. Called at the end of every substep.
    pub(crate) fn apply_line_forces_and_step(
        &mut self,
        world: &crate::world::World,
        h: f64,
        gravity: DVec3,
    ) {
        let h2 = h * h;
        let mut force = [DVec3::ZERO, DVec3::ZERO];

        for c in &world.unilateral_constraints {
            for (slot, anchor) in [(0usize, self.left_anchor), (1, self.right_anchor)] {
                let (this, other) = if c.p1 == anchor {
                    (c.p1, c.p2)
                } else if c.p2 == anchor {
                    (c.p2, c.p1)
                } else {
                    continue;
                };
                let diff = world.particles.pred_pos[this] - world.particles.pred_pos[other];
                let dist = diff.length();
                if dist < 1e-12 {
                    continue;
                }
                // λ ≤ 0 for a taut line: force pulls the anchor toward the
                // other end.
                force[slot] += (c.lambda / h2) * (diff / dist);
            }
        }

        let tips = [self.tip_local(true), self.tip_local(false)];
        let body = &mut self.bodies[self.bar_handle];
        // add_force_at_point accumulates into BOTH user_force and user_torque;
        // rapier clears neither on step, and reset_forces only clears the force
        // half. Without reset_torques every substep's line torque piles up
        // forever and the bar spins to hundreds of rad/s within a minute.
        body.reset_forces(true);
        body.reset_torques(true);
        for (slot, tip) in tips.iter().enumerate() {
            if force[slot] != DVec3::ZERO {
                let p = body.position() * tip;
                body.add_force_at_point(to_vec(force[slot]), p, true);
            }
        }

        self.params.dt = h;
        self.pipeline.step(
            &to_vec(gravity),
            &self.params,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd,
            None,
            &(),
            &(),
        );
    }
}
