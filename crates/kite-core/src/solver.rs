use crate::world::{World, Event};
use crate::constraints::ConstraintState;
use glam::{DVec3, DQuat};
use rayon::prelude::*;

/// Helper to get the scaled rotation vector (axis * angle) from a unit quaternion.
fn scaled_axis_from_quat(q: DQuat) -> DVec3 {
    let xyz = q.xyz();
    let len = xyz.length();
    if len < 1e-12 {
        DVec3::ZERO
    } else {
        let theta = 2.0 * len.atan2(q.w);
        let theta = if theta > std::f64::consts::PI {
            theta - 2.0 * std::f64::consts::PI
        } else if theta < -std::f64::consts::PI {
            theta + 2.0 * std::f64::consts::PI
        } else {
            theta
        };
        (xyz / len) * theta
    }
}

/// Executes a single full simulation step of duration `dt`, running the substep loop.
pub fn step_simulation(world: &mut World, dt: f64) {
    if world.cfg.substeps == 0 {
        return;
    }
    let h = dt / world.cfg.substeps as f64;

    for _ in 0..world.cfg.substeps {
        // 1. Apply external forces (gravity + point forces) to update particle velocities
        for i in 0..world.particles.len() {
            if world.particles.inv_mass[i] > 0.0 {
                let mut ext_accel = world.cfg.gravity;
                if i < world.forces.len() {
                    ext_accel += world.forces[i] * world.particles.inv_mass[i];
                }
                world.particles.vel[i] += h * ext_accel;
            }
        }

        // 1b. Apply external torques to update segment angular velocities (local frame)
        for i in 0..world.orientations.len() {
            let w_q = world.orientations.inv_inertia[i];
            if w_q.length_squared() > 0.0 && i < world.torques.len() {
                let q = world.orientations.quat[i];
                let torque_world = world.torques[i];
                let torque_local = q.conjugate() * torque_world;
                world.orientations.omega[i] += h * w_q * torque_local;
            }
        }

        // 1c. Apply line cross-flow drag to particle velocities
        apply_line_drag(world, h);

        // 1d. Apply canopy aerodynamics to particle velocities
        crate::aero::panel_method::apply_canopy_aerodynamics(world, h);

        // 1e. Apply spar cylinder drag to particle velocities
        crate::aero::spar_drag::apply_spar_drag(world, h);

        // 2. Predict positions (x_pred = x + v*h)
        for i in 0..world.particles.len() {
            world.particles.prev_pos[i] = world.particles.pos[i];
            world.particles.pred_pos[i] = world.particles.pos[i] + world.particles.vel[i] * h;
        }

        // 3. Predict orientations (q_pred = q * exp(0.5 * omega * h))
        for i in 0..world.orientations.len() {
            world.orientations.prev_quat[i] = world.orientations.quat[i];
            let rot_delta = DQuat::from_scaled_axis(world.orientations.omega[i] * h);
            world.orientations.quat[i] = (world.orientations.quat[i] * rot_delta).normalize();
        }

        // 4. Reset Lagrange multipliers (lambda = 0)
        for c in &mut world.distance_constraints {
            c.lambda = 0.0;
        }
        for b in &mut world.bending_constraints {
            b.lambda = DVec3::ZERO;
        }
        for c in &mut world.stretch_shear_constraints {
            c.lambda = DVec3::ZERO;
        }
        for b in &mut world.bend_twist_constraints {
            b.lambda = DVec3::ZERO;
        }
        for b in &mut world.dihedral_bending_constraints {
            b.lambda = 0.0;
        }
        for b in &mut world.unilateral_constraints {
            b.lambda = 0.0;
        }

        // 4.5 Detect self-collisions on predicted positions
        let mut self_contacts = if world.cfg.self_collision_enabled {
            crate::collision::detect_self_collisions(world)
        } else {
            Vec::new()
        };

        // Temporarily extract coloring from world to satisfy the borrow checker
        let mut coloring_opt = world.coloring.take();
        #[cfg(feature = "reference-mode")]
        let use_parallel = false;
        #[cfg(not(feature = "reference-mode"))]
        let use_parallel = world.cfg.parallel_solve;

        if use_parallel && coloring_opt.is_none() {
            coloring_opt = Some(SolverColoring::rebuild(world));
        }

        // 5. Solve constraints iteratively (Gauss-Seidel sweeps)
        for _ in 0..world.cfg.iterations_per_substep {
            if use_parallel {
                let coloring = coloring_opt.as_ref().unwrap();
                solve_distance_constraints_parallel(world, h, &coloring.distance_colors);
                solve_bending_constraints_parallel(world, h, &coloring.bending_colors);
                solve_stretch_shear_constraints_parallel(world, h, &coloring.stretch_shear_colors);
                solve_bend_twist_constraints_parallel(world, h, &coloring.bend_twist_colors);
                solve_dihedral_bending_constraints_parallel(world, h, &coloring.dihedral_bending_colors);
                solve_unilateral_constraints_parallel(world, h, &coloring.unilateral_colors);
            } else {
                solve_distance_constraints(world, h);
                solve_bending_constraints(world, h);
                solve_stretch_shear_constraints(world, h);
                solve_bend_twist_constraints(world, h);
                solve_dihedral_bending_constraints(world, h);
                solve_unilateral_constraints(world, h);
            }

            if world.cfg.self_collision_enabled {
                crate::collision::solve_contact_constraints(world, &mut self_contacts);
            }
            if world.cfg.ground_collision_enabled {
                crate::collision::solve_ground_collision(world);
            }
        }

        // Restore coloring to world
        world.coloring = coloring_opt;

        // 6. Update velocities: v = (x_pred - x_prev) / h
        // and apply velocity damping
        let damping_factor = (-world.cfg.damping * h).exp();
        for i in 0..world.particles.len() {
            if world.particles.inv_mass[i] > 0.0 {
                world.particles.vel[i] = (world.particles.pred_pos[i] - world.particles.prev_pos[i]) / h;
                world.particles.vel[i] *= damping_factor;
            } else {
                world.particles.vel[i] = DVec3::ZERO;
            }
        }

        // 7. Update angular velocities: omega = scaled_axis(conj(q_prev) * q) / h
        for i in 0..world.orientations.len() {
            let inv_i = world.orientations.inv_inertia[i];
            if inv_i.length_squared() > 0.0 {
                let dq = world.orientations.prev_quat[i].conjugate() * world.orientations.quat[i];
                world.orientations.omega[i] = scaled_axis_from_quat(dq) / h;
                world.orientations.omega[i] *= damping_factor;
            } else {
                world.orientations.omega[i] = DVec3::ZERO;
            }
        }

        // 8. Commit positions (x = x_pred)
        for i in 0..world.particles.len() {
            world.particles.pos[i] = world.particles.pred_pos[i];
        }

        // 8b. Process yield and breaking constraints at the end of the substep
        process_yield_and_breaking(world, h);
    }
}

fn solve_distance_constraints(world: &mut World, h: f64) {
    let h2 = h * h;
    for c in &mut world.distance_constraints {
        let w1 = world.particles.inv_mass[c.p1];
        let w2 = world.particles.inv_mass[c.p2];
        let w_sum = w1 + w2;
        if w_sum <= 0.0 {
            continue;
        }

        let p1 = world.particles.pred_pos[c.p1];
        let p2 = world.particles.pred_pos[c.p2];

        let diff = p1 - p2;
        let dist = diff.length();
        if dist < 1e-12 {
            continue;
        }

        let dir = diff / dist;
        let constraint_val = dist - c.rest_length;

        let alpha_tilde = c.compliance / h2;
        let denom = w_sum + alpha_tilde;
        if denom < 1e-12 {
            continue;
        }

        let delta_lambda = (-constraint_val - alpha_tilde * c.lambda) / denom;
        c.lambda += delta_lambda;

        if w1 > 0.0 {
            world.particles.pred_pos[c.p1] += w1 * delta_lambda * dir;
        }
        if w2 > 0.0 {
            world.particles.pred_pos[c.p2] -= w2 * delta_lambda * dir;
        }
    }
}

fn solve_bending_constraints(world: &mut World, h: f64) {
    let h2 = h * h;
    for b in &mut world.bending_constraints {
        let w1 = world.particles.inv_mass[b.p1];
        let w2 = world.particles.inv_mass[b.p2];
        let w3 = world.particles.inv_mass[b.p3];

        let denom_w = w1 + 4.0 * w2 + w3;
        if denom_w <= 0.0 {
            continue;
        }

        let p1 = world.particles.pred_pos[b.p1];
        let p2 = world.particles.pred_pos[b.p2];
        let p3 = world.particles.pred_pos[b.p3];

        let constraint_val = (p1 - 2.0 * p2 + p3) - b.rest_value;

        let alpha_tilde = b.compliance / h2;
        let denom = denom_w + alpha_tilde;
        if denom < 1e-12 {
            continue;
        }

        let delta_lambda = (-constraint_val - alpha_tilde * b.lambda) / denom;
        b.lambda += delta_lambda;

        if w1 > 0.0 {
            world.particles.pred_pos[b.p1] += w1 * delta_lambda;
        }
        if w2 > 0.0 {
            world.particles.pred_pos[b.p2] -= 2.0 * w2 * delta_lambda;
        }
        if w3 > 0.0 {
            world.particles.pred_pos[b.p3] += w3 * delta_lambda;
        }
    }
}

// UNVERIFIED: Standard XPBD stretch-shear constraint projections derived from first principles.
fn solve_stretch_shear_constraints(world: &mut World, h: f64) {
    let h2 = h * h;
    for c in &mut world.stretch_shear_constraints {
        let w1 = world.particles.inv_mass[c.p1];
        let w2 = world.particles.inv_mass[c.p2];
        let w_q = world.orientations.inv_inertia[c.q_index];

        if w1 + w2 + w_q.length_squared() <= 0.0 {
            continue;
        }

        let l0 = c.rest_length;
        let w_pos = (w1 + w2) / (l0 * l0);

        // Solve component by component (shear_x, shear_y, stretch_z)
        for k in 0..3 {
            let p1 = world.particles.pred_pos[c.p1];
            let p2 = world.particles.pred_pos[c.p2];
            let q = world.orientations.quat[c.q_index];

            // Local directors in world frame
            let d_k = match k {
                0 => q * DVec3::X,
                1 => q * DVec3::Y,
                _ => q * DVec3::Z,
            };

            let u = (p2 - p1) / l0;
            let constraint_val = u.dot(d_k) - if k == 2 { 1.0 } else { 0.0 };

            // Rotate segment vector into local frame
            let u_local = q.conjugate() * u;

            // Jacobian with respect to local rotation
            let j_rot_local = match k {
                0 => DVec3::new(0.0, -u_local.z, u_local.y),
                1 => DVec3::new(u_local.z, 0.0, -u_local.x),
                _ => DVec3::new(-u_local.y, u_local.x, 0.0),
            };

            // Rotational inverse mass term
            let w_rot = (j_rot_local.x.powi(2) * w_q.x)
                + (j_rot_local.y.powi(2) * w_q.y)
                + (j_rot_local.z.powi(2) * w_q.z);

            let alpha_tilde = c.compliance[k] / h2;
            let denom = w_pos + w_rot + alpha_tilde;
            if denom < 1e-12 {
                continue;
            }

            let delta_lambda = (-constraint_val - alpha_tilde * c.lambda[k]) / denom;
            c.lambda[k] += delta_lambda;

            // Apply position corrections
            if w1 > 0.0 {
                world.particles.pred_pos[c.p1] -= (w1 / l0) * delta_lambda * d_k;
            }
            if w2 > 0.0 {
                world.particles.pred_pos[c.p2] += (w2 / l0) * delta_lambda * d_k;
            }

            // Apply orientation correction (in local frame)
            let delta_theta_local = DVec3::new(
                j_rot_local.x * w_q.x * delta_lambda,
                j_rot_local.y * w_q.y * delta_lambda,
                j_rot_local.z * w_q.z * delta_lambda,
            );
            world.orientations.quat[c.q_index] =
                (world.orientations.quat[c.q_index] * DQuat::from_scaled_axis(delta_theta_local)).normalize();
        }
    }
}

// UNVERIFIED: Standard XPBD bend-twist constraint projections derived from first principles.
fn solve_bend_twist_constraints(world: &mut World, h: f64) {
    let h2 = h * h;
    let pressure_stiffness_factor = 1.0 + world.cfg.k_pressure * world.cfg.bladder_pressure;

    for b in &mut world.bend_twist_constraints {
        if b.state == ConstraintState::Broken {
            continue;
        }

        let w_q1 = world.orientations.inv_inertia[b.q1_index];
        let w_q2 = world.orientations.inv_inertia[b.q2_index];

        if w_q1.length_squared() + w_q2.length_squared() <= 0.0 {
            continue;
        }

        // Adjust compliance for pressure-dependence
        let mut comp = b.compliance;
        if b.is_inflatable {
            comp.x /= pressure_stiffness_factor;
            comp.y /= pressure_stiffness_factor;
        }

        // Adjust compliance for folded joints
        if b.state == ConstraintState::Folded {
            comp.x *= 1000.0;
            comp.y *= 1000.0;
        }

        // Solve component by component (bend_x, bend_y, twist_z)
        for k in 0..3 {
            let q1 = world.orientations.quat[b.q1_index];
            let q2 = world.orientations.quat[b.q2_index];

            let r = q1.conjugate() * q2;
            let r_vec = r.xyz();
            let r_w = r.w;

            let constraint_val = r_vec[k] - b.rest_value[k];

            // Define e_k unit vector
            let e_k = match k {
                0 => DVec3::X,
                1 => DVec3::Y,
                _ => DVec3::Z,
            };

            // Jacobian with respect to local rotation of segment 1 and 2
            let j0 = -0.5 * (r_w * e_k - e_k.cross(r_vec));
            let j1 = 0.5 * (r_w * e_k + e_k.cross(r_vec));

            let w_rot0 = (j0.x.powi(2) * w_q1.x) + (j0.y.powi(2) * w_q1.y) + (j0.z.powi(2) * w_q1.z);
            let w_rot1 = (j1.x.powi(2) * w_q2.x) + (j1.y.powi(2) * w_q2.y) + (j1.z.powi(2) * w_q2.z);

            let alpha_tilde = comp[k] / h2;
            let denom = w_rot0 + w_rot1 + alpha_tilde;
            if denom < 1e-12 {
                continue;
            }

            let delta_lambda = (-constraint_val - alpha_tilde * b.lambda[k]) / denom;
            b.lambda[k] += delta_lambda;

            // Apply corrections to orientation quaternions
            let delta_theta_local_1 = DVec3::new(
                j0.x * w_q1.x * delta_lambda,
                j0.y * w_q1.y * delta_lambda,
                j0.z * w_q1.z * delta_lambda,
            );
            world.orientations.quat[b.q1_index] =
                (world.orientations.quat[b.q1_index] * DQuat::from_scaled_axis(delta_theta_local_1)).normalize();

            let delta_theta_local_2 = DVec3::new(
                j1.x * w_q2.x * delta_lambda,
                j1.y * w_q2.y * delta_lambda,
                j1.z * w_q2.z * delta_lambda,
            );
            world.orientations.quat[b.q2_index] =
                (world.orientations.quat[b.q2_index] * DQuat::from_scaled_axis(delta_theta_local_2)).normalize();
        }
    }
}

/// Post-substep yield checks.
fn process_yield_and_breaking(world: &mut World, h: f64) {
    let h2 = h * h;
    for idx in 0..world.bend_twist_constraints.len() {
        let b = &mut world.bend_twist_constraints[idx];
        if b.state != ConstraintState::Active {
            continue;
        }

        // Monitor bending moment: lambda.length() / h^2
        let moment = b.lambda.length() / h2;

        if moment > b.yield_threshold {
            if b.is_inflatable {
                b.state = ConstraintState::Folded;
                world.events.push(Event::LeadingEdgeFolded { joint_index: idx });
            } else {
                b.state = ConstraintState::Broken;
                world.events.push(Event::SparBroken { joint_index: idx });
            }
        }
    }
}

// UNVERIFIED: Standard XPBD dihedral bending constraint projections derived from first principles.
fn solve_dihedral_bending_constraints(world: &mut World, h: f64) {
    let h2 = h * h;
    for b in &mut world.dihedral_bending_constraints {
        let w1 = world.particles.inv_mass[b.p1];
        let w2 = world.particles.inv_mass[b.p2];
        let w3 = world.particles.inv_mass[b.p3];
        let w4 = world.particles.inv_mass[b.p4];

        let w_sum_mass = w1 + w2 + w3 + w4;
        if w_sum_mass <= 0.0 {
            continue;
        }

        let p1 = world.particles.pred_pos[b.p1];
        let p2 = world.particles.pred_pos[b.p2];
        let p3 = world.particles.pred_pos[b.p3];
        let p4 = world.particles.pred_pos[b.p4];

        // Compute current dihedral angle
        let angle = crate::world::compute_dihedral_angle(p1, p2, p3, p4);

        let mut constraint_val = angle - b.rest_angle;
        if constraint_val > std::f64::consts::PI {
            constraint_val -= 2.0 * std::f64::consts::PI;
        } else if constraint_val < -std::f64::consts::PI {
            constraint_val += 2.0 * std::f64::consts::PI;
        }

        let e = p2 - p1;
        let e_len = e.length();
        if e_len < 1e-12 {
            continue;
        }

        let e1 = e.cross(p3 - p1);
        let e2 = (p4 - p1).cross(e);

        let e1_len = e1.length();
        let e2_len = e2.length();

        if e1_len < 1e-12 || e2_len < 1e-12 {
            continue;
        }

        let n1 = e1 / e1_len;
        let n2 = e2 / e2_len;

        let h1 = e1_len / e_len;
        let h2_height = e2_len / e_len;

        // Gradients
        let q3 = -n1 / h1;
        let q4 = -n2 / h2_height;

        let d1 = (p3 - p1).dot(e) / (e_len * e_len);
        let d2 = (p4 - p1).dot(e) / (e_len * e_len);

        let q1 = (d1 - 1.0) * q3 + (d2 - 1.0) * q4;
        let q2 = -d1 * q3 - d2 * q4;

        let w_sum = w1 * q1.length_squared()
            + w2 * q2.length_squared()
            + w3 * q3.length_squared()
            + w4 * q4.length_squared();

        let alpha_tilde = b.compliance / h2;
        let denom = w_sum + alpha_tilde;
        if denom < 1e-12 {
            continue;
        }

        let delta_lambda = (-constraint_val - alpha_tilde * b.lambda) / denom;
        b.lambda += delta_lambda;

        if w1 > 0.0 {
            world.particles.pred_pos[b.p1] += w1 * delta_lambda * q1;
        }
        if w2 > 0.0 {
            world.particles.pred_pos[b.p2] += w2 * delta_lambda * q2;
        }
        if w3 > 0.0 {
            world.particles.pred_pos[b.p3] += w3 * delta_lambda * q3;
        }
        if w4 > 0.0 {
            world.particles.pred_pos[b.p4] += w4 * delta_lambda * q4;
        }
    }
}

fn solve_unilateral_constraints(world: &mut World, h: f64) {
    let h2 = h * h;
    for b in &mut world.unilateral_constraints {
        let w1 = world.particles.inv_mass[b.p1];
        let w2 = world.particles.inv_mass[b.p2];
        let w_sum = w1 + w2;
        if w_sum <= 0.0 {
            continue;
        }

        let p1 = world.particles.pred_pos[b.p1];
        let p2 = world.particles.pred_pos[b.p2];

        let dist = (p1 - p2).length();
        let constraint_val = dist - b.rest_length;

        // If slack and lambda is already 0, it's inactive (no tension)
        if constraint_val <= 0.0 && b.lambda >= 0.0 {
            continue;
        }

        if dist < 1e-12 {
            continue;
        }
        let e = (p1 - p2) / dist; // normalized direction

        let alpha_tilde = b.compliance / h2;
        let denom = w_sum + alpha_tilde;
        if denom < 1e-12 {
            continue;
        }

        let delta_lambda = (-constraint_val - alpha_tilde * b.lambda) / denom;
        // In unilateral (tension-only), lambda represents tension (pulling force) which is negative.
        // So lambda must be <= 0.0.
        let new_lambda = (b.lambda + delta_lambda).min(0.0);
        let delta_lambda_clamped = new_lambda - b.lambda;
        b.lambda = new_lambda;

        if w1 > 0.0 {
            world.particles.pred_pos[b.p1] += w1 * delta_lambda_clamped * e;
        }
        if w2 > 0.0 {
            world.particles.pred_pos[b.p2] -= w2 * delta_lambda_clamped * e;
        }
    }
}

fn apply_line_drag(world: &mut World, h: f64) {
    let rho = 1.225; // standard air density kg/m^3
    let cd_line = 1.2; // cylinder drag coefficient

    for b in &world.unilateral_constraints {
        let w1 = world.particles.inv_mass[b.p1];
        let w2 = world.particles.inv_mass[b.p2];
        if w1 <= 0.0 && w2 <= 0.0 {
            continue;
        }

        let p1 = world.particles.pos[b.p1];
        let p2 = world.particles.pos[b.p2];
        let diff = p2 - p1;
        let len = diff.length();
        if len < 1e-6 {
            continue;
        }
        let u = diff / len; // unit tangent along line

        // Midpoint velocity and position
        let v1 = world.particles.vel[b.p1];
        let v2 = world.particles.vel[b.p2];
        let v_mid = 0.5 * (v1 + v2);
        let p_mid = 0.5 * (p1 + p2);

        // Relative wind at midpoint
        let wind_vel = crate::wind::wind_at(p_mid, world.time, &world.cfg.wind);
        let v_rel = wind_vel - v_mid;

        // Decompose into cross-flow component
        let v_axial = v_rel.dot(u) * u;
        let v_cross = v_rel - v_axial;
        let v_cross_mag = v_cross.length();

        if v_cross_mag < 1e-6 {
            continue;
        }

        // Cross-flow drag force: F = 0.5 * rho * cd * V_cross^2 * diameter * length
        // Vector points along v_cross direction
        let force_mag = 0.5 * rho * cd_line * v_cross_mag * v_cross_mag * b.diameter * len;
        let force_vector = force_mag * (v_cross / v_cross_mag);

        // Split force equally between the two particles
        let f_half = 0.5 * force_vector;

        if w1 > 0.0 {
            world.particles.vel[b.p1] += h * f_half * w1;
        }
        if w2 > 0.0 {
            world.particles.vel[b.p2] += h * f_half * w2;
        }
    }
}

// =========================================================================
// Parallel Solver & Graph-Coloring Implementations
// =========================================================================

struct SendPtr<T>(*mut T);

impl<T> Copy for SendPtr<T> {}

impl<T> Clone for SendPtr<T> {
    fn clone(&self) -> Self {
        *self
    }
}

unsafe impl<T> Send for SendPtr<T> {}
unsafe impl<T> Sync for SendPtr<T> {}

impl<T> SendPtr<T> {
    pub unsafe fn add(self, offset: usize) -> *mut T {
        self.0.add(offset)
    }
}

#[derive(Clone, Debug, Default)]
pub struct SolverColoring {
    pub distance_colors: Vec<Vec<usize>>,
    pub bending_colors: Vec<Vec<usize>>,
    pub stretch_shear_colors: Vec<Vec<usize>>,
    pub bend_twist_colors: Vec<Vec<usize>>,
    pub dihedral_bending_colors: Vec<Vec<usize>>,
    pub unilateral_colors: Vec<Vec<usize>>,
}

impl SolverColoring {
    pub fn rebuild(world: &World) -> Self {
        let n_particles = world.particles.len();
        let n_orientations = world.orientations.len();

        let distance_colors = color_distance_constraints(&world.distance_constraints, n_particles);
        let bending_colors = color_bending_constraints(&world.bending_constraints, n_particles);
        let stretch_shear_colors = color_stretch_shear_constraints(&world.stretch_shear_constraints, n_particles, n_orientations);
        let bend_twist_colors = color_bend_twist_constraints(&world.bend_twist_constraints, n_orientations);
        let dihedral_bending_colors = color_dihedral_bending_constraints(&world.dihedral_bending_constraints, n_particles);
        let unilateral_colors = color_unilateral_constraints(&world.unilateral_constraints, n_particles);

        Self {
            distance_colors,
            bending_colors,
            stretch_shear_colors,
            bend_twist_colors,
            dihedral_bending_colors,
            unilateral_colors,
        }
    }
}

fn color_distance_constraints(constraints: &[crate::constraints::DistanceConstraint], n_particles: usize) -> Vec<Vec<usize>> {
    let mut colors: Vec<Vec<usize>> = Vec::new();
    let mut color_used: Vec<Vec<bool>> = Vec::new();

    for (idx, c) in constraints.iter().enumerate() {
        let mut found_color = None;
        for (c_idx, used) in color_used.iter().enumerate() {
            if !used[c.p1] && !used[c.p2] {
                found_color = Some(c_idx);
                break;
            }
        }
        if let Some(c_idx) = found_color {
            colors[c_idx].push(idx);
            color_used[c_idx][c.p1] = true;
            color_used[c_idx][c.p2] = true;
        } else {
            let mut used = vec![false; n_particles];
            used[c.p1] = true;
            used[c.p2] = true;
            color_used.push(used);
            colors.push(vec![idx]);
        }
    }
    colors
}

fn color_bending_constraints(constraints: &[crate::constraints::BendingConstraint], n_particles: usize) -> Vec<Vec<usize>> {
    let mut colors: Vec<Vec<usize>> = Vec::new();
    let mut color_used: Vec<Vec<bool>> = Vec::new();

    for (idx, c) in constraints.iter().enumerate() {
        let mut found_color = None;
        for (c_idx, used) in color_used.iter().enumerate() {
            if !used[c.p1] && !used[c.p2] && !used[c.p3] {
                found_color = Some(c_idx);
                break;
            }
        }
        if let Some(c_idx) = found_color {
            colors[c_idx].push(idx);
            color_used[c_idx][c.p1] = true;
            color_used[c_idx][c.p2] = true;
            color_used[c_idx][c.p3] = true;
        } else {
            let mut used = vec![false; n_particles];
            used[c.p1] = true;
            used[c.p2] = true;
            used[c.p3] = true;
            color_used.push(used);
            colors.push(vec![idx]);
        }
    }
    colors
}

fn color_stretch_shear_constraints(constraints: &[crate::constraints::StretchShearConstraint], n_particles: usize, n_orientations: usize) -> Vec<Vec<usize>> {
    let mut colors: Vec<Vec<usize>> = Vec::new();
    let mut color_p_used: Vec<Vec<bool>> = Vec::new();
    let mut color_q_used: Vec<Vec<bool>> = Vec::new();

    for (idx, c) in constraints.iter().enumerate() {
        let mut found_color = None;
        for (c_idx, (p_used, q_used)) in color_p_used.iter().zip(color_q_used.iter()).enumerate() {
            if !p_used[c.p1] && !p_used[c.p2] && !q_used[c.q_index] {
                found_color = Some(c_idx);
                break;
            }
        }
        if let Some(c_idx) = found_color {
            colors[c_idx].push(idx);
            color_p_used[c_idx][c.p1] = true;
            color_p_used[c_idx][c.p2] = true;
            color_q_used[c_idx][c.q_index] = true;
        } else {
            let mut p_used = vec![false; n_particles];
            let mut q_used = vec![false; n_orientations];
            p_used[c.p1] = true;
            p_used[c.p2] = true;
            q_used[c.q_index] = true;
            color_p_used.push(p_used);
            color_q_used.push(q_used);
            colors.push(vec![idx]);
        }
    }
    colors
}

fn color_bend_twist_constraints(constraints: &[crate::constraints::BendTwistConstraint], n_orientations: usize) -> Vec<Vec<usize>> {
    let mut colors: Vec<Vec<usize>> = Vec::new();
    let mut color_used: Vec<Vec<bool>> = Vec::new();

    for (idx, b) in constraints.iter().enumerate() {
        let mut found_color = None;
        for (c_idx, used) in color_used.iter().enumerate() {
            if !used[b.q1_index] && !used[b.q2_index] {
                found_color = Some(c_idx);
                break;
            }
        }
        if let Some(c_idx) = found_color {
            colors[c_idx].push(idx);
            color_used[c_idx][b.q1_index] = true;
            color_used[c_idx][b.q2_index] = true;
        } else {
            let mut used = vec![false; n_orientations];
            used[b.q1_index] = true;
            used[b.q2_index] = true;
            color_used.push(used);
            colors.push(vec![idx]);
        }
    }
    colors
}

fn color_dihedral_bending_constraints(constraints: &[crate::constraints::DihedralBendingConstraint], n_particles: usize) -> Vec<Vec<usize>> {
    let mut colors: Vec<Vec<usize>> = Vec::new();
    let mut color_used: Vec<Vec<bool>> = Vec::new();

    for (idx, b) in constraints.iter().enumerate() {
        let mut found_color = None;
        for (c_idx, used) in color_used.iter().enumerate() {
            if !used[b.p1] && !used[b.p2] && !used[b.p3] && !used[b.p4] {
                found_color = Some(c_idx);
                break;
            }
        }
        if let Some(c_idx) = found_color {
            colors[c_idx].push(idx);
            color_used[c_idx][b.p1] = true;
            color_used[c_idx][b.p2] = true;
            color_used[c_idx][b.p3] = true;
            color_used[c_idx][b.p4] = true;
        } else {
            let mut used = vec![false; n_particles];
            used[b.p1] = true;
            used[b.p2] = true;
            used[b.p3] = true;
            used[b.p4] = true;
            color_used.push(used);
            colors.push(vec![idx]);
        }
    }
    colors
}

fn color_unilateral_constraints(constraints: &[crate::constraints::UnilateralDistanceConstraint], n_particles: usize) -> Vec<Vec<usize>> {
    let mut colors: Vec<Vec<usize>> = Vec::new();
    let mut color_used: Vec<Vec<bool>> = Vec::new();

    for (idx, c) in constraints.iter().enumerate() {
        let mut found_color = None;
        for (c_idx, used) in color_used.iter().enumerate() {
            if !used[c.p1] && !used[c.p2] {
                found_color = Some(c_idx);
                break;
            }
        }
        if let Some(c_idx) = found_color {
            colors[c_idx].push(idx);
            color_used[c_idx][c.p1] = true;
            color_used[c_idx][c.p2] = true;
        } else {
            let mut used = vec![false; n_particles];
            used[c.p1] = true;
            used[c.p2] = true;
            color_used.push(used);
            colors.push(vec![idx]);
        }
    }
    colors
}

pub fn solve_distance_constraints_parallel(world: &mut World, h: f64, colors: &[Vec<usize>]) {
    let h2 = h * h;
    let pred_pos_ptr = SendPtr(world.particles.pred_pos.as_mut_ptr());
    let inv_mass_ptr = SendPtr(world.particles.inv_mass.as_ptr() as *mut f64);
    let constraints_ptr = SendPtr(world.distance_constraints.as_mut_ptr());

    for color_class in colors {
        color_class.par_iter().for_each(|&idx| unsafe {
            let c = &mut *constraints_ptr.add(idx);
            let w1 = *inv_mass_ptr.add(c.p1);
            let w2 = *inv_mass_ptr.add(c.p2);
            let w_sum = w1 + w2;
            if w_sum <= 0.0 {
                return;
            }

            let p1 = *pred_pos_ptr.add(c.p1);
            let p2 = *pred_pos_ptr.add(c.p2);

            let diff = p1 - p2;
            let dist = diff.length();
            if dist < 1e-12 {
                return;
            }

            let dir = diff / dist;
            let constraint_val = dist - c.rest_length;

            let alpha_tilde = c.compliance / h2;
            let denom = w_sum + alpha_tilde;
            if denom < 1e-12 {
                return;
            }

            let delta_lambda = (-constraint_val - alpha_tilde * c.lambda) / denom;
            c.lambda += delta_lambda;

            if w1 > 0.0 {
                *pred_pos_ptr.add(c.p1) += w1 * delta_lambda * dir;
            }
            if w2 > 0.0 {
                *pred_pos_ptr.add(c.p2) -= w2 * delta_lambda * dir;
            }
        });
    }
}

pub fn solve_bending_constraints_parallel(world: &mut World, h: f64, colors: &[Vec<usize>]) {
    let h2 = h * h;
    let pred_pos_ptr = SendPtr(world.particles.pred_pos.as_mut_ptr());
    let inv_mass_ptr = SendPtr(world.particles.inv_mass.as_ptr() as *mut f64);
    let constraints_ptr = SendPtr(world.bending_constraints.as_mut_ptr());

    for color_class in colors {
        color_class.par_iter().for_each(|&idx| unsafe {
            let b = &mut *constraints_ptr.add(idx);
            let w1 = *inv_mass_ptr.add(b.p1);
            let w2 = *inv_mass_ptr.add(b.p2);
            let w3 = *inv_mass_ptr.add(b.p3);

            let denom_w = w1 + 4.0 * w2 + w3;
            if denom_w <= 0.0 {
                return;
            }

            let p1 = *pred_pos_ptr.add(b.p1);
            let p2 = *pred_pos_ptr.add(b.p2);
            let p3 = *pred_pos_ptr.add(b.p3);

            let constraint_val = (p1 - 2.0 * p2 + p3) - b.rest_value;

            let alpha_tilde = b.compliance / h2;
            let denom = denom_w + alpha_tilde;
            if denom < 1e-12 {
                return;
            }

            let delta_lambda = (-constraint_val - alpha_tilde * b.lambda) / denom;
            b.lambda += delta_lambda;

            if w1 > 0.0 {
                *pred_pos_ptr.add(b.p1) += w1 * delta_lambda;
            }
            if w2 > 0.0 {
                *pred_pos_ptr.add(b.p2) -= 2.0 * w2 * delta_lambda;
            }
            if w3 > 0.0 {
                *pred_pos_ptr.add(b.p3) += w3 * delta_lambda;
            }
        });
    }
}

pub fn solve_stretch_shear_constraints_parallel(world: &mut World, h: f64, colors: &[Vec<usize>]) {
    let h2 = h * h;
    let pred_pos_ptr = SendPtr(world.particles.pred_pos.as_mut_ptr());
    let inv_mass_ptr = SendPtr(world.particles.inv_mass.as_ptr() as *mut f64);
    let orientations_quat_ptr = SendPtr(world.orientations.quat.as_mut_ptr());
    let orientations_inv_inertia_ptr = SendPtr(world.orientations.inv_inertia.as_ptr() as *mut DVec3);
    let constraints_ptr = SendPtr(world.stretch_shear_constraints.as_mut_ptr());

    for color_class in colors {
        color_class.par_iter().for_each(|&idx| unsafe {
            let c = &mut *constraints_ptr.add(idx);
            let w1 = *inv_mass_ptr.add(c.p1);
            let w2 = *inv_mass_ptr.add(c.p2);
            let w_q = *orientations_inv_inertia_ptr.add(c.q_index);

            if w1 + w2 + w_q.length_squared() <= 0.0 {
                return;
            }

            let l0 = c.rest_length;
            let w_pos = (w1 + w2) / (l0 * l0);

            for k in 0..3 {
                let p1 = *pred_pos_ptr.add(c.p1);
                let p2 = *pred_pos_ptr.add(c.p2);
                let q = *orientations_quat_ptr.add(c.q_index);

                let d_k = match k {
                    0 => q * DVec3::X,
                    1 => q * DVec3::Y,
                    _ => q * DVec3::Z,
                };

                let u = (p2 - p1) / l0;
                let constraint_val = u.dot(d_k) - if k == 2 { 1.0 } else { 0.0 };

                let u_local = q.conjugate() * u;

                let j_rot_local = match k {
                    0 => DVec3::new(0.0, -u_local.z, u_local.y),
                    1 => DVec3::new(u_local.z, 0.0, -u_local.x),
                    _ => DVec3::new(-u_local.y, u_local.x, 0.0),
                };

                let w_rot = (j_rot_local.x.powi(2) * w_q.x)
                    + (j_rot_local.y.powi(2) * w_q.y)
                    + (j_rot_local.z.powi(2) * w_q.z);

                let alpha_tilde = c.compliance[k] / h2;
                let denom = w_pos + w_rot + alpha_tilde;
                if denom < 1e-12 {
                    return;
                }

                let delta_lambda = (-constraint_val - alpha_tilde * c.lambda[k]) / denom;
                c.lambda[k] += delta_lambda;

                if w1 > 0.0 {
                    *pred_pos_ptr.add(c.p1) -= (w1 / l0) * delta_lambda * d_k;
                }
                if w2 > 0.0 {
                    *pred_pos_ptr.add(c.p2) += (w2 / l0) * delta_lambda * d_k;
                }

                let delta_theta_local = DVec3::new(
                    j_rot_local.x * w_q.x * delta_lambda,
                    j_rot_local.y * w_q.y * delta_lambda,
                    j_rot_local.z * w_q.z * delta_lambda,
                );
                *orientations_quat_ptr.add(c.q_index) =
                    (*orientations_quat_ptr.add(c.q_index) * DQuat::from_scaled_axis(delta_theta_local)).normalize();
            }
        });
    }
}

pub fn solve_bend_twist_constraints_parallel(world: &mut World, h: f64, colors: &[Vec<usize>]) {
    let h2 = h * h;
    let pressure_stiffness_factor = 1.0 + world.cfg.k_pressure * world.cfg.bladder_pressure;
    let orientations_quat_ptr = SendPtr(world.orientations.quat.as_mut_ptr());
    let orientations_inv_inertia_ptr = SendPtr(world.orientations.inv_inertia.as_ptr() as *mut DVec3);
    let constraints_ptr = SendPtr(world.bend_twist_constraints.as_mut_ptr());

    for color_class in colors {
        color_class.par_iter().for_each(|&idx| unsafe {
            let b = &mut *constraints_ptr.add(idx);
            if b.state == ConstraintState::Broken {
                return;
            }

            let w_q1 = *orientations_inv_inertia_ptr.add(b.q1_index);
            let w_q2 = *orientations_inv_inertia_ptr.add(b.q2_index);

            if w_q1.length_squared() + w_q2.length_squared() <= 0.0 {
                return;
            }

            let mut comp = b.compliance;
            if b.is_inflatable {
                comp.x /= pressure_stiffness_factor;
                comp.y /= pressure_stiffness_factor;
            }

            if b.state == ConstraintState::Folded {
                comp.x *= 1000.0;
                comp.y *= 1000.0;
            }

            for k in 0..3 {
                let q1 = *orientations_quat_ptr.add(b.q1_index);
                let q2 = *orientations_quat_ptr.add(b.q2_index);

                let r = q1.conjugate() * q2;
                let r_vec = r.xyz();
                let r_w = r.w;

                let constraint_val = r_vec[k] - b.rest_value[k];

                let e_k = match k {
                    0 => DVec3::X,
                    1 => DVec3::Y,
                    _ => DVec3::Z,
                };

                let j0 = -0.5 * (r_w * e_k - e_k.cross(r_vec));
                let j1 = 0.5 * (r_w * e_k + e_k.cross(r_vec));

                let w_rot0 = (j0.x.powi(2) * w_q1.x) + (j0.y.powi(2) * w_q1.y) + (j0.z.powi(2) * w_q1.z);
                let w_rot1 = (j1.x.powi(2) * w_q2.x) + (j1.y.powi(2) * w_q2.y) + (j1.z.powi(2) * w_q2.z);

                let alpha_tilde = comp[k] / h2;
                let denom = w_rot0 + w_rot1 + alpha_tilde;
                if denom < 1e-12 {
                    return;
                }

                let delta_lambda = (-constraint_val - alpha_tilde * b.lambda[k]) / denom;
                b.lambda[k] += delta_lambda;

                let delta_theta_local_1 = DVec3::new(
                    j0.x * w_q1.x * delta_lambda,
                    j0.y * w_q1.y * delta_lambda,
                    j0.z * w_q1.z * delta_lambda,
                );
                *orientations_quat_ptr.add(b.q1_index) =
                    (*orientations_quat_ptr.add(b.q1_index) * DQuat::from_scaled_axis(delta_theta_local_1)).normalize();

                let delta_theta_local_2 = DVec3::new(
                    j1.x * w_q2.x * delta_lambda,
                    j1.y * w_q2.y * delta_lambda,
                    j1.z * w_q2.z * delta_lambda,
                );
                *orientations_quat_ptr.add(b.q2_index) =
                    (*orientations_quat_ptr.add(b.q2_index) * DQuat::from_scaled_axis(delta_theta_local_2)).normalize();
            }
        });
    }
}

pub fn solve_dihedral_bending_constraints_parallel(world: &mut World, h: f64, colors: &[Vec<usize>]) {
    let h2 = h * h;
    let pred_pos_ptr = SendPtr(world.particles.pred_pos.as_mut_ptr());
    let inv_mass_ptr = SendPtr(world.particles.inv_mass.as_ptr() as *mut f64);
    let constraints_ptr = SendPtr(world.dihedral_bending_constraints.as_mut_ptr());

    for color_class in colors {
        color_class.par_iter().for_each(|&idx| unsafe {
            let b = &mut *constraints_ptr.add(idx);
            let w1 = *inv_mass_ptr.add(b.p1);
            let w2 = *inv_mass_ptr.add(b.p2);
            let w3 = *inv_mass_ptr.add(b.p3);
            let w4 = *inv_mass_ptr.add(b.p4);

            let w_sum_mass = w1 + w2 + w3 + w4;
            if w_sum_mass <= 0.0 {
                return;
            }

            let p1 = *pred_pos_ptr.add(b.p1);
            let p2 = *pred_pos_ptr.add(b.p2);
            let p3 = *pred_pos_ptr.add(b.p3);
            let p4 = *pred_pos_ptr.add(b.p4);

            let angle = crate::world::compute_dihedral_angle(p1, p2, p3, p4);

            let mut constraint_val = angle - b.rest_angle;
            if constraint_val > std::f64::consts::PI {
                constraint_val -= 2.0 * std::f64::consts::PI;
            } else if constraint_val < -std::f64::consts::PI {
                constraint_val += 2.0 * std::f64::consts::PI;
            }

            let e = p2 - p1;
            let e_len = e.length();
            if e_len < 1e-12 {
                return;
            }

            let e1 = e.cross(p3 - p1);
            let e2 = (p4 - p1).cross(e);

            let e1_len = e1.length();
            let e2_len = e2.length();

            if e1_len < 1e-12 || e2_len < 1e-12 {
                return;
            }

            let n1 = e1 / e1_len;
            let n2 = e2 / e2_len;

            let h1 = e1_len / e_len;
            let h2_height = e2_len / e_len;

            let q3 = -n1 / h1;
            let q4 = -n2 / h2_height;

            let d1 = (p3 - p1).dot(e) / (e_len * e_len);
            let d2 = (p4 - p1).dot(e) / (e_len * e_len);

            let q1 = (d1 - 1.0) * q3 + (d2 - 1.0) * q4;
            let q2 = -d1 * q3 - d2 * q4;

            let w_sum = w1 * q1.length_squared()
                + w2 * q2.length_squared()
                + w3 * q3.length_squared()
                + w4 * q4.length_squared();

            let alpha_tilde = b.compliance / h2;
            let denom = w_sum + alpha_tilde;
            if denom < 1e-12 {
                return;
            }

            let delta_lambda = (-constraint_val - alpha_tilde * b.lambda) / denom;
            b.lambda += delta_lambda;

            if w1 > 0.0 {
                *pred_pos_ptr.add(b.p1) += w1 * delta_lambda * q1;
            }
            if w2 > 0.0 {
                *pred_pos_ptr.add(b.p2) += w2 * delta_lambda * q2;
            }
            if w3 > 0.0 {
                *pred_pos_ptr.add(b.p3) += w3 * delta_lambda * q3;
            }
            if w4 > 0.0 {
                *pred_pos_ptr.add(b.p4) += w4 * delta_lambda * q4;
            }
        });
    }
}

pub fn solve_unilateral_constraints_parallel(world: &mut World, h: f64, colors: &[Vec<usize>]) {
    let h2 = h * h;
    let pred_pos_ptr = SendPtr(world.particles.pred_pos.as_mut_ptr());
    let inv_mass_ptr = SendPtr(world.particles.inv_mass.as_ptr() as *mut f64);
    let constraints_ptr = SendPtr(world.unilateral_constraints.as_mut_ptr());

    for color_class in colors {
        color_class.par_iter().for_each(|&idx| unsafe {
            let b = &mut *constraints_ptr.add(idx);
            let w1 = *inv_mass_ptr.add(b.p1);
            let w2 = *inv_mass_ptr.add(b.p2);
            let w_sum = w1 + w2;
            if w_sum <= 0.0 {
                return;
            }

            let p1 = *pred_pos_ptr.add(b.p1);
            let p2 = *pred_pos_ptr.add(b.p2);

            let dist = (p1 - p2).length();
            let constraint_val = dist - b.rest_length;

            if constraint_val <= 0.0 && b.lambda >= 0.0 {
                return;
            }

            if dist < 1e-12 {
                return;
            }
            let e = (p1 - p2) / dist;

            let alpha_tilde = b.compliance / h2;
            let denom = w_sum + alpha_tilde;
            if denom < 1e-12 {
                return;
            }

            let delta_lambda = (-constraint_val - alpha_tilde * b.lambda) / denom;
            let new_lambda = (b.lambda + delta_lambda).min(0.0);
            let delta_lambda_clamped = new_lambda - b.lambda;
            b.lambda = new_lambda;

            if w1 > 0.0 {
                *pred_pos_ptr.add(b.p1) += w1 * delta_lambda_clamped * e;
            }
            if w2 > 0.0 {
                *pred_pos_ptr.add(b.p2) -= w2 * delta_lambda_clamped * e;
            }
        });
    }
}


