use crate::world::World;
use glam::DVec3;
use parry3d_f64::math::Point;
use parry3d_f64::query::PointQuery;
use parry3d_f64::shape::Triangle;

/// A point-vs-triangle self-collision contact constraint.
#[derive(Clone, Debug)]
pub struct ContactConstraint {
    pub p_i: usize,
    pub p_a: usize,
    pub p_b: usize,
    pub p_c: usize,
    pub normal: DVec3, // points from triangle towards particle i
    pub depth: f64,
    pub u: f64, // barycentric coordinates
    pub v: f64,
    pub w: f64,
    pub lambda: f64,
}

/// Contact thickness (m) shared by self-collision detection and the contact
/// solve — must stay a single constant so the detection margin and the solved
/// separation cannot drift apart.
const CONTACT_THICKNESS: f64 = 0.01;

/// Detects all active self-collisions (point-vs-triangle) between non-adjacent particles/canopy panels.
pub fn detect_self_collisions(world: &World) -> Vec<ContactConstraint> {
    let mut contacts = Vec::new();
    let thickness = CONTACT_THICKNESS;

    let n_particles = world.particles.len();
    let n_panels = world.canopy_panels.len();

    // Check adjacency to avoid false self-collision between neighboring vertices
    let is_adjacent = |p_i: usize, tri_vertices: &[usize]| -> bool {
        if tri_vertices.contains(&p_i) {
            return true;
        }
        for c in &world.distance_constraints {
            if (c.p1 == p_i && tri_vertices.contains(&c.p2))
                || (c.p2 == p_i && tri_vertices.contains(&c.p1))
            {
                return true;
            }
        }
        false
    };

    for i in 0..n_particles {
        let pos_i = world.particles.pred_pos[i];

        for t_idx in 0..n_panels {
            let panel = &world.canopy_panels[t_idx];
            let tri_vertices = [panel.p1, panel.p2, panel.p3];

            if is_adjacent(i, &tri_vertices) {
                continue;
            }

            let pos_a = world.particles.pred_pos[panel.p1];
            let pos_b = world.particles.pred_pos[panel.p2];
            let pos_c = world.particles.pred_pos[panel.p3];

            // Build Parry Triangle
            let tri = Triangle::new(
                Point::new(pos_a.x, pos_a.y, pos_a.z),
                Point::new(pos_b.x, pos_b.y, pos_b.z),
                Point::new(pos_c.x, pos_c.y, pos_c.z),
            );

            let p = Point::new(pos_i.x, pos_i.y, pos_i.z);
            let proj = tri.project_local_point(&p, true);

            let d_vec = pos_i - DVec3::new(proj.point.x, proj.point.y, proj.point.z);
            let dist = d_vec.length();

            if dist < thickness {
                let normal = if dist > 1.0e-6 {
                    d_vec / dist
                } else {
                    // Fallback to normal of the triangle
                    let tri_normal = (pos_b - pos_a).cross(pos_c - pos_a);
                    if tri_normal.length_squared() > 1.0e-12 {
                        tri_normal.normalize()
                    } else {
                        DVec3::Y
                    }
                };

                let (u, v, w) = compute_barycentric(
                    DVec3::new(proj.point.x, proj.point.y, proj.point.z),
                    pos_a,
                    pos_b,
                    pos_c,
                );

                contacts.push(ContactConstraint {
                    p_i: i,
                    p_a: panel.p1,
                    p_b: panel.p2,
                    p_c: panel.p3,
                    normal,
                    depth: thickness - dist,
                    u,
                    v,
                    w,
                    lambda: 0.0,
                });
            }
        }
    }

    contacts
}

/// Helper to compute barycentric coordinates of a projected point onto a triangle.
fn compute_barycentric(p: DVec3, a: DVec3, b: DVec3, c: DVec3) -> (f64, f64, f64) {
    let v0 = b - a;
    let v1 = c - a;
    let v2 = p - a;

    let d00 = v0.dot(v0);
    let d01 = v0.dot(v1);
    let d11 = v1.dot(v1);
    let d20 = v2.dot(v0);
    let d21 = v2.dot(v1);

    let denom = d00 * d11 - d01 * d01;
    if denom.abs() < 1.0e-12 {
        return (1.0, 0.0, 0.0);
    }

    let v = (d11 * d20 - d01 * d21) / denom;
    let w = (d00 * d21 - d01 * d20) / denom;
    let u = 1.0 - v - w;

    // Clamp coordinates to stay strictly inside the triangle boundary
    let u_clamped = u.clamp(0.0, 1.0);
    let v_clamped = v.clamp(0.0, 1.0);
    let w_clamped = w.clamp(0.0, 1.0);
    let sum = u_clamped + v_clamped + w_clamped;

    (u_clamped / sum, v_clamped / sum, w_clamped / sum)
}

/// Solves point-vs-triangle self-collision contacts.
pub fn solve_contact_constraints(world: &mut World, contacts: &mut [ContactConstraint]) {
    for b in contacts {
        let w_i = world.particles.inv_mass[b.p_i];
        let w_a = world.particles.inv_mass[b.p_a];
        let w_b = world.particles.inv_mass[b.p_b];
        let w_c = world.particles.inv_mass[b.p_c];

        let pos_i = world.particles.pred_pos[b.p_i];
        let pos_a = world.particles.pred_pos[b.p_a];
        let pos_b = world.particles.pred_pos[b.p_b];
        let pos_c = world.particles.pred_pos[b.p_c];

        let p_closest = b.u * pos_a + b.v * pos_b + b.w * pos_c;
        let constraint_val = (pos_i - p_closest).dot(b.normal) - CONTACT_THICKNESS;

        // Active only when penetrating
        if constraint_val >= 0.0 && b.lambda <= 0.0 {
            continue;
        }

        let denom = w_i + b.u * b.u * w_a + b.v * b.v * w_b + b.w * b.w * w_c;
        if denom < 1.0e-12 {
            continue;
        }

        let delta_lambda = -constraint_val / denom;
        let new_lambda = (b.lambda + delta_lambda).max(0.0);
        let delta_lambda_clamped = new_lambda - b.lambda;
        b.lambda = new_lambda;

        if w_i > 0.0 {
            world.particles.pred_pos[b.p_i] += w_i * delta_lambda_clamped * b.normal;
        }
        if w_a > 0.0 {
            world.particles.pred_pos[b.p_a] -= b.u * w_a * delta_lambda_clamped * b.normal;
        }
        if w_b > 0.0 {
            world.particles.pred_pos[b.p_b] -= b.v * w_b * delta_lambda_clamped * b.normal;
        }
        if w_c > 0.0 {
            world.particles.pred_pos[b.p_c] -= b.w * w_c * delta_lambda_clamped * b.normal;
        }
    }
}

/// Solves infinitely-stiff ground-plane collisions (projecting vertical coordinate).
pub fn solve_ground_collision(world: &mut World) {
    let thickness = 0.005; // 5mm ground thickness
    for i in 0..world.particles.len() {
        let w = world.particles.inv_mass[i];
        if w <= 0.0 {
            continue;
        }
        let pos = world.particles.pred_pos[i];
        if pos.y < thickness {
            world.particles.pred_pos[i].y = thickness;
        }
    }
}
