# Kiter architecture

Kiter simulates a kite as a network of deformable bodies (spars, fabric, bridle
lines) sitting in a wind field, and integrates the whole thing forward in time
with XPBD (Extended Position-Based Dynamics). This document explains how the
pieces fit together. For what a specific function does, read its doc comment;
this file is about the shape of the system, not line-by-line behavior.

## Why XPBD

Position-Based Dynamics solves for positions directly instead of integrating
forces twice (force -> acceleration -> velocity -> position). Each constraint
(a rod staying rigid, a cloth edge staying at its rest length, a bridle line
staying under a max length) computes how far a small set of particles are from
satisfying it, then nudges those particles toward satisfaction, weighted by
inverse mass. XPBD adds *compliance*: a spring-like softness parameter so
constraints behave like real materials with finite stiffness instead of
infinitely rigid rods, converging to the same behavior regardless of how many
substeps you run. That's what makes it practical to represent an aeroelastic
kite (spars that bend, cloth that stretches, lines that go slack) instead of a
rigid-body kite.

The alternative would be a classical force-based ODE solver (semi-implicit
Euler, RK4) integrating explicit spring/torque forces. That's more familiar
but numerically stiff structures (thin carbon rods, near-inextensible lines)
force tiny timesteps to stay stable. XPBD's iterative position correction
stays stable at larger timesteps because it's solving a sequence of small,
well-conditioned local problems instead of one large explicit force
integration.

## Data model

The physics state lives in `World` (`crates/kite-core/src/world.rs`), and it's
struct-of-arrays throughout, not an entity-component-system and not one struct
per particle. Two parallel sets carry the two kinds of degrees of freedom:

- `ParticleSet`: position, previous position, predicted position, velocity,
  and inverse mass, one entry per point mass (a rod endpoint, a cloth vertex,
  a bridle knot).
- `OrientationSet`: quaternion, previous quaternion, angular velocity, and
  inverse inertia tensor, one entry per rod *segment*. A rod segment is not a
  particle; it's the material frame living between two particles, needed
  because a Cosserat rod can twist and bend independently of how its
  endpoints move.

Constraints are stored as flat `Vec`s on `World`, one vector per constraint
type (`distance_constraints`, `bend_twist_constraints`,
`dihedral_bending_constraints`, and so on), each holding plain data: which
particle/orientation indices it couples, its rest value, its compliance, and
its accumulated Lagrange multiplier for the current substep. There's no
dynamic dispatch or constraint trait object; the solver just loops over each
`Vec` in turn. This keeps the hot loop cache-friendly and keeps every
constraint's math in one visible function instead of behind an interface.

## The solve loop

`World::step(dt)` (in `world.rs`) forwards into `solver::step_simulation`,
which is the whole engine's entry point and the only place time advances. It
splits `dt` into `substeps` (24 by default) and, for each substep:

1. Applies external forces and torques (gravity, wind drag on lines and
   canopy, aerodynamic lift/drag from the panel method) to velocities.
2. Predicts new positions/orientations by integrating velocity forward one
   substep, without touching the "real" position yet.
3. Resets each constraint's Lagrange multiplier to zero.
4. Runs `iterations_per_substep` Gauss-Seidel sweeps over every constraint
   type, each sweep nudging predicted positions/orientations toward
   satisfying that constraint and accumulating the multiplier.
5. Derives velocity from the change between predicted and previous position
   (`(x_pred - x_prev) / h`), applies damping, and commits the predicted
   state as the new real state.
6. Checks yield/break thresholds and emits `Event`s for anything that snapped
   or folded.

Constraint compliance is deliberately never allowed to hit exactly zero in a
division (see rule 3 in `AGENTS.md`): an "infinitely stiff" constraint is
clamped to a tiny epsilon compliance instead, since dividing by true zero is
the most common source of NaNs when porting XPBD from a paper to code.

## Constraint types

Each constraint type lives in its own file under `constraints/` and solves a
different physical coupling:

- **Distance / bending** (`mod.rs`): the generic building blocks. A distance
  constraint holds two particles at a rest separation; a bending constraint
  holds three particles' curvature near a rest value. Used for simple
  cloth-adjacent geometry.
- **Cosserat rod stretch-shear and bend-twist** (`rod_cosserat.rs`): the spar
  model. Stretch-shear couples two particles and one segment orientation so
  the segment can stretch, shear, and stay aligned with its own material
  frame; bend-twist couples two adjacent segment orientations via their
  relative rotation (the Darboux vector), giving the rod real bending and
  torsional stiffness instead of just being a chain of rigid links.
- **Cloth stretch-shear and dihedral bending** (`cloth_stretch_shear.rs`,
  `cloth_bending.rs`): the fabric model, anisotropic (warp/weft/shear
  compliances can all differ, matching how woven fabric actually behaves) plus
  a dihedral angle constraint across shared triangle edges so panels resist
  folding, not just stretching.
- **Unilateral distance** (`bridle_unilateral.rs`): a bridle line, which can
  go slack (zero force) but only ever pulls, never pushes; the constraint is
  a no-op below rest length and a normal compliant distance constraint above
  it.
- **Anchor / ground contact / self-collision** (`anchor.rs`,
  `ground_contact.rs`, `self_collision.rs`, `collision/mod.rs`): pinning and
  contact. Collision *detection* (point-vs-triangle self-intersection, ground
  plane) uses `parry3d-f64`; the actual contact response is still a custom
  XPBD position correction, not a Rapier rigid-body solve.

## Aerodynamics

`aero/panel_method.rs` treats the canopy as a set of flat triangular panels,
computes each panel's relative wind velocity and angle of attack, looks up
lift/drag coefficients from a flat-plate/thin-membrane model
(`coefficients.rs`), and applies the resulting force. Where to apply that
force per panel (not just how much) turned out to need real care: naive
approaches either can't produce pitching moment on a flat single-panel sail
or don't converge as the mesh is refined. See the doc comment on
`apply_canopy_aerodynamics` for the two approaches that were tried and
rejected before landing on chordwise-distribution loading. `unsteady.rs` adds
an optional added-mass correction (a light panel accelerating through air
entrains some of that air's inertia too), off by default since it mainly
matters for very light, fast-maneuvering kites. `spar_drag.rs` handles drag on
the rods themselves as cylinders in cross-flow, separately from canopy drag.

## Wind field

`wind/` composes a few independent effects into one sampled velocity field:
a mean shear profile (`mean_profile.rs`, power-law with height), advected
curl noise for patchy turbulence (`curl_noise.rs`, built on the `noise`
crate's Perlin implementation), scripted gust fronts (`gust_events.rs`, a
1-cosine ramp that travels through space), and optional convective thermal
cells (`thermal_cells.rs`, divergence-free vertical-velocity blobs that drift
and age). All of it is driven by a seeded RNG stored in `WindConfig`, never
by wall-clock time or thread-local nondeterminism, so a scenario replays
identically every run.

## Determinism and parallelism

Two things make `World::step` reproducible: no hidden global state (no
wall-clock reads, no unseeded RNGs), and the solve order for iteration order
that could vary is normalized (e.g. sorting a `HashMap`'s entries before
building constraints from it, since Rust doesn't guarantee hash map iteration
order across runs).

The solver has two paths: a serial Gauss-Seidel sweep, and an optional
parallel path (`solver.rs`) that graph-colors constraints so same-color
constraints touch disjoint particles/orientations and can be solved
concurrently with Rayon without a data race. The parallel path only kicks in
above `PARALLEL_CONSTRAINT_THRESHOLD` constraints, since Rayon's task overhead
isn't worth it on small scenes, and it's compiled out entirely under the
`reference-mode` feature so regression tests have a single, bit-exact serial
baseline to compare against.

## Optional rigid coupling: the control bar

Almost everything in the engine is custom XPBD, on purpose (see rule 10 in
`AGENTS.md`: Rapier is approved for collision detection and for genuinely
rigid *accessory* bodies, never as the dynamics solver for rods, cloth, or
lines). The one place a real Rapier rigid body shows up is the optional
control bar (`control_bar.rs`): a rigid bar pinned at its center by a
spherical joint, coupled to the XPBD world at exactly two points (its tips,
which drive two kinematic anchor particles) once per substep in each
direction. It's a deliberately narrow, explicit staggered exchange, not a
general physics-engine bridge.

## Crates

- `crates/kite-core`: the engine itself. Zero I/O, zero rendering, pure
  simulation. See `kite-core.md`.
- `crates/kite-cli`: a headless dev binary that loads a scenario, runs it,
  and optionally streams it to the `rerun` viewer. See `kite-cli.md`.
- `crates/kite-viz`: a native egui/wgpu application for building kites
  visually and watching them fly, on top of `kite-core`. See `kite-viz.md`.

## Scenario files

Kites and simulation settings are described in TOML (`scenarios/*.toml`),
loaded by both `kite-cli` and `kite-viz`. See `SCENARIO_FORMAT.md` for the
format.
