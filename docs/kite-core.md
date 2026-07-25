# kite-core

The physics engine. A library crate with no I/O and no rendering: it takes a
`World`, a `dt`, and turns the crank. Everything in `kiter` that actually
simulates a kite lives here; `kite-cli` and `kite-viz` are both thin shells
around this crate.

If you haven't already, read `ARCHITECTURE.md` first for the solve loop and
data model; this file is a tour of the module layout.

## Module map

```
src/
  lib.rs           re-exports the public API
  world.rs         World struct, Config, step(), energy accounting
  solver.rs        the substep loop: serial + parallel constraint solving
  particles.rs      ParticleSet (struct-of-arrays point masses)
  orientation.rs     OrientationSet (struct-of-arrays segment frames)
  materials.rs        Material properties -> constraint compliance
  geometry.rs      KiteDefinition -> World (the TOML-to-particles importer)
  control_bar.rs   optional rigid control bar, Rapier-backed
  constraints/     one file per constraint type
  aero/            panel-method aerodynamics
  wind/            wind field: shear, turbulence, gusts, thermals
  collision/       ground and self-collision detection + response
```

## `world.rs`

`World` owns every particle, orientation, constraint vector, and the current
`Config`. `Config` is where you set substep count, gravity, damping, wind,
and feature toggles (`ground_collision_enabled`, `self_collision_enabled`,
`parallel_solve`, `added_mass_enabled`). `World::step(dt)` is the only way
time moves; it's a pure function of `World` and `dt`, no hidden state.

`World::compute_total_energy()` sums kinetic (translational + rotational),
gravitational potential, and elastic energy across every constraint. It
exists for validation: a system with no external forcing should conserve
energy, so the analytic test suite (`tests/validation_analytic.rs`) leans on
this heavily to catch subtly wrong constraint math that would otherwise still
"look" stable.

## `solver.rs`

The substep loop lives here (see `ARCHITECTURE.md` for the step-by-step
breakdown). Two things worth knowing if you're reading this file directly:

- Every constraint type has two solve functions, `solve_x_constraints` and
  `solve_x_constraints_parallel`. They implement the same math; the parallel
  version just operates on one graph-coloring "color class" at a time so it
  can hand disjoint chunks to Rayon safely. `SolverColoring` caches the
  coloring and rebuilds it only when constraint counts change.
- `PARALLEL_CONSTRAINT_THRESHOLD` (512) gates whether the parallel path runs
  at all; below it, Rayon's per-task overhead costs more than it saves.

## `particles.rs` / `orientation.rs`

Two small struct-of-arrays containers. `ParticleSet` is a point mass:
position, previous position, predicted position, velocity, inverse mass.
`OrientationSet` is a segment's material frame: quaternion, previous
quaternion, angular velocity, inverse inertia. Inverse mass/inertia of `0.0`
means pinned/infinitely heavy, which is how anchors, welded joints, and
kinematic control-bar anchors are represented; there's no separate "is this
pinned" flag.

## `materials.rs`

Converts physical material properties (Young's modulus, shear modulus,
density, cross-section geometry) into the compliance values the constraints
actually consume. `Material` has presets for carbon fiber and fiberglass rod.
`SectionGeometry` (solid round or hollow tube) computes cross-sectional area
and the two moments of inertia (bending, torsion) needed for
`stretch_shear_compliance` and `bend_twist_compliance`. This is the one place
"this rod is 5mm carbon fiber" turns into "this constraint has compliance
X m/N."

## `geometry.rs`

The importer: takes a `KiteDefinition` (spars, panels, bridle lines, pinned
points, stiff junctions, lashings, all described by position in space) and
builds the actual particles, segment orientations, and constraints in a
`World`, welding coincident points within a 1mm tolerance so spars, panels,
and bridles that share a physical location share a particle. This is the
file `kite-viz`'s editor and `kite-cli`'s scenario loader both ultimately
call into. It's also the biggest file in the crate because kite topology has
real edge cases: bridle lines that terminate on another bridle's interior
node, T-junctions between different spars, subdivided cloth panels needing a
shared-edge map for dihedral constraints. The comments in this file explain
those cases as they come up rather than up front.

## `control_bar.rs`

An optional rigid pilot control bar, the one place the crate uses Rapier's
actual rigid-body dynamics instead of custom XPBD (see the "Optional rigid
coupling" section of `ARCHITECTURE.md` for why it's boxed off like this). Not
built by default; a scenario or the CLI/viz layer constructs one and attaches
it to two kinematic anchor particles when it wants a dynamic, swingable bar
instead of two fixed pilot points.

## `constraints/`

One file per constraint kernel. `mod.rs` holds the two generic building
blocks (`DistanceConstraint`, `BendingConstraint`); everything else is
specific to a physical part of the kite:

| File | Models |
|---|---|
| `rod_cosserat.rs` | spar stretch/shear and bend/twist (Cosserat rod) |
| `cloth_stretch_shear.rs` | anisotropic fabric stretch (warp/weft/shear) |
| `cloth_bending.rs` | fabric resistance to folding along a shared edge |
| `bridle_unilateral.rs` | tension-only bridle/tether lines |
| `anchor.rs` | fixed pin points |
| `ground_contact.rs` | ground-plane contact response |
| `self_collision.rs` | point-vs-panel self-collision response |

`ConstraintState` (`Active` / `Folded` / `Broken`) tracks failure modes for
constraints that can yield: a `Folded` leading-edge joint gets a much softer
effective compliance instead of vanishing outright, and a `Broken` one stops
being solved. See `process_yield_and_breaking` in `solver.rs`.

## `aero/`

- `panel_method.rs`: per-panel relative wind, angle of attack, and force
  application. The interesting part is *where* on the panel the force gets
  applied, covered in `ARCHITECTURE.md`.
- `coefficients.rs`: `flat_plate_coefficients(alpha)`, a closed-form
  lift/drag/normal-force curve for a flat plate or thin membrane across the
  full angle-of-attack range, not just small angles.
- `spar_drag.rs`: cylinder cross-flow drag on rod segments, independent of
  canopy drag.
- `unsteady.rs`: optional added-mass correction (`Config::added_mass_enabled`)
  for light, fast-accelerating panels.

## `wind/`

Each file is an independent contribution to the sampled wind velocity at a
point in space and time; `WindConfig` composes them:

- `mean_profile.rs`: power-law height shear.
- `curl_noise.rs`: advected Perlin curl noise for patchy turbulence, built on
  the `noise` crate. Rebuilding the noise field's permutation tables is
  comparatively expensive, so it's memoized per-thread (see the comment on
  `FIELD_CACHE` in `wind/mod.rs` for why that memoization is still
  deterministic).
- `gust_events.rs`: scripted 1-cosine gust fronts that travel through space
  starting at a configured time.
- `thermal_cells.rs`: drifting, aging, radially-symmetric vertical-velocity
  blobs (convective thermals), built so the field stays analytically
  divergence-free.

## `collision/`

Detection only, using `parry3d-f64` for point-vs-triangle self-collision
queries and a simple ground-plane check. The actual contact *response*
(pushing particles apart) is a custom XPBD position constraint, same as
everything else in the engine; Rapier/Parry never touch rod, cloth, or line
dynamics (see rule 10 in `AGENTS.md`).

## Tests

`tests/` holds the validation suite: analytic reference tests (energy
conservation, closed-form deflection formulas), feature-specific tests
(control bar, thermals, unsteady aero, parallel-vs-serial agreement), and
`readback.rs` (force sign conventions for anything reading `lambda` back out
of a constraint for visualization). `benches/solver_bench.rs` is a Criterion
benchmark of the substep loop.
