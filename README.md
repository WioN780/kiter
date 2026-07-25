# Kiter

Kiter is an aeroelastic simulation engine for kites: rigid/elastic spars,
woven-fabric canopy, branching bridle lines, panel-method aerodynamics, and a
non-uniform wind field, all solved with Extended Position-Based Dynamics
(XPBD).

Right now the project is the simulation engine and a native debug/editor
tool, both in Rust. A web-based CAD application on top of the same engine is
a future goal, not part of this repo yet.

## How it works

The physics runs on XPBD rather than a classical force-integration ODE
solver, which is what lets thin carbon-fiber rods and near-inextensible
bridle lines stay stable without tiny timesteps. Spars are Cosserat rods
(bending, torsion, and shear, not just rigid links). The canopy is an
anisotropic woven-fabric model, so warp, weft, and shear stretch each have
their own stiffness. Bridle lines are unilateral (tension-only) constraints
that go slack instead of resisting compression. Aerodynamic loads come from
a panel-method solver over the canopy triangles, plus separate cylinder drag
on the spars. Wind is spatially non-uniform: a mean shear profile, advected
curl-noise turbulence, scripted gusts, and optional thermal cells.

See `docs/ARCHITECTURE.md` for the full picture.

## Repository structure

- `crates/kite-core`: the physics engine library (zero I/O, zero rendering).
  See `docs/kite-core.md`.
- `crates/kite-cli`: a headless dev binary that loads a scenario, runs it,
  and can stream it to the `rerun` viewer. See `docs/kite-cli.md`.
- `crates/kite-viz`: a native egui/wgpu editor and simulation viewer built
  on `kite-core`. See `docs/kite-viz.md`.
- `scenarios/`: TOML files describing kites and test cases. See
  `docs/SCENARIO_FORMAT.md`.

## Development

You'll need a Rust toolchain installed.

Build:
```bash
cargo build -p kite-core
cargo build -p kite-cli
cargo build -p kite-viz
```

Run the test suite (analytical and validation tests):
```bash
cargo test -p kite-core
```

Run a scenario headlessly:
```bash
cargo run -p kite-cli -- scenarios/simple_kite_v1.toml
```

Run the editor/viewer:
```bash
cargo run -p kite-viz
```
