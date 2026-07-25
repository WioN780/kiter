# CLAUDE.md — Kite Engine Agent Context

This file is the *operating rules* for working on this repo, not the spec. See
`docs/ARCHITECTURE.md` for how the engine works, and `docs/kite-core.md`,
`docs/kite-cli.md`, `docs/kite-viz.md` for how each crate is organized. Don't
duplicate physics math here; look it up there.

We are building **only the simulation engine** (Rust, native, no UI/web/DB yet) for
a physically-accurate kite aeroelastic simulator: rigid/elastic spars as Cosserat
rods, anisotropic woven-fabric canopy, branching bridle lines, panel-method
aerodynamics, and a spatially non-uniform (patchy/turbulent) wind field, all solved
via XPBD.

Work proceeds incrementally, one well-scoped change at a time. The original
`docs/BUILD_PROMPTS.md` milestone checklist covered the now-complete initial
build-out and has been removed; there is no active milestone file to follow.

---

## Non-negotiable rules

1. **No change is "done" without its acceptance tests green.** Every constraint
   type needs a passing analytic/reference test (see `crates/kite-core/tests/`)
   before it counts as implemented. Don't loosen a tolerance just to make a red
   test pass: if a test won't pass at a reasonable tolerance, that's a bug to fix
   or a flag to raise, not a number to fudge.
2. **Stack:** `f64` precision, `glam` for math, struct-of-arrays data layout (no
   general ECS crate in the physics core). Don't switch to `f32` or add an ECS
   dependency.
   The solver was kept single-threaded through the initial build-out on purpose:
   Gauss-Seidel constraint solving reads and writes shared particle/orientation
   state each sweep, so naive parallelism there is a correctness and determinism
   risk (see rule 6), not just a performance question, and a validated serial
   baseline was needed before anything could be checked against it. Rayon-based
   parallelism was added later, on top of that baseline: only same-color
   constraint classes run concurrently (graph colored so parallel solves never
   touch shared state), gated behind a size threshold
   (`PARALLEL_CONSTRAINT_THRESHOLD`) so small scenes stay serial, with the
   original serial path kept intact behind the `reference-mode` feature as a
   bit-exact regression baseline. Any further parallelism needs the same
   discipline: provably race-free, not ad hoc threading, and the serial
   reference path must keep working.
3. **Compliance is never literally `0.0`** in a division — clamp to an epsilon or
   special-case "infinitely stiff" constraints as solved-exactly. This is the #1
   source of NaNs in XPBD ports; if you see a NaN, check this first.
4. **Y-up, right-handed world frame**, everywhere, no exceptions.
5. **`World::step(dt)` is a pure transform.** No hidden globals, no thread-local
   state, no wall-clock reads inside the engine.
6. **Determinism:** all randomness (wind noise seeding, anything stochastic) goes
   through a seeded RNG stored in `World`, passed in explicitly, never sourced from
   system time.
7. **Don't silently guess at unverified formulas.** A few spots in the codebase
   are explicitly marked as needing verification against a source (search for
   `UNVERIFIED:`, e.g. the flat-plate CoP travel in `aero/panel_method.rs` and the
   stiff-junction Darboux-vector convention in `geometry.rs`). When you hit one of
   these, either look up the cited paper and implement it properly, or implement
   your best derivation *and* flag it clearly in a code comment (`// UNVERIFIED:`)
   and in your summary to the user: don't ship an unverified derivation as if it
   were settled.
8. **Don't add a dependency that isn't already listed in a crate's `Cargo.toml`**
   without flagging it to the user first and explaining why.
9. **Don't reorder the solve sequence or change a documented architectural
   decision** (see `docs/ARCHITECTURE.md`) without flagging it: if you think a
   decision is wrong, say so explicitly rather than quietly working around it.
10. **`rapier3d` is scope-limited to collision detection** (ground + self-collision
    broad/narrow-phase, see `crates/kite-core/src/collision/`) and, optionally,
    truly rigid accessory bodies (see `crates/kite-core/src/control_bar.rs`). It is
    never the dynamics solver for rods, cloth, or bridle lines: those stay custom
    XPBD, full stop. If you find yourself reaching for a Rapier joint to represent
    a rod or a line, stop, that's the exact failure mode the original platform ADR
    already ruled out.

## Commands

- Build: `cargo build -p kite-core`
- Test: `cargo test -p kite-core`
- Lint (must be clean before calling anything "done"): `cargo clippy --all-targets -- -D warnings`
- Bench: `cargo bench -p kite-core` (only relevant from Milestone 8 onward)
- Run a scenario with live debug viz: `cargo run -p kite-cli -- scenarios/<name>.toml`

## Repo layout (abbreviated; see `docs/ARCHITECTURE.md` for the full picture)

```
kiter/
  crates/
    kite-core/     # the physics engine library, zero IO, zero rendering
    kite-cli/      # headless dev binary: loads a scenario, runs steps, logs to rerun
    kite-viz/      # native egui/wgpu editor and simulation viewer
  scenarios/       # *.toml scenario files used by tests and the CLI/viewer
  docs/
    ARCHITECTURE.md
    kite-core.md
    kite-cli.md
    kite-viz.md
    SCENARIO_FORMAT.md
```

## Scenario file format

The TOML scenario format is documented in `docs/SCENARIO_FORMAT.md`. Treat it as
stable: if new work needs a new field, extend it additively and update that doc,
don't redesign it.

## Definition of done

- [ ] `cargo build` and `cargo test` succeed with no new warnings
- [ ] `cargo clippy --all-targets -- -D warnings` is clean
- [ ] The task's specific acceptance criteria pass
- [ ] Any deviation from `docs/ARCHITECTURE.md` is called out explicitly in your
      summary to the user, with the reasoning
- [ ] Stop and report back; do not start unrelated follow-on work unprompted

## Communication style

When implementing constraint math or aerodynamics, briefly explain the physical
reasoning in your summary (not just "added function X"): the user is validating
this for physical accuracy, not just code style. If something in
`docs/ARCHITECTURE.md` looks wrong or you found a better approach while
implementing, say so explicitly rather than silently deviating.
