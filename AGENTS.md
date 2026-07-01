# CLAUDE.md — Kite Engine Agent Context

This file is the *operating rules* for working on this repo, not the spec. The full
engineering spec lives at `docs/kite_engine_rust_masterplan.md` — read the relevant
section before implementing anything in that area. Don't duplicate physics math
here; look it up there.

We are building **only the simulation engine** (Rust, native, no UI/web/DB yet) for
a physically-accurate kite aeroelastic simulator: rigid/elastic spars as Cosserat
rods, anisotropic woven-fabric canopy, branching bridle lines, panel-method
aerodynamics, and a spatially non-uniform (patchy/turbulent) wind field, all solved
via XPBD.

Work proceeds **one milestone at a time**, using the prompts in
`docs/BUILD_PROMPTS.md`, in order. Do not skip ahead or batch milestones together.

---

## Non-negotiable rules

1. **No milestone is "done" without its acceptance tests green.** Every constraint
   type needs a passing analytic/reference test (masterplan §11) before it counts
   as implemented. Don't loosen a tolerance just to make a red test pass — if a test
   won't pass at a reasonable tolerance, that's a bug to fix or a flag to raise, not
   a number to fudge.
2. **Stack is fixed:** single-threaded solver, `f64` precision, `glam` for
   math, struct-of-arrays data layout (no general ECS crate in the physics core).
   Don't introduce `rayon` parallelism, switch to `f32`, or add an ECS dependency
   before Milestone 9 — and only then if explicitly resumed there.
3. **Compliance is never literally `0.0`** in a division — clamp to an epsilon or
   special-case "infinitely stiff" constraints as solved-exactly. This is the #1
   source of NaNs in XPBD ports; if you see a NaN, check this first.
4. **Y-up, right-handed world frame**, everywhere, no exceptions.
5. **`World::step(dt)` is a pure transform.** No hidden globals, no thread-local
   state, no wall-clock reads inside the engine.
6. **Determinism:** all randomness (wind noise seeding, anything stochastic) goes
   through a seeded RNG stored in `World`, passed in explicitly, never sourced from
   system time.
7. **Don't silently guess at unverified formulas.** A few items in the masterplan
   are explicitly marked "verify against source" (the Cosserat-rod quaternion
   Jacobians, the dihedral bending-constraint gradient). When you hit one of these,
   either look up the cited paper and implement it properly, or implement your best
   derivation *and* flag it clearly in a code comment (`// UNVERIFIED:`) and in your
   summary to the user — don't ship an unverified derivation as if it were settled.
8. **Don't add a dependency that isn't already listed in masterplan §9** without
   flagging it to the user first and explaining why.
9. **Don't reorder the solve sequence or change a documented architectural
   decision (masterplan §14)** without flagging it — if you think a decision is
   wrong, say so explicitly rather than quietly working around it.
10. **`rapier3d` is approved from Milestone 8 onward, scope-limited to collision
    detection** (ground + self-collision broad/narrow-phase, masterplan §5.5) and,
    optionally, truly rigid accessory bodies. It is never the dynamics solver for
    rods, cloth, or bridle lines — those stay custom XPBD, full stop. If you find
    yourself reaching for a Rapier joint to represent a rod or a line, stop —
    that's the exact failure mode the original platform ADR already ruled out.

## Commands

- Build: `cargo build -p kite-core`
- Test: `cargo test -p kite-core`
- Lint (must be clean before calling anything "done"): `cargo clippy --all-targets -- -D warnings`
- Bench: `cargo bench -p kite-core` (only relevant from Milestone 8 onward)
- Run a scenario with live debug viz: `cargo run -p kite-cli -- scenarios/<name>.toml`

## Repo layout (abbreviated — full detail in masterplan §10)

```
kiter/
  crates/
    kite-core/     # the physics engine library — zero IO, zero rendering
    kite-cli/      # dev binary: loads a scenario, runs steps, logs to rerun
  scenarios/       # *.toml scenario files used by tests and the CLI
  docs/
    kite_engine_rust_masterplan.md
    BUILD_PROMPTS.md
    PROGRESS.md
    SCENARIO_FORMAT.md   # created in Milestone 0, kept stable after
```

## Scenario file format

The TOML scenario format is decided once, in Milestone 0, and documented in
`docs/SCENARIO_FORMAT.md`. Treat it as stable after that — if a later milestone
needs a new field, extend it additively and update that doc; don't redesign it.

## Definition of done, every milestone

- [ ] `cargo build` and `cargo test` succeed with no new warnings
- [ ] `cargo clippy --all-targets -- -D warnings` is clean
- [ ] The milestone's specific acceptance criteria (see its prompt) pass
- [ ] `docs/PROGRESS.md` checklist updated, with a one-line note on any deviation
      from the masterplan and why
- [ ] Stop and report back — do not start the next milestone unprompted

## Communication style

When implementing constraint math or aerodynamics, briefly explain the physical
reasoning in your summary (not just "added function X") — the user is validating
this for physical accuracy, not just code style. If something in the masterplan
looks wrong or you found a better approach while implementing, say so explicitly
rather than silently deviating.
