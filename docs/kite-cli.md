# kite-cli

A small headless binary for running a scenario without the editor: load a
TOML file, build the kite, step the simulation, print a summary, and
optionally record the run.

## Usage

```
cargo run -p kite-cli -- scenarios/simple_kite_v1.toml [output_snapshot.json] [--rerun | --rerun=<out.rrd>]
```

- The scenario path is required.
- The output snapshot path is optional; if omitted, it defaults to
  `<scenario_dir>/simple_kite_v1_snapshot.json` (a fixed name; the CLI
  doesn't derive it from the input filename).
- `--rerun` spawns the `rerun` viewer and streams the run live; `--rerun=path`
  saves an `.rrd` file instead of spawning a viewer window.

## What it does

`main.rs` deserializes the TOML into a `Scenario` (name, gravity, duration,
optional `wind`, optional `kite`, optional `[sim]` overrides), builds a
`World` from the `KiteDefinition` via `kite_core::build_kite_from_def`, then
steps the simulation at a fixed `dt = 0.01s` for `duration / dt` steps,
asserting every particle stays finite (`is_finite()`) after each step as a
cheap divergence check.

Every 10th step (plus the last one), it records a `SnapshotEntry` (kite time
plus five named particle positions: tail, nose, left tip, right tip, bridle
junction) into a trajectory, indexed by the particle layout of the
diamond-kite scenarios (`p_tail = 0`, `p_nose = 4`, `p_left = 6`, `p_right =
8`, `p_junction = 9`). This assumes the classic diamond-kite particle
ordering; it isn't a general per-scenario introspection tool. The trajectory
is written out as JSON at the end. `scenarios/simple_kite_v1_snapshot.json` is
a committed example of that output, used as a regression baseline.

If `--rerun` is passed, `log_rerun` streams particle positions, per-segment
rod material frames (the three director axes of each Cosserat segment,
drawn as colored arrows), and any external force vectors to the `rerun` SDK
each step, viewable live or replayed from the saved `.rrd`.

## What it's for

This is the fast path for iterating on physics changes and for regression
testing: run a known scenario, compare the resulting snapshot JSON against a
previous one, and anything that changed the trajectory shows up as a diff.
It has no editing or interactive capability; that's `kite-viz`.
