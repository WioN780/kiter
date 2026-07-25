# kite-viz

A native egui/wgpu application for building kites visually and watching them
fly, sitting on top of `kite-core`. It's a development tool (a CAD-ish editor
plus a live simulation viewer), not part of the physics engine itself; nothing
in `kite-core` depends on it.

## Two modes

The app (`app.rs`, `App::mode: Mode`) has exactly two modes:

- **Edit**: build and modify the kite's structure. Points, spars, panels,
  bridle lines, pins, stiff junctions, and lashings are added and connected
  with click-based tools (`tools.rs`), rendered against the in-memory
  `EditorDoc`.
- **Simulate**: run the kite through `kite-core`, watch it fly, and tinker
  with live config (gravity, wind, damping, substeps) while it runs.

Switching from Edit to Simulate builds a fresh `SimSession` (`session.rs`)
from the current `EditorDoc`, unless one already exists, in which case it's
reused so playback history survives toggling back and forth. Structural edits
made in Edit mode (points, spars, panels, bridles) do not retroactively patch
the running `World`; `SimSession::is_stale` detects the drift by hashing the
doc's structural fields and the UI offers an explicit "Apply & Restart"
instead of silently running a stale simulation. Live config edits (gravity,
wind, sim parameters) are the exception: those write directly into
`world.cfg` while simulating, since a plain field mutation can't invalidate
the constraint list or the cached parallel-solve coloring.

## `doc.rs`: the editor's own document format

`EditorDoc` is index-based: points live in one flat `Vec<DVec3>`, and every
other element (a spar, a panel, a bridle line) stores indices into that
vector rather than raw coordinates. That's what lets the editor weld and
reuse points cheaply while you're actively dragging things around. It also
holds the serde `Scenario`/`SimOverrides` types that mirror the TOML files in
`scenarios/`, plus `to_scenario`/`from_scenario` conversions to and from
`kite-core`'s coordinate-based `KiteDefinition`. Since `kite-core` re-welds
coincident coordinates within its own 1mm tolerance on build, an index shared
by two elements in `EditorDoc` is guaranteed to map to the same particle on
the `kite-core` side; the round trip is stable by construction, not by
convention.

## `tools.rs`: editing

CPU-side ray picking against the doc's geometry, a small state machine per
tool (some tools like `Spar` need two point clicks; `AddPoint` needs one),
and the actual mutation each tool performs (add a point, wire up a spar
between two picked points, delete a selection and cascade-delete anything
that referenced a deleted point). Selection (`SelectionSet`) is a Blender-
style ordered, deduplicated multi-select: click replaces it, shift-click
toggles a member, clicking empty space clears it. The translate/rotate gizmo
(axis arrows, rotation rings) is also built here; `ring_basis`/`ring_angle`
handle the plane-projection math for rotation dragging.

## `session.rs`: running the simulation

`SimSession` owns the live `World`, paces stepping against wall-clock time
(so playback speed is independent of frame rate), and keeps a capped ring
buffer of `Snapshot`s (`snapshot.rs`) for scrubbing back through history.
`GrabState` implements a mouse "grab a particle and drag it" interaction by
temporarily zeroing its inverse mass, and `ColorScale` autoscales the force
color ramps and arrow lengths per channel (spar, bridle, cloth, aero) with a
decaying running max so the visualization doesn't need a fixed force scale
picked in advance. `EventMarker` turns `kite-core`'s `Event`s (spar breaks,
leading-edge folds) into persistent 3D markers and timeline ticks.

## `panels.rs`: the UI chrome

The egui side panels: an element tree and property inspector for the
selected item, global scenario/sim/wind/view sections, the Simulate-mode
live-config inspector, and the bottom timeline/playback bar.

## `render.rs`: the GPU layer

Three wgpu render pipelines (instanced lit primitives like spheres and
cylinders, unlit line segments, and a double-sided dynamic mesh for cloth
panels), driven through an `egui_wgpu` paint callback. `SceneData` is the
frozen contract every other module builds against: they populate a
`SceneData` describing what to draw this frame, and `render.rs` is the only
place that touches wgpu directly. `shaders.wgsl` holds the corresponding
vertex/fragment shaders.

## `viewport.rs`: the 3D view

The orbit camera and the widget that hosts the wgpu paint callback. Produces
a `Ray` from the current pointer position each frame for `tools.rs` to pick
against; camera orbiting/panning/zooming stays entirely inside this module.

## `windviz.rs`: wind overlay

A sampled arrow grid plus advected "streak" particles, both driven by
`kite_core::wind::wind_at`, so you can see the turbulence and gusts you've
configured instead of only inferring them from how the kite moves. Owned by
`App` directly (not `SimSession`) so the streaks and toggle state survive
switching between Edit and Simulate.

## `snapshot.rs`: playback history

A per-step recording of world state, captured in `f32` (not the engine's
`f64`) to keep a few thousand frames of history cheap to hold in memory.
Also documents the force sign conventions used when reading a constraint's
accumulated Lagrange multiplier back out for visualization (`force =
-lambda / h^2`, with the sign flipped per constraint type to read as a
physically intuitive tension/compression), matching
`kite-core/tests/readback.rs`.
