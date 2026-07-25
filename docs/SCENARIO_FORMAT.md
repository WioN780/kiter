# Scenario file format

A scenario (`scenarios/*.toml`) is a TOML file describing a kite, the wind
it's flying in, and how long to run it. Both `kite-cli` and `kite-viz` parse
the same format (`Scenario` in `kite-cli/src/main.rs` and
`kite-viz/src/doc.rs` are independent structs but describe the same schema,
kept in sync by hand rather than shared as a single type). `KiteDefinition`
and its nested types (`kite-core/src/geometry.rs`) are the source of truth;
this document mirrors them.

## Top level

```toml
name = "Simple Stunt Kite Scenario"
gravity = [0.0, -9.81, 0.0]
duration = 5.0

[wind]
# ...

[sim]
# ...

[kite]
# ...
```

- `name`: scenario label.
- `gravity`: `[x, y, z]`, Y-up (see rule 4 in `AGENTS.md`); Earth gravity is
  `[0.0, -9.81, 0.0]`.
- `duration`: seconds to simulate.
- `wind`, `sim`, `kite` are all optional tables.

## `[sim]`: engine config overrides

An additive, all-optional table. Anything left out keeps `kite-core`'s
`Config` default.

```toml
[sim]
substeps = 24
iterations_per_substep = 1
damping = 2.0
bladder_pressure = 0.0
k_pressure = 1.0
ground_collision_enabled = false
self_collision_enabled = false
```

## `[wind]`

```toml
[wind]
v_ref = 8.0             # reference wind speed, m/s
h_ref = 10.0             # reference height for v_ref, m
shear_exponent = 0.15    # power-law exponent for height shear
direction = [0.0, 0.0, 1.0]   # horizontal unit vector
turbulence_intensity = 0.05   # std-dev / mean ratio; 0 = laminar
length_scale = 20.0      # turbulence integral length scale, m
octaves = 3               # curl-noise fractal octaves
seed = 42                 # deterministic RNG seed
```

Optional nested arrays for scripted weather:

```toml
[[wind.gusts]]
start_time = 2.0
length = 15.0
amplitude = 4.0
direction = [0.0, 0.0, 1.0]
propagation_velocity = 8.0

[[wind.thermals]]
center = [0.0, 0.0, 0.0]
radius = 5.0
strength = 3.0
start_time = 0.0
lifetime = 20.0
drift = [1.0, 0.0, 0.0]
```

## `[kite]`

```toml
[kite]
name = "Simple Stunt Kite"
bridle_junction = [0.0, -0.8, -1.0]
bridle_junction_pinned = true
```

`bridle_junction` is where all bridle lines converge (the tow point);
`bridle_junction_pinned` fixes it in space (a ground-anchored kite) versus
leaving it free (a kite flown purely on line tension, or coupled to a control
bar built separately by the host application).

### `spars`

```toml
spars = [
    { name = "Spine", start = [0,0,0.75], end = [0,0,-0.75],
      num_segments = 4, radius = 0.005,
      youngs_modulus = 4.0e10, shear_modulus = 4.0e9, density = 1950.0 },
]
```

Each spar becomes a chain of Cosserat rod segments between `start` and `end`.
`youngs_modulus`/`shear_modulus`/`density` are Pa/Pa/kg per cubic meter; see
`Material::carbon_fiber()` and `Material::fiberglass()` in `materials.rs` for
realistic ballpark values.

### `panels`

```toml
panels = [
    { name = "TL1", p1 = [0,0,-0.75], p2 = [-0.5,0,0], p3 = [0,0,-0.375],
      warp_compliance = 1.0e-4, weft_compliance = 1.0e-4,
      shear_compliance = 2.0e-4, bending_compliance = 0.1,
      subdivisions = 1, areal_density = 0.05 },
]
```

Each panel is a canopy triangle. `warp_compliance`/`weft_compliance` are the
along-fiber stretch compliance in each fabric direction (they can differ,
matching real woven cloth); `shear_compliance` resists in-plane skewing;
`bending_compliance` resists folding along edges shared with neighboring
panels. `subdivisions` (default 1) splits the triangle into an n^2 grid of
sub-triangles before building constraints, useful for panels that need finer
aerodynamic or structural resolution. `areal_density` (default 0.05 kg/m^2,
typical ripstop nylon) sets fabric mass per unit area.

### `bridles`

```toml
bridles = [
    { name = "Nose-line", from = [0,0,-0.75], to = [0,-0.8,-1.0],
      rest_length = 0.85, compliance = 1.0e-6, diameter = 0.002,
      num_segments = 1, density = 970.0 },
]
```

A bridle line runs from a kite attachment point (`from`) to `to`, which is
either the kite's `bridle_junction` or another bridle line's endpoint or
interior node (build order across bridles is resolved automatically; see
`geometry.rs`). It's a unilateral (tension-only) constraint: slack below
`rest_length`, a normal compliant distance constraint above it.
`num_segments` (default 1) chains multiple unilateral segments together
instead of one straight line, letting a long line sag or drape.
`density` defaults to 970 kg/m^3 (Dyneema).

### `pinned_points`, `stiff_junctions`, `lashings`

```toml
pinned_points = [[0.0, 0.0, 0.0]]

[[kite.stiff_junctions]]
point = [0.0, 0.0, 0.0]
compliance = 1.0e-8

[[kite.lashings]]
point = [0.5, 0.0, 0.0]
compliance = 1.0e-6
```

- `pinned_points`: fixes a point in space (inverse mass 0). Welds to an
  existing node within 1mm, or creates a new isolated anchor particle
  (e.g. a tether point).
- `stiff_junctions`: locks two different spars' segments together at their
  as-built relative angle where they meet at a welded point (a T-joint or
  cross-strut), instead of letting them hinge freely. Needs the two spars to
  already share a welded particle.
- `lashings`: a stiff, position-only tie (free pivot, no angle locking)
  between the nearest node of two different spars, for a zip-tie-style
  connection that doesn't require the spars to already share a node.

## Coordinate frame

Everything is Y-up, right-handed, in meters. There's no separate scaling or
unit-conversion step; whatever coordinates you write are the coordinates the
solver simulates.

## kite-viz's own document format

`kite-viz`'s in-editor `EditorDoc` (`kite-viz/src/doc.rs`) is index-based
(points are a flat list, elements reference indices) rather than
coordinate-based like the TOML above, since that's cheaper to mutate live in
an editor. It converts to and from this TOML schema via
`EditorDoc::to_scenario`/`from_scenario`; you never need to hand-edit the
index form, only the TOML.
