# Kiter

`kiter` is a high-fidelity, physically-accurate aeroelastic simulation engine designed specifically for kites. 

Currently, the project focuses entirely on the **core simulation engine** (written in Rust). In the future, this engine will power a **web-based CAD application** allowing users to design, simulate, and test custom kites in real-time under realistic weather conditions.

---

## Technical Architecture

The physics simulation is built from the ground up using **Extended Position-Based Dynamics (XPBD)** for stable, real-time dynamics:

*   **Spars & Frame:** Modeled as rigid/elastic Cosserat rods supporting bending, torsion, and shear.
*   **Canopy & Sail:** Modeled with an anisotropic woven-fabric model to match actual fabric stretch.
*   **Bridle Lines:** Branching cable networks with unilateral (tension-only) constraints.
*   **Aerodynamics:** A panel-method aerodynamic solver that computes lift, drag, and moment across the canopy panels, combined with specialized spar drag.
*   **Environment:** Spatially non-uniform, patchy, and turbulent wind fields (utilizing curl noise and mean profile gradients).

---

## Repository Structure

*   [`crates/kite-core`](file:///D:/Random%20Projects/kites/kiter/crates/kite-core): The core physics engine library (pure math, zero I/O, zero rendering).
*   [`crates/kite-cli`](file:///D:/Random%20Projects/kites/kiter/crates/kite-cli): A developer command-line tool that loads scenarios, runs simulation steps, and logs outputs.
*   [`scenarios/`](file:///D:/Random%20Projects/kites/kiter/scenarios): TOML scenario files describing the structure of various kites and test cases.

---

## Development

### Prerequisites

You will need a Rust toolchain installed.

### Build and Test

To build the physics core and CLI:
```bash
cargo build -p kite-core
cargo build -p kite-cli
```

To run the test suite (analytical and validation tests):
```bash
cargo test -p kite-core
```

### Running Scenarios

You can run a simulation scenario with the developer CLI:
```bash
cargo run -p kite-cli -- scenarios/simple_kite_v1.toml
```
