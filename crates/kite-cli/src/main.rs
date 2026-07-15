use glam::DVec3;
use kite_core::wind::WindConfig;
use kite_core::{build_kite_from_def, KiteDefinition, World};
use serde::Deserialize;
use std::env;
use std::fs::File;
use std::io::Read;

#[derive(Debug, Deserialize)]
struct Scenario {
    name: String,
    gravity: DVec3,
    duration: f64,
    wind: Option<WindConfig>,
    kite: Option<KiteDefinition>,
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <scenario_file_path>", args[0]);
        std::process::exit(1);
    }
    let scenario_path = &args[1];
    println!("Loading scenario file: {}", scenario_path);

    let mut file = File::open(scenario_path).expect("Failed to open scenario file");
    let mut toml_str = String::new();
    file.read_to_string(&mut toml_str)
        .expect("Failed to read scenario file");

    let scenario: Scenario = toml::from_str(&toml_str).expect("Failed to parse TOML scenario");
    println!("Scenario Name: {}", scenario.name);
    println!("Gravity: {:?}", scenario.gravity);
    println!("Duration: {}s", scenario.duration);

    // Initialize World
    let mut world = World::new();
    world.cfg.gravity = scenario.gravity;

    if let Some(wind_cfg) = &scenario.wind {
        world.cfg.wind = wind_cfg.clone();
        println!(
            "Wind Configured: v_ref={}, direction={:?}",
            wind_cfg.v_ref, wind_cfg.direction
        );
    }

    // Build Kite
    if let Some(kite_def) = &scenario.kite {
        println!("Building kite: {}", kite_def.name);
        build_kite_from_def(&mut world, kite_def);
        println!("Kite successfully built!");
        println!("Particles: {}", world.particles.len());
        println!("Distance constraints: {}", world.distance_constraints.len());
        println!(
            "Stretch-shear constraints: {}",
            world.stretch_shear_constraints.len()
        );
        println!(
            "Bend-twist constraints: {}",
            world.bend_twist_constraints.len()
        );
        println!(
            "Dihedral bending constraints: {}",
            world.dihedral_bending_constraints.len()
        );
        println!(
            "Unilateral bridle constraints: {}",
            world.unilateral_constraints.len()
        );
        println!("Canopy panels: {}", world.canopy_panels.len());
    } else {
        println!("No kite definition found in scenario.");
        return;
    }

    // Run simulation steps
    let dt = 0.01;
    let total_steps = (scenario.duration / dt).round() as usize;
    println!("Running {} simulation steps...", total_steps);

    for step in 0..total_steps {
        world.step(dt);
        for (i, p) in world.particles.pos.iter().enumerate() {
            assert!(
                p.is_finite(),
                "Simulation exploded at step {} (particle {})",
                step,
                i
            );
        }
    }

    println!("Simulation finished stably!");
}
