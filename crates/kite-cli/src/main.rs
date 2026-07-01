use std::env;
use std::fs::File;
use std::io::{Read, Write};
use std::path::Path;
use serde::{Deserialize, Serialize};
use glam::DVec3;
use kite_core::{World, build_kite_from_def, KiteDefinition};
use kite_core::wind::WindConfig;

#[derive(Debug, Deserialize)]
struct Scenario {
    name: String,
    gravity: DVec3,
    duration: f64,
    wind: Option<WindConfig>,
    kite: Option<KiteDefinition>,
}

#[derive(Debug, Serialize)]
struct SnapshotEntry {
    time: f64,
    nose: DVec3,
    tail: DVec3,
    left_tip: DVec3,
    right_tip: DVec3,
    junction: DVec3,
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <scenario_file_path> [output_snapshot_path]", args[0]);
        std::process::exit(1);
    }
    let scenario_path = &args[1];
    println!("Loading scenario file: {}", scenario_path);

    let mut file = File::open(scenario_path).expect("Failed to open scenario file");
    let mut toml_str = String::new();
    file.read_to_string(&mut toml_str).expect("Failed to read scenario file");

    let scenario: Scenario = toml::from_str(&toml_str).expect("Failed to parse TOML scenario");
    println!("Scenario Name: {}", scenario.name);
    println!("Gravity: {:?}", scenario.gravity);
    println!("Duration: {}s", scenario.duration);

    // Initialize World
    let mut world = World::new();
    world.cfg.gravity = scenario.gravity;
    
    if let Some(wind_cfg) = &scenario.wind {
        world.cfg.wind = wind_cfg.clone();
        println!("Wind Configured: v_ref={}, direction={:?}", wind_cfg.v_ref, wind_cfg.direction);
    }

    // Build Kite
    if let Some(kite_def) = &scenario.kite {
        println!("Building kite: {}", kite_def.name);
        build_kite_from_def(&mut world, kite_def);
        println!("Kite successfully built!");
        println!("Particles: {}", world.particles.len());
        println!("Distance constraints: {}", world.distance_constraints.len());
        println!("Stretch-shear constraints: {}", world.stretch_shear_constraints.len());
        println!("Bend-twist constraints: {}", world.bend_twist_constraints.len());
        println!("Dihedral bending constraints: {}", world.dihedral_bending_constraints.len());
        println!("Unilateral bridle constraints: {}", world.unilateral_constraints.len());
        println!("Canopy panels: {}", world.canopy_panels.len());
    } else {
        println!("No kite definition found in scenario.");
        return;
    }

    // Run simulation steps
    let dt = 0.01;
    let total_steps = (scenario.duration / dt).round() as usize;
    println!("Running {} simulation steps...", total_steps);

    let mut trajectory = Vec::new();

    // We assume standard particle indices for simple diamond kite:
    // Tail = 0, Nose = 4, Left tip = 6, Right tip = 8, Junction = 9 (based on simple_kite_v1.toml layout)
    let p_tail = 0;
    let p_nose = 4;
    let p_left = 6;
    let p_right = 8;
    let p_junction = 9;

    for step in 0..total_steps {
        world.step(dt);

        let t = world.time;
        
        // Assert no NaNs
        let nose_pos = world.particles.pos[p_nose];
        assert!(!nose_pos.x.is_nan() && !nose_pos.y.is_nan() && !nose_pos.z.is_nan(), "Simulation exploded at step {}", step);

        if step % 10 == 0 || step == total_steps - 1 {
            trajectory.push(SnapshotEntry {
                time: t,
                nose: world.particles.pos[p_nose],
                tail: world.particles.pos[p_tail],
                left_tip: world.particles.pos[p_left],
                right_tip: world.particles.pos[p_right],
                junction: world.particles.pos[p_junction],
            });
        }
    }

    println!("Simulation finished stably!");
    
    // Print final attitude
    let final_nose = world.particles.pos[p_nose];
    let final_tail = world.particles.pos[p_tail];
    let attitude_vector = (final_nose - final_tail).normalize();
    println!("Final Nose Position: {:?}", final_nose);
    println!("Final Tail Position: {:?}", final_tail);
    println!("Final Attitude Direction Vector: {:?}", attitude_vector);

    // Save snapshot trajectory if requested
    let output_path = if args.len() >= 3 {
        Some(args[2].clone())
    } else {
        // Default to same directory as scenario
        let p = Path::new(scenario_path);
        p.parent().map(|parent| parent.join("simple_kite_v1_snapshot.json").to_str().unwrap().to_string())
    };

    if let Some(path) = output_path {
        println!("Saving regression trajectory snapshot to: {}", path);
        let snapshot_json = serde_json::to_string_pretty(&trajectory).expect("Failed to serialize trajectory");
        let mut out_file = File::create(path).expect("Failed to create snapshot output file");
        out_file.write_all(snapshot_json.as_bytes()).expect("Failed to write snapshot output file");
        println!("Regression snapshot successfully recorded!");
    }
}
