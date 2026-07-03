use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use glam::DVec3;
use kite_core::geometry::build_cloth_grid;
use kite_core::World;

fn make_world(n: usize, parallel: bool) -> World {
    let mut world = World::new();
    world.cfg.parallel_solve = parallel;
    world.cfg.substeps = 8;
    world.cfg.iterations_per_substep = 2;
    world.cfg.gravity = DVec3::new(0.0, -9.81, 0.0);
    build_cloth_grid(&mut world, n, 0.05, 0.02);
    world
}

fn bench_step(c: &mut Criterion) {
    let mut group = c.benchmark_group("world_step_cloth");
    group.sample_size(10);

    for n in [32usize, 64, 128] {
        for (label, parallel) in [("serial", false), ("parallel", true)] {
            group.bench_with_input(BenchmarkId::new(label, format!("{n}x{n}")), &n, |b, &n| {
                let mut world = make_world(n, parallel);
                // Warm up: settles initial transients and builds the coloring cache.
                world.step(0.01);
                b.iter(|| world.step(0.01));
            });
        }
    }
    group.finish();
}

criterion_group!(benches, bench_step);
criterion_main!(benches);
