use criterion::{black_box, criterion_group, criterion_main, Criterion};
use mapf_post::{
    mapf_post,
    na::{Isometry2, Vector2},
    MapfResult, Trajectory,
};
use parry2d::shape::Ball;
use std::sync::Arc;

fn bench_agent_scaling(c: &mut Criterion) {
    let mut group = c.benchmark_group("agent_scaling");
    for &num_agents in &[10, 50, 100, 200, 500] {
        group.bench_with_input(format!("agents_{}", num_agents), &num_agents, |b, &n| {
            let mut trajectories = Vec::with_capacity(n);
            let mut footprints = Vec::with_capacity(n);
            let num_waypoints = 10;
            let step_dist = 1.0;

            for agent_idx in 0..n {
                let mut poses = Vec::with_capacity(num_waypoints);
                for wp_idx in 0..num_waypoints {
                    let x = (wp_idx as f32 - agent_idx as f32) * step_dist;
                    poses.push(Isometry2::new(Vector2::new(x, 0.0), 0.0));
                }
                trajectories.push(Trajectory { poses });
                footprints.push(Arc::new(Ball::new(0.49)) as Arc<dyn parry2d::shape::Shape>);
            }

            let mapf_result = MapfResult {
                trajectories,
                footprints,
                discretization_timestep: 1.0,
            };

            b.iter(|| {
                black_box(mapf_post(&mapf_result));
            });
        });
    }
    group.finish();
}

criterion_group!(benches, bench_agent_scaling);
criterion_main!(benches);
