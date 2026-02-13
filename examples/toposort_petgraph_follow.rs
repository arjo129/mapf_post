use std::sync::Arc;

use macroquad::miniquad::date::now;
use macroquad::prelude::*;
use mapf_post::Trajectory;
use mapf_post::na::{Isometry2, Vector2};
use mapf_post::{MapfResult, SemanticPlan};
use mapf_post::{SemanticWaypoint, mapf_post};
use petgraph::algo::toposort;

pub fn print_sort_order(mapf_result: &MapfResult) {
    let semantic_plan = mapf_post(mapf_result);

    // Generate the DOT representation
    let dot_representation = semantic_plan.to_dot();
    let pet_graph = semantic_plan.current_traffic_deps(&vec![
        SemanticWaypoint {
            agent: 0,
            trajectory_index: 2,
        },
        SemanticWaypoint {
            agent: 1,
            trajectory_index: 2,
        },
        SemanticWaypoint {
            agent: 2,
            trajectory_index: 2,
        },
    ]);

    let indices = toposort(&pet_graph, None).unwrap();

    for index in indices {
        println!("{:?}", pet_graph[index]);
    }

    let p = semantic_plan.is_follower(&SemanticWaypoint {
        agent: 0,
        trajectory_index: 2,
    });

    semantic_plan.figure_out_leader_follower_zone();
}

#[macroquad::main("MyGame")]
async fn main() {
    let mapf_result = MapfResult {
        trajectories: vec![
            vec![
                Isometry2::new(Vector2::new(0.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(1.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(2.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(3.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(4.0, 0.0), 0.0),
            ], // Trajectory for agent1 (horizontal path)
            vec![
                Isometry2::new(Vector2::new(1.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(2.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(3.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(4.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(5.0, 0.0), 0.0),
            ], // Trajectory for agent2 (vertical path)
            vec![
                Isometry2::new(Vector2::new(2.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(3.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(4.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(5.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(6.0, 0.0), 0.0),
            ],
        ]
        .into_iter()
        .map(|poses| Trajectory { poses })
        .collect(),
        footprints: vec![
            Arc::new(parry2d::shape::Ball::new(0.49)), // Footprint for agent1
            Arc::new(parry2d::shape::Ball::new(0.49)), // Footprint for agent2
            Arc::new(parry2d::shape::Ball::new(0.49)), // Footprint for agent2
        ],
        discretization_timestep: 1.0, // Example timestep
    };

    let mut i = 0;
    let mut last_time = now();
    loop {
        let interp = now() - last_time;
        clear_background(RED);
        for c in &mapf_result.trajectories {
            let p = (1.0 - interp as f32) * c.poses[i].translation.vector
                + c.poses[i + 1].translation.vector * (interp as f32);

            draw_circle(
                p.x * 40.0 + 20.0,
                p.y * 40.0 + screen_height() / 2.0,
                10.0,
                BLUE,
            );
        }

        if (now() - last_time) > 1.0 {
            i += 1;
            if i + 1 >= mapf_result.trajectories[0].poses.len() {
                i = 0;
                last_time = now();
            }
        }
        next_frame().await
    }
}

/*fn main() {
    // Create a cross junction MapfResult
    let mapf_result = MapfResult {
        trajectories: vec![
            vec![
                Isometry2::new(Vector2::new(0.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(1.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(2.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(3.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(4.0, 0.0), 0.0),
            ], // Trajectory for agent1 (horizontal path)
            vec![
                Isometry2::new(Vector2::new(1.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(2.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(3.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(4.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(5.0, 0.0), 0.0),
            ], // Trajectory for agent2 (vertical path)
            vec![
                Isometry2::new(Vector2::new(2.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(3.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(4.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(5.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(6.0, 0.0), 0.0),
            ],
        ]
        .into_iter()
        .map(|poses| Trajectory { poses })
        .collect(),
        footprints: vec![
            Arc::new(parry2d::shape::Ball::new(0.49)), // Footprint for agent1
            Arc::new(parry2d::shape::Ball::new(0.49)), // Footprint for agent2
            Arc::new(parry2d::shape::Ball::new(0.49)), // Footprint for agent2
        ],
        discretization_timestep: 1.0, // Example timestep
    };

    print_sort_order(&mapf_result);
}*/
