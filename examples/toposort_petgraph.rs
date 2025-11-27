use std::sync::Arc;

use mapf_post::na::{Isometry2, Vector2};
use mapf_post::Trajectory;
use mapf_post::{mapf_post, SemanticWaypoint};
use mapf_post::{MapfResult, SemanticPlan};
use petgraph::algo::toposort;

pub fn print_sort_order(mapf_result: MapfResult, output_path: &str) {
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

    let p = semantic_plan.is_follower(SemanticWaypoint {
        agent: 0,
        trajectory_index: 2,
    });

    semantic_plan.figure_out_leader_follower_zone();

    println!("{:?}", p);
}

fn main() {
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

    let output_path = "output.dot"; // Specify the desired output path

    print_sort_order(mapf_result, output_path);
}
