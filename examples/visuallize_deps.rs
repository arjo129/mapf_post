use std::sync::Arc;

use mapf_post::MapfResult;
use mapf_post::mapf_post;
use mapf_post::Trajectory;
use parry2d::na::Isometry2;
use parry2d::na::Vector2;

pub fn generate_dot_file(mapf_result: MapfResult, output_path: &str) {
    let semantic_plan = mapf_post(mapf_result);

    // Generate the DOT representation
    let dot_representation = semantic_plan.to_dot();

    // Write the DOT representation to the specified file
    std::fs::write(output_path, dot_representation).expect("Unable to write DOT file");
}

fn main() {
    // Create a cross junction MapfResult
    let mapf_result = MapfResult {
        trajectories: vec![
            vec![
                Isometry2::new(Vector2::new(0.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(1.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(2.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(2.0, 0.0), 0.0),
            ], // Trajectory for agent1 (horizontal path)
            vec![
                Isometry2::new(Vector2::new(1.0, -1.0), 0.0),
                Isometry2::new(Vector2::new(1.0, -1.0), 0.0),
                Isometry2::new(Vector2::new(1.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(1.0, 1.0), 0.0),
            ], // Trajectory for agent2 (vertical path)
        ]
        .into_iter()
        .map(|poses| Trajectory { poses })
        .collect(),
        footprints: vec![
            Arc::new(parry2d::shape::Ball::new(0.49)), // Footprint for agent1
            Arc::new(parry2d::shape::Ball::new(0.49)), // Footprint for agent2
        ],
        discretization_timestep: 1.0, // Example timestep
    };

    let output_path = "output.dot"; // Specify the desired output path

    generate_dot_file(mapf_result, output_path);
}