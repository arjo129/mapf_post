use mapf_post::*;

#[cfg(test)]
#[test]
fn test_get_claims_dict_follow() {
    use std::sync::Arc;

    use mapf_post::na::{Isometry2, Vector2};

    let mapf_result = MapfResult {
        trajectories: vec![
            vec![
                Isometry2::new(Vector2::new(0.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(1.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(2.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(3.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(4.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(4.0, 0.0), 0.0),
            ], // Trajectory for agent1
            vec![
                Isometry2::new(Vector2::new(1.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(2.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(3.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(4.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(5.0, 0.0), 0.0),
                Isometry2::new(Vector2::new(5.0, 0.0), 0.0),
            ], // Trajectory for agent2
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

    let semantic_plan = mapf_post(&mapf_result);

    let reserved_parts = semantic_plan.get_claim_dict(&vec![
        SemanticWaypoint {
            agent: 0,
            trajectory_index: 0,
        },
        SemanticWaypoint {
            agent: 1,
            trajectory_index: 3,
        },
    ]);

    assert_eq!(reserved_parts.len(), 2);
    assert_eq!(reserved_parts.get(&0).unwrap().end_id, 4);
    assert_eq!(reserved_parts.get(&1).unwrap().end_id, 6);

    let reserved_parts = semantic_plan.get_claim_dict(&vec![
        SemanticWaypoint {
            agent: 0,
            trajectory_index: 3,
        },
        SemanticWaypoint {
            agent: 1,
            trajectory_index: 3,
        },
    ]);

    assert_eq!(reserved_parts.len(), 2);
    assert_eq!(reserved_parts.get(&0).unwrap().end_id, 4);
    assert_eq!(reserved_parts.get(&1).unwrap().end_id, 6);
}
