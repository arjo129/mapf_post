use mapf_post::*;

#[cfg(test)]
#[test]
fn test_check_for_violation() {
    use std::sync::Arc;
    use mapf_post::na::{Isometry2, Vector2};

    // Setup same as test_get_claim_dict.rs
    // Agent 0 and Agent 1 follow the same path, Agent 1 starts ahead.
    // Agent 0: (0,0) -> (1,0) -> (2,0) ...
    // Agent 1: (1,0) -> (2,0) -> (3,0) ...
    // So Agent 0 at step k corresponds to Agent 1 at step k-1 location.
    // Agent 0 at step k requires Agent 1 to have left step k-1 (which is Agent 1's step 0?? No)
    // Agent 1 starts at (1,0). Agent 0's step 1 is (1,0).
    // so Agent 0 at step 1 (index 1) needs Agent 1 to leave (1,0) (index 0).
    // So (0, 1) depends on (1, 0).

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
            Arc::new(parry2d::shape::Ball::new(0.49)),
            Arc::new(parry2d::shape::Ball::new(0.49)),
        ],
        discretization_timestep: 1.0,
    };

    let semantic_plan = mapf_post(&mapf_result);

    // Case 1: Safe state
    // Agent 0 at 0, Agent 1 at 0.
    // Agent 0 at (0,0). Agent 1 at (1,0). No conflict.
    let safe_state = vec![
        SemanticWaypoint { agent: 0, trajectory_index: 0 },
        SemanticWaypoint { agent: 1, trajectory_index: 0 },
    ];
    assert!(!semantic_plan.check_for_violation(&safe_state));

    // Case 2: Violation
    // Agent 0 at 1 (moves to (1,0)). Agent 1 at 0 (at (1,0)).
    // Agent 0 collides with Agent 1.
    // This should be a violation.
    let violation_state = vec![
        SemanticWaypoint { agent: 0, trajectory_index: 1 },
        SemanticWaypoint { agent: 1, trajectory_index: 0 },
    ];
    // Note: check_for_violation is expected to return TRUE if there is a violation.
    // However, I haven't implemented it yet to return bool. I will do that.
    // So this test will fail to compile until I modify lib.rs.
    // But I will run it after modifying lib.rs.
    assert!(semantic_plan.check_for_violation(&violation_state));
    
    // Case 3: Safe again
    // Agent 0 at 1. Agent 1 at 1 (moves to (2,0)).
    // Agent 1 has left (1,0). Agent 0 is at (1,0). Safe.
    let safe_state_2 = vec![
        SemanticWaypoint { agent: 0, trajectory_index: 1 },
        SemanticWaypoint { agent: 1, trajectory_index: 1 },
    ];
    assert!(!semantic_plan.check_for_violation(&safe_state_2));
}
