use crate::*;
use parry2d::na::Vector2;

/// Build a `MapfResult` from per-agent `(x, y)` waypoint lists. Every agent
/// gets a ball footprint of radius-`r` and the timestep is 1.0.
fn mapf_result_from(paths: &[Vec<(f32, f32)>], r: f32) -> MapfResult {
    use parry2d::shape::Ball;
    MapfResult {
        trajectories: paths
            .iter()
            .map(|path| Trajectory {
                poses: path
                    .iter()
                    .map(|&(x, y)| Isometry2::new(Vector2::new(x, y), 0.0))
                    .collect(),
            })
            .collect(),
        footprints: paths
            .iter()
            .map(|_| Arc::new(Ball::new(r)) as Arc<dyn Shape>)
            .collect(),
        discretization_timestep: 1.0,
    }
}

/// Collect the cross-agent dependency edges of a plan
/// as `(predecessor, successor)` pairs.
fn type2_edges_of(
    plan: &SemanticPlan,
) -> std::collections::HashSet<(SemanticWaypoint, SemanticWaypoint)> {
    let mut edges = std::collections::HashSet::new();
    for successor in &plan.waypoints {
        let Some(predecessor_ids) = plan.comes_before(successor) else {
            continue;
        };
        for &pred_id in predecessor_ids {
            let predecessor = plan.waypoints[pred_id];
            if predecessor.agent != successor.agent {
                edges.insert((predecessor, *successor));
            }
        }
    }
    edges
}

/// The brute-force O((NT)^2) collision-dependency algorithm.
fn brute_force_type2_edges(
    mapf_result: &MapfResult,
) -> std::collections::HashSet<(SemanticWaypoint, SemanticWaypoint)> {
    let mut edges = std::collections::HashSet::new();
    let trajectories = &mapf_result.trajectories;
    for agent1 in 0..trajectories.len() {
        for ti1 in 1..trajectories[agent1].len() {
            for agent2 in 0..trajectories.len() {
                if agent1 == agent2 {
                    continue;
                }
                for ti2 in ti1 + 1..trajectories[agent2].len() {
                    if collides(
                        &trajectories[agent1].poses[ti1 - 1],
                        &trajectories[agent1].poses[ti1],
                        &*mapf_result.footprints[agent1],
                        &trajectories[agent2].poses[ti2 - 1],
                        &trajectories[agent2].poses[ti2],
                        &*mapf_result.footprints[agent2],
                        mapf_result.discretization_timestep,
                    ) {
                        edges.insert((
                            SemanticWaypoint {
                                agent: agent1,
                                trajectory_index: ti1 - 1,
                            },
                            SemanticWaypoint {
                                agent: agent2,
                                trajectory_index: ti2 - 1,
                            },
                        ));
                    }
                }
            }
        }
    }
    edges
}

/// For ball footprints, straight-line motion scenarios the sweep-line
/// must produce the identical dependency graph as the brute-force
/// implementation: the merged endpoint AABB contains each segment's swept
/// volume, so AABB-disjoint segments cannot collide, and both algorithms
/// then apply the same exact collision check.
#[test]
fn test_sweep_line_matches_bruteforce() {
    let scenarios: Vec<(&str, Vec<Vec<(f32, f32)>>)> = vec![
        (
            "horizontal_cross",
            vec![
                vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (2.0, 0.0)],
                vec![(1.0, -1.0), (1.0, -1.0), (1.0, 0.0), (1.0, 1.0)],
            ],
        ),
        (
            "horizontal_follow",
            vec![
                vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
                vec![(1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0)],
            ],
        ),
        (
            "vertical_follow",
            vec![
                vec![(0.0, 0.0), (0.0, 1.0), (0.0, 2.0), (0.0, 3.0)],
                vec![(0.0, 1.0), (0.0, 2.0), (0.0, 3.0), (0.0, 4.0)],
            ],
        ),
        (
            "head_on_swap",
            vec![
                vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)],
                vec![(2.0, 0.0), (1.0, 0.0), (0.0, 0.0)],
            ],
        ),
        (
            "diagonal_cross",
            vec![
                vec![(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)],
                vec![(0.0, 2.0), (1.0, 1.0), (2.0, 0.0)],
            ],
        ),
        (
            "parallel_far_apart",
            vec![
                vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)],
                vec![(0.0, 50.0), (1.0, 50.0), (2.0, 50.0)],
            ],
        ),
        (
            "three_agents",
            vec![
                vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
                vec![(1.5, -1.5), (1.5, 0.0), (1.5, 1.5), (1.5, 1.5)],
                vec![(3.0, 0.0), (2.0, 0.0), (1.0, 0.0), (0.0, 0.0)],
            ],
        ),
        (
            "unequal_lengths_crossing",
            vec![
                vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)],
                vec![(1.0, -2.0), (1.0, -1.0), (1.0, 0.0), (1.0, 1.0), (1.0, 2.0)],
            ],
        ),
    ];

    for (name, paths) in scenarios {
        let mapf_result = mapf_result_from(&paths, 0.49);
        let actual = type2_edges_of(&mapf_post(&mapf_result));
        let expected = brute_force_type2_edges(&mapf_result);
        assert_eq!(
            actual, expected,
            "sweep-line and brute-force disagree on scenario '{name}'"
        );
    }
}

/// A scene and its 90-degree rotation must yield identical edges,
/// since ball footprints make collisions rotation-invariant.
/// Tests that the sort-by-Y branch agrees with the sort-by-X branch.
#[test]
fn test_sort_axis_invariance() {
    // Spread is larger along X -> sweep sorts by X.
    let horizontal = mapf_result_from(
        &[
            vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
            vec![(1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0)],
        ],
        0.49,
    );
    // Same agent/index structure rotated 90 degrees onto Y -> sweep sorts by Y.
    let vertical = mapf_result_from(
        &[
            vec![(0.0, 0.0), (0.0, 1.0), (0.0, 2.0), (0.0, 3.0)],
            vec![(0.0, 1.0), (0.0, 2.0), (0.0, 3.0), (0.0, 4.0)],
        ],
        0.49,
    );

    let horizontal_edges = type2_edges_of(&mapf_post(&horizontal));
    let vertical_edges = type2_edges_of(&mapf_post(&vertical));

    assert!(
        !horizontal_edges.is_empty(),
        "overlapping follow paths must produce collision dependencies"
    );
    assert_eq!(
        horizontal_edges, vertical_edges,
        "sort-by-X and sort-by-Y branches must produce identical edges"
    );
}
