use std::collections::HashMap;
use std::sync::Arc;

use parry2d::na::{Isometry2, Point2};
use parry2d::query::{NonlinearRigidMotion, ShapeCastStatus};
use parry2d::shape::Shape;
use parry2d::query::cast_shapes_nonlinear;

#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub struct SemanticWaypoint {
    pub agent: usize,
    pub trajectory_index: usize
}

#[derive(Clone, Debug, Default)]
pub struct SemanticPlan {
    /// These serve as graph nodes, describing each waypoint
    pub waypoints: Vec<SemanticWaypoint>,
    /// For book keeping
    agent_time_to_wp_id: HashMap<SemanticWaypoint, usize>,
    /// Say Key comes after all of Values
    comes_after: HashMap<usize, Vec<usize>>,
}

impl SemanticPlan {
    fn add_waypoint(&mut self, waypoint: &SemanticWaypoint) {
        self.agent_time_to_wp_id.insert(*waypoint, self.waypoints.len());
        self.waypoints.push(*waypoint);
    }

    fn requires_comes_after(&mut self, after: &SemanticWaypoint, before: &SemanticWaypoint) {
        let Some(after_id) = self.agent_time_to_wp_id.get(before) else {
            return;
        };
        let Some(before_id) = self.agent_time_to_wp_id.get(after) else {
            return;
        };
        
        let Some(to_vals) = self.comes_after.get_mut(&after_id) else {
            self.comes_after.insert(*after_id, vec![*before_id]);
            return;
        };
        to_vals.push(*before_id);
    }

    /// Check which waypoints should come before the given waypoint
    fn comes_before(&self, waypoint: &SemanticWaypoint) -> Option<&Vec<usize>> {
        let Some(waypoint_id) = self.agent_time_to_wp_id.get(waypoint) else {
            return None;
        };
        self.comes_after.get(waypoint_id)
    }

    /// Generate a DOT representation of the graph for visualization
    pub fn to_dot(&self) -> String {
        let mut dot = String::from("digraph SemanticPlan {\n");
        for (id, waypoint) in self.waypoints.iter().enumerate() {
            dot.push_str(&format!("    {} [label=\"Agent: {}, Index: {}\"];\n", id, waypoint.agent, waypoint.trajectory_index));
        }
        for (after, befores) in &self.comes_after {
            for before in befores {
                dot.push_str(&format!("    {} -> {};\n", before, after));
            }
        }
        dot.push_str("}\n");
        dot
    }
}


pub struct Trajectory {
    pub poses: Vec<Isometry2<f32>>
}

impl Trajectory {
    pub fn len(&self) ->usize {
        self.poses.len()
    }
}


/// Input of time discretized MAPF result
/// The trajectories are time discretized, and the
/// footprints are the shapes of the agents.
/// We expect all trajectories to be of the same length.
pub struct MapfResult {
    /// The trajectories of the agents
    pub trajectories: Vec<Trajectory>,
    /// The shapes of the agents
    pub footprints: Vec<Arc<dyn Shape>>,
    /// The time discretization of the trajectories
    pub discretization_timestep: f32
}

fn calculate_nonlinear_rigid_motion(
    isometry1: &Isometry2<f32>,
    isometry2: &Isometry2<f32>,
    delta_time: f32, // Time elapsed between the two isometries
    local_center: Point2<f32>, // Local center of rotation
) -> NonlinearRigidMotion {
    // 1. Calculate Linear Velocity:
    let linear_velocity = (isometry2.translation.vector - isometry1.translation.vector) / delta_time;

    // 2. Calculate Angular Velocity:
    //    This is more complex and involves extracting the rotation difference.
    let rotation1 = isometry1.rotation;
    let rotation2 = isometry2.rotation;

    // Calculate the relative rotation (rotation that transforms rotation1 to rotation2)
    let relative_rotation = rotation2 * rotation1.inverse();

    // Convert the relative rotation to an axis-angle representation
    let angle = relative_rotation.angle();

    // Angular velocity is the axis scaled by the angle and divided by the time step.
    let angular_velocity = angle / delta_time;

    NonlinearRigidMotion {
        start: *isometry1, // Or *isometry2, depending on your starting point
        local_center,
        linvel: linear_velocity,
        angvel: angular_velocity,
    }
}

/// Mock for now
fn collides(ti1: &Isometry2<f32>, ti2: &Isometry2<f32>, shape_i: &dyn Shape,
    tj1: &Isometry2<f32>, tj2: &Isometry2<f32>, shape_j: &dyn Shape,
    delta_time: f32) -> bool
{
    let motion_i = calculate_nonlinear_rigid_motion(ti1, ti2, delta_time, Point2::origin());
    let motion_j = calculate_nonlinear_rigid_motion(tj1, tj2, delta_time, Point2::origin());
    let time_of_impact = cast_shapes_nonlinear(&motion_i, shape_i, &motion_j, shape_j, 0.0, delta_time, true);
    if let Ok(Some(toi)) = time_of_impact {
        // Check if the time of impact is within the delta_time
        println!("Time of impact: {}", toi.time_of_impact);
        println!("Agent 1: {:?}", (ti1, ti2));
        println!("Agent 2: {:?}", (tj2, tj2));
        println!("Shape Cast: {:?}", toi);
        toi.status == ShapeCastStatus::Converged &&
        toi.time_of_impact <= delta_time || toi.status == ShapeCastStatus::PenetratingOrWithinTargetDist
    } else {
        false
    }
}

/// Simple implementation of mapf_post: A MapF -> semantic plan
/// Method.
/// TODO(arjoc): This can be improved.
///
/// Based on https://whoenig.github.io/publications/2019_RA-L_Hoenig.pdf
pub fn mapf_post(mapf_result: MapfResult) -> SemanticPlan {
    
    let mut semantic_plan = SemanticPlan::default();
    // Type 1 edges
    for agent in 0..mapf_result.trajectories.len() {
        for trajectory_index in 0..mapf_result.trajectories[agent].len() {
            semantic_plan.add_waypoint(&SemanticWaypoint {agent, trajectory_index});

            if trajectory_index < 1 {
                continue;
            }
            semantic_plan.comes_after.insert(semantic_plan.waypoints.len()-1, vec![semantic_plan.waypoints.len()-2]);
        }
    }

    // Type 2 edges
    for agent1 in 0..mapf_result.trajectories.len() {
        for trajectory_index1 in 1..mapf_result.trajectories[agent1].len() {
            for agent2 in 0..mapf_result.trajectories.len() {
                if agent1 == agent2 {
                    continue;
                }
                for trajectory_index2 in trajectory_index1+1..mapf_result.trajectories[agent2].len() {
                    if collides(&mapf_result.trajectories[agent1].poses[trajectory_index1 - 1], &mapf_result.trajectories[agent1].poses[trajectory_index1], &*mapf_result.footprints[agent1],
                        &mapf_result.trajectories[agent2].poses[trajectory_index2 - 1], &mapf_result.trajectories[agent2].poses[trajectory_index2], &*mapf_result.footprints[agent2],
                    mapf_result.discretization_timestep)
                    {
                        println!("Collision detected between agent {} at index {} and agent {} at index {}", agent1, trajectory_index1, agent2, trajectory_index2);
                        semantic_plan.requires_comes_after(&SemanticWaypoint{agent: agent1, trajectory_index: trajectory_index1}, &SemanticWaypoint{agent: agent2, trajectory_index: trajectory_index2});
                    }
                }
            }
        }
    }
    semantic_plan
}

// ---
// Unit Tests
// ---

#[cfg(test)]
mod tests {
    use parry2d::{na::Vector2, utils::hashset::HashSet};

    use super::*;
    use std::collections::HashMap; // Make sure HashMap is in scope

    #[test]
    fn test_add_waypoint() {
        let mut plan = SemanticPlan::default();
        let wp1 = SemanticWaypoint { agent: 0, trajectory_index: 0 };
        let wp2 = SemanticWaypoint { agent: 0, trajectory_index: 1 };
        let wp3 = SemanticWaypoint { agent: 1, trajectory_index: 0 };

        plan.add_waypoint(&wp1);
        assert_eq!(plan.waypoints.len(), 1);
        assert_eq!(plan.waypoints[0], wp1);
        assert_eq!(plan.agent_time_to_wp_id.get(&wp1), Some(&0));

        plan.add_waypoint(&wp2);
        assert_eq!(plan.waypoints.len(), 2);
        assert_eq!(plan.waypoints[1], wp2);
        assert_eq!(plan.agent_time_to_wp_id.get(&wp2), Some(&1));

        plan.add_waypoint(&wp3);
        assert_eq!(plan.waypoints.len(), 3);
        assert_eq!(plan.waypoints[2], wp3);
        assert_eq!(plan.agent_time_to_wp_id.get(&wp3), Some(&2));

        // Adding an existing waypoint (by value) should still add it again with a new ID
        // This behavior is derived from how `add_waypoint` is implemented.
        let wp1_again = SemanticWaypoint { agent: 0, trajectory_index: 0 };
        plan.add_waypoint(&wp1_again);
        assert_eq!(plan.waypoints.len(), 4);
        assert_eq!(plan.waypoints[3], wp1_again);
        assert_eq!(plan.agent_time_to_wp_id.get(&wp1_again), Some(&3)); // Now maps to the new ID
    }

    #[test]
    fn test_add_comes_after_basic() {
        let mut plan = SemanticPlan::default();
        let wp0_0 = SemanticWaypoint { agent: 0, trajectory_index: 0 }; // ID 0
        let wp0_1 = SemanticWaypoint { agent: 0, trajectory_index: 1 }; // ID 1
        let wp1_0 = SemanticWaypoint { agent: 1, trajectory_index: 0 }; // ID 2

        plan.add_waypoint(&wp0_0);
        plan.add_waypoint(&wp0_1);
        plan.add_waypoint(&wp1_0);

        // wp1_0 (ID 2) comes after wp0_0 (ID 0)
        plan.requires_comes_after(&wp0_0, &wp1_0);
        let mut expected_comes_after: HashMap<usize, Vec<usize>> = HashMap::new();
        expected_comes_after.insert(2, vec![0]);
        assert_eq!(plan.comes_after, expected_comes_after);

        // wp1_0 (ID 2) also comes after wp0_1 (ID 1)
        plan.requires_comes_after(&wp0_1, &wp1_0);
        let mut expected_comes_after_2: HashMap<usize, Vec<usize>> = HashMap::new();
        expected_comes_after_2.insert(2, vec![0, 1]); 
        
        // Retrieve the actual vector and sort it for consistent comparison
        let mut actual_vec = plan.comes_after.get(&2).unwrap().clone();
        actual_vec.sort_unstable();
        
        // Retrieve the expected vector and sort it
        let mut expected_vec = expected_comes_after_2.get(&2).unwrap().clone();
        expected_vec.sort_unstable();

        assert_eq!(actual_vec, expected_vec);
    }

    #[test]
    fn test_add_comes_after_non_existent_waypoints() {
        let mut plan = SemanticPlan::default();
        let wp_a = SemanticWaypoint { agent: 0, trajectory_index: 0 };
        let wp_b = SemanticWaypoint { agent: 0, trajectory_index: 1 };
        let wp_c = SemanticWaypoint { agent: 0, trajectory_index: 2 };

        plan.add_waypoint(&wp_a); // ID 0

        // Attempt to add a dependency where 'before' waypoint (wp_b) is not in the plan
        plan.requires_comes_after(&wp_b, &wp_a); 
        assert!(plan.comes_after.is_empty());

        // Attempt to add a dependency where 'after' waypoint (wp_c) is not in the plan
        plan.requires_comes_after(&wp_a, &wp_c); 
        assert!(plan.comes_after.is_empty());

        // Add wp_c to the plan
        plan.add_waypoint(&wp_c); // ID 1

        // Now, this dependency should be added successfully: wp_c (ID 1) comes after wp_a (ID 0)
        plan.requires_comes_after(&wp_a, &wp_c); 
        let mut expected_comes_after: HashMap<usize, Vec<usize>> = HashMap::new();
        expected_comes_after.insert(1, vec![0]);
        assert_eq!(plan.comes_after, expected_comes_after);
    }

    #[test]
    fn test_add_comes_after_self_dependency() {
        let mut plan = SemanticPlan::default();
        let wp1 = SemanticWaypoint { agent: 0, trajectory_index: 0 };

        plan.add_waypoint(&wp1); // ID 0

        // Test self-dependency: wp1 (ID 0) comes after wp1 (ID 0)
        plan.requires_comes_after(&wp1, &wp1);
        let mut expected_comes_after: HashMap<usize, Vec<usize>> = HashMap::new();
        expected_comes_after.insert(0, vec![0]);
        assert_eq!(plan.comes_after, expected_comes_after);
    }

    #[test]
    fn test_collision_detection() {
        let isometry1 = Isometry2::new(Vector2::new(0.0, 0.0), 0.0);
        let isometry2 = Isometry2::new(Vector2::new(1.0, 1.0), 0.0);
        let shape1 = Arc::new(parry2d::shape::Ball::new(0.5));
        let shape2 = Arc::new(parry2d::shape::Ball::new(0.5));
        let delta_time = 1.0;

        assert!(collides(&isometry1, &isometry2, &*shape1, &isometry1, &isometry2, &*shape2, delta_time));
    }

    #[test]
    fn test_post_processing() {
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

        // Generate the semantic plan
        let semantic_plan = mapf_post(mapf_result);
        // Check the number of waypoints
        assert_eq!(semantic_plan.waypoints.len(), 8); // 4 for each agent
        let res = semantic_plan.comes_before(&SemanticWaypoint { agent: 1, trajectory_index: 2 }).unwrap();
        assert_eq!(res.len(), 2); // 2 waypoints should come before this one
        let t1  = semantic_plan.waypoints[res[0]];
        let t2  = semantic_plan.waypoints[res[1]];

        let desired = HashSet::from_iter(vec![t1, t2]);
        assert!(desired.contains(&SemanticWaypoint { agent: 0, trajectory_index: 1 }));
        assert!(desired.contains(&SemanticWaypoint { agent: 1, trajectory_index: 1 }));
    }
}
