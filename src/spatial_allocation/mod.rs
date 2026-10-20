use std::collections::{HashMap, HashSet, VecDeque};

use bresenham::Bresenham;

use crate::{
    IntersectionType, LeaderFollowerZones, MapfResult, SemanticPlan, SemanticWaypoint, mapf_post,
};

#[derive(Clone)]
pub struct CurrentPosition {
    pub semantic_position: SemanticWaypoint,
    pub real_position: (f32, f32),
}
#[derive(Clone)]
pub struct Grid2D {
    static_obstacles: Vec<Vec<i8>>,
    cell_size: f32,
    width: usize,
    height: usize,
    blur_radius: f32,
}

impl Grid2D {
    pub fn new(static_obstacles: Vec<Vec<i8>>, cell_size: f32) -> Self {
        let width = static_obstacles.len();
        let height = if width > 0 {
            static_obstacles[0].len()
        } else {
            0
        };
        Self {
            static_obstacles,
            cell_size,
            width,
            height,
            blur_radius: 1.0,
        }
    }

    pub fn with_blur_radius(mut self, blur_radius: f32) -> Self {
        self.blur_radius = blur_radius.max(0.0);
        self
    }

    pub fn blur_radius(&self) -> f32 {
        self.blur_radius
    }

    pub fn to_world_coords(&self, x: isize, y: isize) -> (f32, f32) {
        (
            self.cell_size * x as f32,
            self.cell_size * (self.height as isize - 1 - y) as f32,
        )
    }

    pub fn from_world_coords(&self, x: f32, y: f32) -> (isize, isize) {
        (
            (x / self.cell_size).floor() as isize,
            (self.height as isize - 1 - (y / self.cell_size).floor() as isize),
        )
    }

    pub fn allocate_trajectory(
        &self,
        trajectories: &MapfResult,
        positions: &[CurrentPosition],
    ) -> AllocationField {
        let semantic_plan = mapf_post(trajectories);
        let semantic_positions: Vec<SemanticWaypoint> =
            positions.iter().map(|p| p.semantic_position).collect();
        let assignment = semantic_plan.get_claim_dict(&semantic_positions);

        let mut allocation_field =
            AllocationField::create(semantic_plan, self.width.max(1000), self.height.max(1000));

        for (agent, traj) in trajectories.trajectories.iter().enumerate() {
            let Some(region) = assignment.get(&agent) else {
                panic!("Unknown Agent - something went wrong in get_claim_dict");
            };
            for i in region.start_id..region.end_id + 1 {
                if i >= traj.poses.len() {
                    continue;
                }

                let start_pose = traj.poses[i];
                let start_coords =
                    self.from_world_coords(start_pose.translation.x, start_pose.translation.y);

                if i == region.start_id
                    && let Some(pos) = positions.get(agent)
                {
                    let real_coords =
                        self.from_world_coords(pos.real_position.0, pos.real_position.1);
                    for (rx, ry) in Bresenham::new(real_coords, start_coords) {
                        self.blur(
                            &TrajectoryAllocation {
                                agent,
                                priority: i,
                                dist_from_center: 0.0,
                            },
                            &(rx, ry),
                            &mut allocation_field,
                            self.blur_radius,
                        );
                    }
                }

                self.blur(
                    &TrajectoryAllocation {
                        agent,
                        priority: i,
                        dist_from_center: 0.0,
                    },
                    &start_coords,
                    &mut allocation_field,
                    self.blur_radius,
                );

                if i + 1 >= traj.poses.len() {
                    continue;
                }

                let end_pose = traj.poses[i + 1];
                let end_coords =
                    self.from_world_coords(end_pose.translation.x, end_pose.translation.y);

                for (x, y) in Bresenham::new(start_coords, end_coords) {
                    self.blur(
                        &TrajectoryAllocation {
                            agent,
                            priority: i,
                            dist_from_center: 0.0,
                        },
                        &(x, y),
                        &mut allocation_field,
                        self.blur_radius,
                    );
                }
                self.blur(
                    &TrajectoryAllocation {
                        agent,
                        priority: i,
                        dist_from_center: 0.0,
                    },
                    &end_coords,
                    &mut allocation_field,
                    self.blur_radius,
                );
            }
        }
        allocation_field
    }

    fn blur(
        &self,
        trajectory_alloc: &TrajectoryAllocation,
        start_cell: &(isize, isize),
        grid_space: &mut AllocationField,
        max_dist: f32,
    ) {
        let mut queue = VecDeque::new();
        queue.push_back((*start_cell, 0.0f32));
        let mut visited = HashSet::new();
        if start_cell.0 >= 0 && start_cell.1 >= 0 {
            visited.insert((start_cell.0 as usize, start_cell.1 as usize));
        }
        grid_space.mark(start_cell.0, start_cell.1, trajectory_alloc);
        while let Some(((x, y), dist)) = queue.pop_front() {
            if dist + 1.0 > max_dist {
                continue;
            }
            for i in -1..=1 {
                for j in -1..=1 {
                    if x + i < 0 || y + j < 0 {
                        continue;
                    }
                    let nx = (x + i) as usize;
                    let ny = (y + j) as usize;

                    if visited.contains(&(nx, ny)) {
                        continue;
                    }

                    if nx >= self.static_obstacles.len() {
                        continue;
                    }

                    if ny >= self.static_obstacles[nx].len() {
                        continue;
                    }

                    if self.static_obstacles[nx][ny] == 100 {
                        continue;
                    }

                    visited.insert((nx, ny));
                    queue.push_back(((nx as isize, ny as isize), dist + 1.0));
                    let alloc = TrajectoryAllocation {
                        agent: trajectory_alloc.agent,
                        priority: trajectory_alloc.priority,
                        dist_from_center: dist + 1.0,
                    };
                    grid_space.mark(nx as isize, ny as isize, &alloc);
                }
            }
        }
    }
}

#[derive(Clone, Debug)]
pub struct TrajectoryAllocation {
    agent: usize,
    priority: usize,
    dist_from_center: f32,
}

impl TrajectoryAllocation {
    fn to_wp(&self) -> SemanticWaypoint {
        SemanticWaypoint {
            agent: self.agent,
            trajectory_index: self.priority,
        }
    }
}

/// An allocation field. Each point on the grid is allocated to a specific robot
/// based on its original MAPF plan
pub struct AllocationField {
    grid_space: Vec<Vec<Option<TrajectoryAllocation>>>,
    plan: SemanticPlan,
    leader_follower: LeaderFollowerZones,
    cell_by_agent: HashMap<usize, HashSet<(usize, usize)>>,
}

impl AllocationField {
    fn create(plan: SemanticPlan, width: usize, height: usize) -> Self {
        let leader_follower = plan.figure_out_leader_follower_zone();
        Self {
            grid_space: vec![vec![None; height]; width],
            plan,
            leader_follower,
            cell_by_agent: HashMap::default(),
        }
    }

    fn width(&self) -> usize {
        self.grid_space.len()
    }

    fn height(&self) -> usize {
        if self.width() == 0 {
            0
        } else {
            self.grid_space[0].len()
        }
    }

    fn update_spot(&mut self, alloc: &TrajectoryAllocation, x: usize, y: usize) {
        if let Some(prev_alloc) = &self.grid_space[x][y]
            && let Some(p) = self.cell_by_agent.get_mut(&prev_alloc.agent)
        {
            p.remove(&(x, y));
        }
        self.grid_space[x][y] = Some(alloc.clone());
        if let Some(allocated_list) = self.cell_by_agent.get_mut(&alloc.agent) {
            allocated_list.insert((x, y));
        } else {
            self.cell_by_agent
                .insert(alloc.agent, HashSet::from_iter([(x, y)].iter().cloned()));
        }
    }

    fn mark(&mut self, x: isize, y: isize, alloc: &TrajectoryAllocation) {
        if x < 0 || y < 0 {
            return;
        }
        let x = x as usize;
        let y = y as usize;

        if x >= self.width() {
            return;
        }

        if y >= self.height() {
            return;
        }

        let Some(previous_alloc) = self.grid_space[x][y].clone() else {
            self.update_spot(alloc, x, y);
            return;
        };

        if previous_alloc.agent == alloc.agent {
            if alloc.dist_from_center < previous_alloc.dist_from_center
                || ((alloc.dist_from_center - previous_alloc.dist_from_center).abs() < 1e-5
                    && alloc.priority < previous_alloc.priority)
            {
                self.update_spot(alloc, x, y);
            }
            return;
        }

        // We keep reservations from earlier times first
        if previous_alloc.priority < alloc.priority {
            self.grid_space[x][y] = Some(previous_alloc.clone());
            return;
        }
        // In the event that a reservation is of the same priority/time step
        // we need to apply rules for allocation.
        // In the event the two agents have no relationship at that time step
        // partition the space by who ever is nearer.
        // In the event the relationship is a leader follower one give priority
        // to the leader.
        // In the event the relationship is an intersection based relationship,
        // give priority to the agent that is leaving the junction.
        else if previous_alloc.priority == alloc.priority {
            // Follower logic for tie-breaking.
            if let Some((_mode, priority, cluster_id)) = self
                .leader_follower
                .allocation_strategy
                .get(&previous_alloc.to_wp())
                && let Some((_mode2, priority2, cluster_id2)) =
                    self.leader_follower.allocation_strategy.get(&alloc.to_wp())
                && cluster_id == cluster_id2
            {
                if priority <= priority2 {
                    self.grid_space[x][y] = Some(previous_alloc.clone());
                    return;
                } else {
                    self.update_spot(alloc, x, y);
                    return;
                }
            }

            // Logic for intersections
            if let Some(intersection_type) = self
                .plan
                .is_intersection_participant(&previous_alloc.to_wp())
            {
                if let IntersectionType::Lead(followers) = intersection_type {
                    if followers.contains(&alloc.to_wp()) {
                        self.update_spot(alloc, x, y);
                        return;
                    }
                } else if let IntersectionType::Next(leaders) = intersection_type
                    && leaders.contains(&alloc.to_wp())
                {
                    self.update_spot(&previous_alloc, x, y);
                    return;
                }
            }
            // Vicinity rule.
            if previous_alloc.dist_from_center < alloc.dist_from_center {
                self.grid_space[x][y] = Some(previous_alloc.clone());
            } else {
                self.update_spot(alloc, x, y);
            }
        } else {
            self.update_spot(alloc, x, y);
        }
    }

    pub fn get_allocation(&self, x: isize, y: isize) -> Option<usize> {
        if x < 0 || y < 0 {
            return None;
        }
        let x = x as usize;
        let y = y as usize;

        if x >= self.width() || y >= self.height() {
            return None;
        }

        let Some(p) = self.grid_space[x][y].clone() else {
            return None;
        };
        Some(p.agent)
    }

    pub fn get_alloc_for_agent(&self, agent: usize) -> Option<Vec<(usize, usize)>> {
        self.cell_by_agent
            .get(&agent)
            .map(|p| Vec::from_iter(p.iter().cloned()))
    }

    pub fn get_alloc_priority(&self, x: isize, y: isize) -> Option<usize> {
        if x < 0 || y < 0 {
            return None;
        }
        let x = x as usize;
        let y = y as usize;

        if x >= self.width() || y >= self.height() {
            return None;
        }

        let Some(p) = self.grid_space[x][y].clone() else {
            return None;
        };
        Some(p.priority)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Trajectory;
    use crate::na::{Isometry2, Vector2};
    use std::sync::Arc;

    #[test]
    fn test_subdivided_grid_adjacent_lanes_do_not_overlap() {
        // Two 1.0m lanes at y = 1.5m and y = 2.5m on a 0.2m grid (subdiv = 5, blur_radius = 2.0 cells)
        let width = 50;
        let height = 50;
        let cell_size = 0.2;
        let grid = Grid2D::new(vec![vec![0; height]; width], cell_size).with_blur_radius(2.0);

        let mapf_result = MapfResult {
            trajectories: vec![
                Trajectory {
                    poses: vec![
                        Isometry2::new(Vector2::new(1.5, 1.5), 0.0),
                        Isometry2::new(Vector2::new(2.5, 1.5), 0.0),
                        Isometry2::new(Vector2::new(3.5, 1.5), 0.0),
                    ],
                },
                Trajectory {
                    poses: vec![
                        Isometry2::new(Vector2::new(3.5, 2.5), 0.0),
                        Isometry2::new(Vector2::new(2.5, 2.5), 0.0),
                        Isometry2::new(Vector2::new(1.5, 2.5), 0.0),
                    ],
                },
            ],
            footprints: vec![
                Arc::new(parry2d::shape::Ball::new(0.49)),
                Arc::new(parry2d::shape::Ball::new(0.49)),
            ],
            discretization_timestep: 1.0,
        };

        let positions = vec![
            CurrentPosition {
                semantic_position: SemanticWaypoint {
                    agent: 0,
                    trajectory_index: 0,
                },
                real_position: (1.5, 1.5),
            },
            CurrentPosition {
                semantic_position: SemanticWaypoint {
                    agent: 1,
                    trajectory_index: 0,
                },
                real_position: (3.5, 2.5),
            },
        ];

        let field = grid.allocate_trajectory(&mapf_result, &positions);

        // Check that every waypoint on lane 0 (y = 1.5) and its 5x5 subcell block belongs to agent 0
        for &wx in &[1.5, 2.5, 3.5] {
            let (cx, cy) = grid.from_world_coords(wx, 1.5);
            for dx in -2..=2 {
                for dy in -2..=2 {
                    assert_eq!(
                        field.get_allocation(cx + dx, cy + dy),
                        Some(0),
                        "Lane 0 subcell ({}, {}) around ({}, 1.5) should belong to agent 0",
                        cx + dx,
                        cy + dy,
                        wx
                    );
                }
            }
        }

        // Check that every waypoint on lane 1 (y = 2.5) and its 5x5 subcell block belongs to agent 1
        for &wx in &[1.5, 2.5, 3.5] {
            let (cx, cy) = grid.from_world_coords(wx, 2.5);
            for dx in -2..=2 {
                for dy in -2..=2 {
                    assert_eq!(
                        field.get_allocation(cx + dx, cy + dy),
                        Some(1),
                        "Lane 1 subcell ({}, {}) around ({}, 2.5) should belong to agent 1",
                        cx + dx,
                        cy + dy,
                        wx
                    );
                }
            }
        }
    }

    #[test]
    fn test_blocked_follower_does_not_claim_leader_cell() {
        let width = 50;
        let height = 50;
        let cell_size = 0.2;
        let grid = Grid2D::new(vec![vec![0; height]; width], cell_size).with_blur_radius(2.0);

        let mapf_result = MapfResult {
            trajectories: vec![
                Trajectory {
                    poses: vec![
                        Isometry2::new(Vector2::new(0.5, 1.5), 0.0),
                        Isometry2::new(Vector2::new(1.5, 1.5), 0.0),
                        Isometry2::new(Vector2::new(2.5, 1.5), 0.0),
                        Isometry2::new(Vector2::new(3.5, 1.5), 0.0),
                        Isometry2::new(Vector2::new(4.5, 1.5), 0.0),
                        Isometry2::new(Vector2::new(4.5, 1.5), 0.0),
                    ],
                },
                Trajectory {
                    poses: vec![
                        Isometry2::new(Vector2::new(1.5, 1.5), 0.0),
                        Isometry2::new(Vector2::new(2.5, 1.5), 0.0),
                        Isometry2::new(Vector2::new(3.5, 1.5), 0.0),
                        Isometry2::new(Vector2::new(4.5, 1.5), 0.0),
                        Isometry2::new(Vector2::new(5.5, 1.5), 0.0),
                        Isometry2::new(Vector2::new(5.5, 1.5), 0.0),
                    ],
                },
            ],
            footprints: vec![
                Arc::new(parry2d::shape::Ball::new(0.49)),
                Arc::new(parry2d::shape::Ball::new(0.49)),
            ],
            discretization_timestep: 1.0,
        };

        let positions = vec![
            CurrentPosition {
                semantic_position: SemanticWaypoint {
                    agent: 0,
                    trajectory_index: 3,
                },
                real_position: (3.5, 1.5),
            },
            CurrentPosition {
                semantic_position: SemanticWaypoint {
                    agent: 1,
                    trajectory_index: 3,
                },
                real_position: (4.5, 1.5),
            },
        ];

        let field = grid.allocate_trajectory(&mapf_result, &positions);

        let (c0x, c0y) = grid.from_world_coords(3.5, 1.5);
        let (c1x, c1y) = grid.from_world_coords(4.5, 1.5);
        assert_eq!(field.get_allocation(c0x, c0y), Some(0));
        assert_eq!(field.get_allocation(c1x, c1y), Some(1));
    }
}
