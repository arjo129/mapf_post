use axum::{Json, Router, extract::State, routing::post};
use clap::Parser;
use core::alloc;
use csv;
use mapf_post::spatial_allocation::{CurrentPosition, Grid2D};
use rand::seq::IndexedRandom;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::error::Error;
use std::net::SocketAddr;
use std::sync::{Arc, Mutex};

use axum::serve;
use mapf_post::na::{Isometry2, Vector2};
use mapf_post::{
    MapfResult, SemanticPlan, SemanticWaypoint, Trajectory, WaypointFollower, mapf_post,
};
use parry2d::shape::Shape;
use tokio::net::TcpListener;

#[derive(Parser)]
#[clap(version = "1.0", author = "Arjo Chakravarty")]
struct Cli {
    #[clap(short, long)]
    path: Option<String>,
}

// --- API Request/Response Structs ---
#[derive(Debug, Deserialize)]
pub struct AgentPoseRequest {
    pub agent_id: usize,
    pub x: f32,
    pub y: f32,
    pub angle: f32, // Added angle for Isometry2
}

#[derive(Debug, Serialize)]
pub struct AgentAllocationResponse {
    pub agent_id: usize,
    pub cell_size: f32,
    pub allocated_free_space: Vec<(f32, f32)>,
    pub next_goal: (f32, f32),
    pub remaining_traj: Vec<(f32, f32)>,
}

// --- Server State ---
#[derive(Clone)]
pub struct AppState {
    pub mapf_result: Arc<MapfResult>,
    pub semantic_plan: Arc<SemanticPlan>,
    pub grid: Arc<Grid2D>,
    pub waypoint_followers: Arc<Mutex<HashMap<usize, WaypointFollower>>>,
    pub current_wp: Arc<Mutex<Vec<CurrentPosition>>>,
}

#[tokio::main]
async fn main() {
    let cli = Cli::parse();
    // --- Initialization ---
    let mapf_result = Arc::new(match cli.path {
        Some(path) => match load_trajectories_from_csv(&path) {
            Ok(result) => {
                println!("Loaded trajectories from CSV: {}", path);
                result
            }
            Err(e) => {
                eprintln!(
                    "Error loading CSV from {}: {}. Using default trajectories.",
                    path, e
                );
                generate_default_mapf_result()
            }
        },
        None => {
            println!("No CSV path provided. Using default trajectories.");
            generate_default_mapf_result()
        }
    });

    let semantic_plan = Arc::new(mapf_post(&mapf_result));

    let mut followers = HashMap::new();
    let mut current_wp = vec![];
    for (i, trajectory) in mapf_result.trajectories.iter().enumerate() {
        followers.insert(i, WaypointFollower::from_trajectory(i, trajectory.clone()));
        current_wp.push(CurrentPosition {
            semantic_position: SemanticWaypoint {
                agent: i,
                trajectory_index: 0,
            },
            real_position: (
                trajectory.poses[0].translation.x,
                trajectory.poses[0].translation.y,
            ),
        });
    }

    let app_state = Arc::new(AppState {
        mapf_result: mapf_result.clone(),
        semantic_plan: semantic_plan.clone(),
        grid: Arc::new(Grid2D::new(vec![vec![0; 20]; 20], 1.0)), //TODO load  from occupancy grid
        waypoint_followers: Arc::new(Mutex::new(followers)),
        current_wp: Arc::new(Mutex::new(current_wp)),
    });

    // --- Axum Router ---
    let app = Router::new()
        .route("/update_pose", post(update_pose))
        .with_state(app_state);

    let addr = SocketAddr::from(([127, 0, 0, 1], 3000));
    let listener = TcpListener::bind(addr).await.unwrap();
    println!("listening on {}", addr);
    serve(listener, app.into_make_service()).await.unwrap();
}

// --- Handler Function ---
async fn update_pose(
    State(app_state): State<Arc<AppState>>,
    Json(payload): Json<AgentPoseRequest>,
) -> Json<AgentAllocationResponse> {
    let mut followers = app_state.waypoint_followers.lock().unwrap();

    let agent_id = payload.agent_id;
    let uncertainty = 0.5; // Example uncertainty value

    println!("Received pose {:?}", payload);

    // 1. Update the waypoint follower for the specific agent
    let current_pose = Isometry2::new(Vector2::new(payload.x, payload.y), payload.angle);
    let Some(follower) = followers.get_mut(&agent_id) else {
        return Json(AgentAllocationResponse {
            agent_id,
            cell_size: 1.0,
            allocated_free_space: vec![],
            next_goal: (payload.x, payload.y),
            remaining_traj: vec![],
        });
    };
    follower.update_position_estimate(&current_pose, uncertainty);

    // Update internal position state
    let mut current_pos = app_state.current_wp.lock().unwrap();
    current_pos[agent_id] = CurrentPosition {
        semantic_position: follower.get_semantic_waypoint(),
        real_position: (payload.x, payload.y),
    };

    let mut v_current_pos = vec![];
    for id in 0..current_pos.len() {
        v_current_pos.push(current_pos[id].semantic_position.clone());
    }
    let safe_claims = app_state.semantic_plan.get_claim_dict(&v_current_pos);

    // TODO(arjoc)
    let allocation_field = app_state
        .grid
        .allocate_trajectory(&app_state.mapf_result, &current_pos);
    if let Some(cells) = allocation_field.get_alloc_for_agent(agent_id) {
        let mut traj = vec![(payload.x, payload.y)];
        traj.extend(
            follower.remaining_safe_trajectory_segment(safe_claims.get(&agent_id).unwrap()),
        );
        let p = Json(AgentAllocationResponse {
            agent_id,
            cell_size: 1.0,
            allocated_free_space: cells
                .iter()
                .map(|&(x, y)| app_state.grid.to_world_coords(x as isize, y as isize))
                .collect(),
            next_goal: (
                follower.next_waypoint().translation.x,
                follower.next_waypoint().translation.y,
            ),
            remaining_traj: traj,
        });
        println!("{:?}", p);
        p
    } else {
        Json(AgentAllocationResponse {
            agent_id,
            cell_size: 0.0,
            allocated_free_space: vec![],
            next_goal: (payload.x, payload.y),
            remaining_traj: vec![],
        })
    }
}

// --- Helper Functions from leader_follower_vis.rs ---
fn load_trajectories_from_csv(path: &str) -> Result<MapfResult, Box<dyn Error>> {
    use parry2d::shape::Shape;
    let mut reader = csv::Reader::from_path(path)?;
    let headers = reader.headers()?.clone();
    let num_agents = headers.len() / 2;

    let mut trajectories: Vec<Trajectory> = (0..num_agents)
        .map(|_| Trajectory { poses: vec![] })
        .collect();

    for result in reader.records() {
        let record = result?;
        for i in 0..num_agents {
            let x: f64 = record[i * 2].parse()?;
            let y: f64 = record[i * 2 + 1].parse()?;
            trajectories[i]
                .poses
                .push(Isometry2::new(Vector2::new(x as f32, y as f32), 0.0));
        }
    }

    Ok(MapfResult {
        trajectories,
        footprints: (0..num_agents)
            .map(|_| Arc::new(parry2d::shape::Ball::new(0.49)) as Arc<dyn Shape>)
            .collect(),
        discretization_timestep: 1.0,
    })
}

// --- Helper Functions from leader_follower_vis.rs ---
fn generate_default_mapf_result() -> MapfResult {
    MapfResult {
        trajectories: vec![
            Trajectory {
                poses: vec![
                    Isometry2::new(Vector2::new(0.0, 5.0), 0.0),
                    Isometry2::new(Vector2::new(1.0, 5.0), 0.0),
                    Isometry2::new(Vector2::new(2.0, 5.0), 0.0),
                    Isometry2::new(Vector2::new(3.0, 5.0), 0.0),
                    Isometry2::new(Vector2::new(4.0, 5.0), 0.0),
                    Isometry2::new(Vector2::new(5.0, 5.0), 0.0),
                ],
            },
            Trajectory {
                poses: vec![
                    Isometry2::new(Vector2::new(3.0, 5.0), 0.0),
                    Isometry2::new(Vector2::new(4.0, 5.0), 0.0),
                    Isometry2::new(Vector2::new(5.0, 5.0), 0.0),
                    Isometry2::new(Vector2::new(6.0, 5.0), 0.0),
                    Isometry2::new(Vector2::new(7.0, 5.0), 0.0),
                    Isometry2::new(Vector2::new(8.0, 5.0), 0.0),
                ],
            },
        ],
        footprints: (0..2)
            .map(|_| Arc::new(parry2d::shape::Ball::new(0.49)) as Arc<dyn Shape>)
            .collect(),
        discretization_timestep: 1.0,
    }
}
