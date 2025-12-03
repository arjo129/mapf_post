use std::sync::Arc;
use std::error::Error;

use clap::Parser;
use mapf_post::na::{Isometry2, Vector2};
use mapf_post::spatial_allocation::{CurrentPosition, Grid2D};
use mapf_post::{MapfResult, Trajectory, SemanticWaypoint};
use macroquad::prelude::*;
use ::rand::Rng;

#[derive(Parser)]
#[clap(version = "1.0", author = "Arjo Chakravarty")]
struct Cli {
    #[clap(short, long)]
    path: Option<String>,
}
fn window_conf() -> Conf {
    Conf {
        window_title: "Leader Follower Visualization".to_owned(),
        fullscreen: false,
        ..Default::default()
    }
}

fn load_trajectories_from_csv(path: &str) -> Result<MapfResult, Box<dyn Error>> {
    use parry2d::shape::Shape;
    let mut reader = csv::Reader::from_path(path)?;
    let headers = reader.headers()?.clone();
    let num_agents = headers.len() / 2;

    let mut trajectories: Vec<Trajectory> = (0..num_agents).map(|_| Trajectory { poses: vec![] }).collect();

    for result in reader.records() {
        let record = result?;
        for i in 0..num_agents {
            let x: f64 = record[i * 2].parse()?;
            let y: f64 = record[i * 2 + 1].parse()?;
            trajectories[i].poses.push(Isometry2::new(Vector2::new(x as f32, y as f32), 0.0));
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
        footprints: vec![
            Arc::new(parry2d::shape::Ball::new(0.49)),
            Arc::new(parry2d::shape::Ball::new(0.49)),
        ],
        discretization_timestep: 1.0,
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let cli = Cli::parse();
    let mapf_result = match cli.path {
        Some(path) => match load_trajectories_from_csv(&path) {
            Ok(result) => result,
            Err(e) => {
                println!("Error loading CSV: {}", e);
                return;
            }
        },
        None => generate_default_mapf_result(),
    };

    let grid = Grid2D::new(vec![vec![0; 20]; 20], 1.0);

    let mut current_positions: Vec<CurrentPosition> = mapf_result
        .trajectories
        .iter()
        .enumerate()
        .map(|(i, trajectory)| CurrentPosition {
            semantic_position: SemanticWaypoint {
                agent: i,
                trajectory_index: 0,
            },
            real_position: (
                trajectory.poses[0].translation.x,
                trajectory.poses[0].translation.y,
            ),
        })
        .collect();

    let mut last_update = macroquad::time::get_time();
    let mut is_paused = false;

    loop {
        if is_key_pressed(KeyCode::Space) {
            is_paused = !is_paused;
        }
        clear_background(WHITE);

        let allocation_field = grid.allocate_trajectory(&mapf_result, &current_positions);
        let cell_size = 20.0;
        let padding = 2.0;
        let grid_height = 20.0;

        for i in 0..20 {
            for j in 0..20 {
                if let Some(agent) = allocation_field.get_allocation(i, j) {
                    let color = match agent {
                        0 => RED,
                        1 => BLUE,
                        _ => GRAY,
                    };
                    draw_rectangle(
                        i as f32 * cell_size + padding,
                        j as f32 * cell_size + padding,
                        cell_size - 2.0 * padding,
                        cell_size - 2.0 * padding,
                        color,
                    );
                    if let Some(priority) = allocation_field.get_alloc_priority(i as isize, j as isize) {
                        let text = &priority.to_string();
                        let text_dims = measure_text(text, None, 16, 1.0);
                        draw_text(
                            text,
                            i as f32 * cell_size + cell_size / 2.0 - text_dims.width / 2.0,
                            j as f32 * cell_size + cell_size / 2.0 + text_dims.height / 2.0,
                            16.0,
                            BLACK,
                        );
                    }
                }
            }
        }

        for (agent_id, position) in current_positions.iter().enumerate() {
            let color = match agent_id {
                0 => Color { r: 0.8, g: 0.0, b: 0.0, a: 1.0 },
                1 => Color { r: 0.0, g: 0.0, b: 0.8, a: 1.0 },
                _ => Color { r: 0.3, g: 0.3, b: 0.3, a: 1.0 },
            };
            let agent_x = position.real_position.0 as f32;
            let agent_y = position.real_position.1 as f32;
            draw_circle(
                (agent_x + 0.5) * cell_size,
                (grid_height - (agent_y + 0.5)) * cell_size,
                cell_size / 4.0,
                color,
            );
        }

        if !is_paused && macroquad::time::get_time() - last_update > 0.5 {
            last_update = macroquad::time::get_time();
            let mut all_agents_finished = true;
            for (agent_id, position) in current_positions.iter_mut().enumerate() {
                if agent_id == 0 {
                    if ::rand::thread_rng().gen_bool(0.5) {
                        continue;
                    }
                }
                if position.semantic_position.trajectory_index < mapf_result.trajectories[agent_id].poses.len() - 1 {
                    position.semantic_position.trajectory_index += 1;
                    let next_pose = &mapf_result.trajectories[agent_id].poses[position.semantic_position.trajectory_index];
                    position.real_position = (next_pose.translation.x, next_pose.translation.y);
                    all_agents_finished = false;
                }
            }

            if all_agents_finished {
                break;
            }
        }

        let time_step_text = if !current_positions.is_empty() {
            format!("Time Step: {}", current_positions[0].semantic_position.trajectory_index)
        } else {
            "Time Step: 0".to_string()
        };
        draw_text(&time_step_text, 20.0, 20.0, 30.0, BLACK);

        next_frame().await
    }
}
