use std::{f64::consts::FRAC_PI_2, time::Duration};

use mini_exam::SimJakaMini2;
use robot_behavior::behavior::*;
use rsbullet::{DebugLineOptions, RsBullet, RsBulletRobot};

fn main() -> anyhow::Result<()> {
    let limit = mini_exam::step_limit()?;
    let mode = mini_exam::simulation_mode()?;
    let show_debug_lines = matches!(&mode, rsbullet::Mode::Gui);
    let mut physics = RsBullet::new(mode)?;

    physics
        .add_search_path(mini_exam::asset_dir())?
        .set_gravity([0., 0., -10.])?
        .set_step_time(Duration::from_secs_f64(1. / 240.))?;

    // 仿真器基础配置
    let mut robot: RsBulletRobot<SimJakaMini2> = physics
        .robot_builder::<SimJakaMini2>("exam_robot")
        .base([0., 0., 0.])
        .base_fixed(true)
        .load()?;

    // Bullet DIRECT has no debug renderer and rejects add_user_debug_line.
    if show_debug_lines {
        for link_index in &robot.joint_indices {
            for (to, color) in [
                ([0.2, 0., 0.], [1., 0., 0.]),
                ([0., 0.2, 0.], [0., 1., 0.]),
                ([0., 0., 0.2], [0., 0., 1.]),
            ] {
                physics.client.add_user_debug_line(&DebugLineOptions {
                    from: [0., 0., 0.],
                    to,
                    color: Some(color),
                    line_width: 1.,
                    life_time: 0.,
                    parent_object_unique_id: Some(robot.body_id),
                    parent_link_index: Some(*link_index),
                    replace_item_unique_id: None,
                })?;
            }
        }
    }

    // 坐标轴随各连杆运动；运动队列由 step 推进。
    robot.move_to::<JointSpace<6>>([FRAC_PI_2; 6])?;

    for step in 0.. {
        if limit.is_some_and(|n| step >= n) {
            break;
        }
        physics.step()?;
    }
    Ok(())
}
