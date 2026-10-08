use std::{f64::consts::FRAC_PI_2, time::Duration};

use mini_exam::SimJakaMini2;
use robot_behavior::behavior::*;
use rsbullet::{RsBullet, RsBulletRobot};

fn main() -> anyhow::Result<()> {
    let limit = mini_exam::step_limit()?;
    let mut physics = RsBullet::new(mini_exam::simulation_mode()?)?;

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

    // RsBullet 将运动加入队列；后续 step 推进轨迹与物理仿真。
    robot.move_to::<JointSpace<6>>([FRAC_PI_2; 6])?;

    for step in 0.. {
        if limit.is_some_and(|n| step >= n) {
            break;
        }
        physics.step()?;
    }
    Ok(())
}
