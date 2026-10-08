use std::{f64::consts::PI, time::Duration};

use anyhow::{Context, ensure};
use franka_rust::FrankaEmika;
use nalgebra as na;
use robot_behavior::{behavior::*, utils::path_generate::joint_s_curve};
#[cfg(feature = "rerun")]
use roplat_rerun::RerunHost;
use rsbullet::{InverseKinematicsOptions, RsBullet};

fn main() -> anyhow::Result<()> {
    let limit = mini_exam::step_limit()?;
    let mode = mini_exam::simulation_mode()?;
    #[cfg(feature = "rerun")]
    let mut renderer = if matches!(&mode, rsbullet::Mode::Gui) {
        Some(RerunHost::new("panda_cartesian")?)
    } else {
        None
    };
    let mut physics_engine = RsBullet::new(mode)?;
    let step_time = Duration::from_secs_f64(1. / 240.);

    physics_engine
        .add_search_path(mini_exam::asset_dir())?
        .set_gravity([0., 0., -9.81])?
        .set_step_time(step_time)?;

    let mut robot = physics_engine
        .robot_builder::<FrankaEmika>("robot_1")
        .base([0.0, 0.0, 0.0])
        .base_fixed(true)
        .end_effector_link_name("panda_grasptarget")
        .load()?;

    #[cfg(feature = "rerun")]
    if let Some(renderer) = renderer.as_mut() {
        renderer.add_search_path(mini_exam::asset_dir())?;
        renderer
            .robot_builder::<FrankaEmika>("robot_1")
            .base([0.0, 0.0, 0.0])
            .base_fixed(true)
            .load()?
            .attach_from(&mut robot)?;
    }

    let translation = na::Translation3::new(0.16, -0.20, 0.0);
    let rotation = na::UnitQuaternion::from_euler_angles(PI, 0.0, 0.0);
    let target_pose = na::Isometry3::from_parts(translation, rotation);
    let mut ik_step = None;

    for step in 0_usize.. {
        if limit.is_some_and(|n| step >= n) {
            break;
        }
        if step == 100 {
            // 与 move_to 使用同一缓存状态、S 曲线规划器和模型限制，确保
            // 第一项队列任务结束，并留 0.5 s 稳定时间后才提交下一项运动。
            let start: [f64; 7] = robot
                .state()?
                .joint
                .meas
                .q
                .context("Panda joint state is unavailable before planning")?;
            let (_, motion_duration) = joint_s_curve(
                &start,
                &FrankaEmika::JOINT_DEFAULT,
                &FrankaEmika::JOINT_VEL_BOUND,
                &FrankaEmika::JOINT_ACC_BOUND,
                &FrankaEmika::JOINT_JERK_BOUND,
            );
            let delay = motion_duration
                .checked_add(Duration::from_millis(500))
                .context("Panda motion duration exceeds the simulation time range")?;
            let delay_steps = usize::try_from(delay.as_nanos().div_ceil(step_time.as_nanos()))?;
            let next_step = step
                .checked_add(delay_steps)
                .context("Panda IK step exceeds the simulation step range")?;
            robot.move_to::<JointSpace<7>>(FrankaEmika::JOINT_DEFAULT)?;
            ik_step = Some(next_step);
            eprintln!(
                "Panda joint trajectory: {:.3} s; IK starts at step {next_step} after 0.5 s settling",
                motion_duration.as_secs_f64()
            );
        }
        if ik_step == Some(step) {
            // RsBullet 0.4 的通用 FlangeSpace 运动不提供 IK；显式使用模型的
            // grasp target 求解，再将七个机械臂关节目标交给运动队列。
            // Panda URDF 另有两个手指自由度，求解种子须包含全部活动关节。
            let current_positions: Vec<f64> = physics_engine
                .client
                .get_joint_states(robot.body_id, &robot.joint_indices)?
                .into_iter()
                .map(|state| state.position)
                .collect();
            let solution = physics_engine.client.calculate_inverse_kinematics(
                robot.body_id,
                robot.end_effector_link,
                target_pose,
                &InverseKinematicsOptions {
                    current_positions: Some(&current_positions),
                    max_iterations: Some(100),
                    residual_threshold: Some(1e-5),
                    ..Default::default()
                },
            )?;
            let target: [f64; 7] = solution
                .get(..7)
                .context("Panda IK returned fewer than seven arm joints")?
                .try_into()?;
            ensure!(
                target.iter().all(|q| q.is_finite()),
                "Panda IK returned a non-finite joint target"
            );
            robot.move_to::<JointSpace<7>>(target)?;
        }
        physics_engine.step()?;
    }
    Ok(())
}
