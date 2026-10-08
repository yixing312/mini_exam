use std::time::Duration;

use anyhow::Context;
use mini_exam::{SimJakaMini2, asset_dir};
use robot_behavior::behavior::*;
use rsbullet::{Mode, RsBullet};

#[test]
fn direct_jaka_reaches_joint_target() -> anyhow::Result<()> {
    let mut physics = RsBullet::new(Mode::Direct)?;
    physics
        .add_search_path(asset_dir())?
        // Isolate command execution from gravity and contact tuning.
        .set_gravity([0.0; 3])?
        .set_step_time(Duration::from_secs_f64(1.0 / 240.0))?;
    let mut robot = physics
        .robot_builder::<SimJakaMini2>("joint_motion_test")
        .base([0.0; 3])
        .base_fixed(true)
        .load()?;
    assert_eq!(
        robot.joint_indices.len(),
        6,
        "expected the bundled JAKA arm"
    );

    let initial = robot
        .state()?
        .joint
        .meas
        .q
        .context("initial measured joint positions are missing")?;
    assert!(initial.iter().all(|q| q.is_finite()));

    let target = [0.30, -0.25, 0.20, -0.15, 0.10, -0.05];
    robot.move_to::<JointSpace<6>>(target)?;
    // RsBullet queues motion; only stepping advances the queued command.
    // Five simulated seconds also allows the position controller to settle.
    for _ in 0..1200 {
        physics.step()?;
    }

    let state = robot.state()?;
    let actual = state
        .joint
        .meas
        .q
        .context("final measured joint positions are missing")?;
    for joint in 0..6 {
        assert!(actual[joint].is_finite(), "joint {joint} is not finite");
        assert!(
            (actual[joint] - initial[joint]).abs() > 0.02,
            "joint {joint} did not move: initial={}, actual={}",
            initial[joint],
            actual[joint]
        );
        assert!(
            (actual[joint] - target[joint]).abs() < 0.03,
            "joint {joint} missed its target: expected={}, actual={}",
            target[joint],
            actual[joint]
        );
    }
    let pose = state
        .flange
        .meas
        .pose
        .context("final measured flange pose is missing")?;
    assert!(pose.quat().to_homogeneous().iter().all(|v| v.is_finite()));
    physics.shutdown();
    Ok(())
}
