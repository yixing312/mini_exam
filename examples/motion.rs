use libjaka::JakaMini2;
use nalgebra as na;
use robot_behavior::behavior::*;

fn robot_ip() -> String {
    std::env::var("JAKA_IP").unwrap_or_else(|_| "10.5.5.100".to_owned())
}

fn main() -> anyhow::Result<()> {
    // 运行前应在现场完成上电、使能，并确认下列目标和轨迹可执行。
    let mut robot = JakaMini2::new(&robot_ip());

    // 当前驱动用 set_scale 配置后续运动的速度、加速度倍率。
    robot.set_scale(0.3);
    robot.move_to::<JointSpace<6>>([0.; 6])?;

    robot.move_to::<FlangeSpace>(Pose::Quat(na::Isometry3::identity()))?;

    // 使用 `xx.json` 文件中的密集关节轨迹；每个采样点是六个弧度值。
    // 文件格式示意（实际运行前需换成已检查的轨迹）：
    // [
    //   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    //   [0.001, 0.0, 0.0, 0.0, 0.0, 0.0]
    // ]
    robot.move_traj_from_file::<JointSpace<6>>("xx.json")?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use libjaka::JakaMini2;
    use robot_behavior::{RobotResult, behavior::*};

    #[test]
    #[ignore = "requires an enabled real JAKA robot and explicit operator authorization"]
    fn move_to_default() -> RobotResult<()> {
        let mut robot = JakaMini2::new(&super::robot_ip());
        robot.move_to::<JointSpace<6>>(JakaMini2::JOINT_DEFAULT)?;
        Ok(())
    }

    #[test]
    #[ignore = "requires an enabled real JAKA robot and explicit operator authorization"]
    fn move_to_packed() -> RobotResult<()> {
        let mut robot = JakaMini2::new(&super::robot_ip());
        robot.move_to::<JointSpace<6>>(JakaMini2::JOINT_PACKED)?;
        Ok(())
    }
}
