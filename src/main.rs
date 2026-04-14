use std::f64::consts::FRAC_PI_2;

use libjaka::JakaMini2;
use robot_behavior::behavior::*;
#[cfg(feature = "rerun")]
use roplat_rerun::RerunHost;
use rsbullet::{Mode, RsBullet};

fn main() -> anyhow::Result<()> {
    let mut physics = RsBullet::new(Mode::Gui)?;

    #[cfg(feature = "rerun")]
    let mut renderer = RerunHost::new("mini_exam")?;

    let mut robot = physics
        .robot_builder::<JakaMini2>("exam_robot")
        .base([0., 0., 0.])
        .load()?;

    #[cfg(feature = "rerun")]
    let robot_render = renderer
        .robot_builder("exam_robot")
        .base([0., 0., 0.])
        .load()?;

    #[cfg(feature = "rerun")]
    robot_render.attach_from(&mut robot)?;

    robot.move_joint(&[FRAC_PI_2; _])?;

    loop {
        physics.step()?;
    }
}
