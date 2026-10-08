use std::f64::consts::FRAC_PI_2;

use mini_exam::SimJakaMini2;
use robot_behavior::behavior::*;
#[cfg(feature = "rerun")]
use roplat_rerun::RerunHost;
use rsbullet::RsBullet;

fn main() -> anyhow::Result<()> {
    let limit = mini_exam::step_limit()?;
    let mode = mini_exam::simulation_mode()?;
    #[cfg(feature = "rerun")]
    let mut renderer = if matches!(&mode, rsbullet::Mode::Gui) {
        Some(RerunHost::new("mini_exam")?)
    } else {
        None
    };
    let mut physics = RsBullet::new(mode)?;
    physics.add_search_path(mini_exam::asset_dir())?;

    let mut robot = physics
        .robot_builder::<SimJakaMini2>("exam_robot")
        .base([0., 0., 0.])
        .base_fixed(true)
        .load()?;

    #[cfg(feature = "rerun")]
    if let Some(renderer) = renderer.as_mut() {
        renderer.add_search_path(mini_exam::asset_dir())?;
        renderer
            .robot_builder::<SimJakaMini2>("exam_robot")
            .base([0., 0., 0.])
            .base_fixed(true)
            .load()?
            .attach_from(&mut robot)?;
    }

    robot.move_to::<JointSpace<6>>([FRAC_PI_2; 6])?;

    for step in 0.. {
        if limit.is_some_and(|n| step >= n) {
            break;
        }
        physics.step()?;
    }
    Ok(())
}
