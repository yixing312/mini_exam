use nalgebra::{self as na, Vector3};
use std::{f64::consts::FRAC_PI_2, time::Duration};

use mini_exam::SimJakaMini2;
use robot_behavior::{Entity, behavior::*};
use rsbullet::RsBullet;

fn main() -> anyhow::Result<()> {
    let limit = mini_exam::step_limit()?;
    let mut physics_engine = RsBullet::new(mini_exam::simulation_mode()?)?;

    physics_engine
        .add_search_path(mini_exam::asset_dir())?
        .set_gravity([0., 0., -10.])?
        .set_step_time(Duration::from_secs_f64(1. / 240.))?;

    let mut robot_1 = physics_engine
        .robot_builder::<SimJakaMini2>("robot_1")
        .base(na::Isometry3::from_parts(
            [0.0, 0.2, 0.0].into(),
            na::Rotation3::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2).into(),
        ))
        .base_fixed(true)
        .load()?;
    let mut robot_2 = physics_engine
        .robot_builder::<SimJakaMini2>("robot_2")
        .base(na::Isometry3::from_parts(
            [0.0, -0.2, 0.0].into(),
            na::Rotation3::from_axis_angle(&Vector3::z_axis(), -FRAC_PI_2).into(),
        ))
        .base_fixed(true)
        .load()?;

    physics_engine
        .visual(Entity::Box {
            half_extents: [0.01, 0.375, 0.01],
        })
        .base([0.2125, 0., 0.3])
        .load()?;
    physics_engine
        .visual(Entity::Box {
            half_extents: [0.01, 0.375, 0.01],
        })
        .base([0.2125, 0., 0.15])
        .load()?;
    physics_engine
        .visual(Entity::Box {
            half_extents: [0.01, 0.01, 0.3],
        })
        .base([0.2125, 0.1875, 0.15])
        .load()?;
    physics_engine
        .visual(Entity::Box {
            half_extents: [0.01, 0.01, 0.3],
        })
        .base([0.2125, -0.1875, 0.15])
        .load()?;
    physics_engine
        .visual(Entity::Box {
            half_extents: [0.5, 0.5, 0.5],
        })
        .base([0.1, 0., -0.25])
        .load()?;

    for step in 0.. {
        if limit.is_some_and(|n| step >= n) {
            break;
        }
        if step == 100 {
            robot_1.move_to::<JointSpace<6>>([0.; 6])?;
            robot_2.move_to::<JointSpace<6>>([0.; 6])?;
        }
        physics_engine.step()?;
    }
    Ok(())
}
