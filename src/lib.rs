//! Shared configuration for the simulator examples.

use std::{env, path::PathBuf};

use anyhow::{Context, bail};
use libjaka::JakaMini2;
use robot_behavior::{EndPoint, Joints, RobotDescription};
use rsbullet::Mode;

/// JAKA Mini2 model with a finite jerk bound for Bullet trajectory planning.
///
/// The driver's default `f64::MAX` jerk can overflow when robot_behavior 0.6.1
/// normalizes short motions. The 1000 rad/s^3 bound is a simulation planning
/// parameter, not a hardware specification; real JAKA examples use the driver
/// model directly. All other model constants are preserved.
pub struct SimJakaMini2;

impl RobotDescription for SimJakaMini2 {
    const URDF: Option<&'static str> = JakaMini2::URDF;
}

impl Joints<6> for SimJakaMini2 {
    const JOINT_DEFAULT: [f64; 6] = JakaMini2::JOINT_DEFAULT;
    const JOINT_PACKED: [f64; 6] = JakaMini2::JOINT_PACKED;
    const JOINT_MIN: [f64; 6] = JakaMini2::JOINT_MIN;
    const JOINT_MAX: [f64; 6] = JakaMini2::JOINT_MAX;
    const JOINT_VEL_BOUND: [f64; 6] = JakaMini2::JOINT_VEL_BOUND;
    const JOINT_ACC_BOUND: [f64; 6] = JakaMini2::JOINT_ACC_BOUND;
    const JOINT_JERK_BOUND: [f64; 6] = [1000.0; 6];
    const TORQUE_BOUND: [f64; 6] = JakaMini2::TORQUE_BOUND;
    const TORQUE_DOT_BOUND: [f64; 6] = JakaMini2::TORQUE_DOT_BOUND;
}

impl EndPoint for SimJakaMini2 {
    const CARTESIAN_VEL_BOUND: f64 = JakaMini2::CARTESIAN_VEL_BOUND;
    const CARTESIAN_ACC_BOUND: f64 = JakaMini2::CARTESIAN_ACC_BOUND;
    const CARTESIAN_JERK_BOUND: f64 = JakaMini2::CARTESIAN_JERK_BOUND;
    const ROTATION_VEL_BOUND: f64 = JakaMini2::ROTATION_VEL_BOUND;
    const ROTATION_ACC_BOUND: f64 = JakaMini2::ROTATION_ACC_BOUND;
    const ROTATION_JERK_BOUND: f64 = JakaMini2::ROTATION_JERK_BOUND;
}

/// Resolve bundled models independently of the caller's working directory.
pub fn asset_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("asserts")
}

/// Use Bullet DIRECT for finite command-line checks; GUI is the interactive default.
pub fn simulation_mode() -> anyhow::Result<Mode> {
    match env::var("MINI_EXAM_DIRECT") {
        Ok(value) if value == "1" => Ok(Mode::Direct),
        Ok(value) if value == "0" => Ok(Mode::Gui),
        Err(env::VarError::NotPresent) => Ok(Mode::Gui),
        _ => bail!("MINI_EXAM_DIRECT must be 0 or 1"),
    }
}

/// Bound the total simulation steps when MINI_EXAM_STEPS is set.
pub fn step_limit() -> anyhow::Result<Option<usize>> {
    match env::var("MINI_EXAM_STEPS") {
        Ok(value) => {
            Ok(Some(value.parse().context(
                "MINI_EXAM_STEPS must be a non-negative integer",
            )?))
        }
        Err(env::VarError::NotPresent) => Ok(None),
        Err(error) => Err(error).context("could not read MINI_EXAM_STEPS"),
    }
}
