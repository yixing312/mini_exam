use libjaka::JakaMini2;
use robot_behavior::Arm;

fn robot_ip() -> String {
    std::env::var("JAKA_IP").unwrap_or_else(|_| "10.5.5.100".to_owned())
}

fn main() -> anyhow::Result<()> {
    let mut robot = JakaMini2::new(&robot_ip());
    let state = robot.state()?;
    println!("Robot State: {:?}", state);
    Ok(())
}

#[cfg(test)]
mod tests {
    use anyhow::Context;
    use libjaka::JakaMini2;
    use robot_behavior::Arm;

    #[test]
    #[ignore = "requires a real JAKA robot and explicit operator authorization"]
    fn get_state() -> anyhow::Result<()> {
        let mut robot = JakaMini2::new(&super::robot_ip());
        let state = robot.state()?;
        let q = state
            .joint
            .meas
            .q
            .context("joint position is unavailable")?;
        let pose = state
            .flange
            .meas
            .pose
            .context("flange pose is unavailable")?
            .euler();
        println!("q:{q:?}\nflange pose:{pose:?}");
        Ok(())
    }
}
