use libjaka::JakaMini2;
use robot_behavior::Robot;

fn robot_ip() -> String {
    std::env::var("JAKA_IP").unwrap_or_else(|_| "10.5.5.100".to_owned())
}

fn main() -> anyhow::Result<()> {
    let mut robot = JakaMini2::new(&robot_ip());

    // 当前 JAKA 驱动的 init 对应上电，enable 对应使能。
    robot.init()?;
    robot.enable()?;

    // 此时你才可以发送运动指令

    robot.disable()?;
    // 当前 JAKA 驱动的 shutdown 对应断电。
    robot.shutdown()?;

    Ok(())
}

/// 这些测试直接操作设备，普通测试会跳过；现场确认后通过 --ignored 显式运行。
#[cfg(test)]
mod tests {
    use robot_behavior::Robot;

    #[test]
    #[ignore = "requires a real JAKA robot and explicit operator authorization"]
    fn power_on() -> anyhow::Result<()> {
        let mut robot = super::JakaMini2::new(&super::robot_ip());
        robot.init()?;
        Ok(())
    }

    #[test]
    #[ignore = "requires a real JAKA robot and explicit operator authorization"]
    fn power_off() -> anyhow::Result<()> {
        let mut robot = super::JakaMini2::new(&super::robot_ip());
        robot.shutdown()?;
        Ok(())
    }

    #[test]
    #[ignore = "requires a real JAKA robot and explicit operator authorization"]
    fn enable() -> anyhow::Result<()> {
        let mut robot = super::JakaMini2::new(&super::robot_ip());
        robot.enable()?;
        Ok(())
    }

    #[test]
    #[ignore = "requires a real JAKA robot and explicit operator authorization"]
    fn disable() -> anyhow::Result<()> {
        let mut robot = super::JakaMini2::new(&super::robot_ip());
        robot.disable()?;
        Ok(())
    }
}
