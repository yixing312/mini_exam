# mini_exam

JAKA Mini2 真机、JAKA/Franka Bullet 仿真与可选 Rerun 可视化例程。

## 依赖基线（2026-10-08）

所有依赖从 crates.io 获取；提交的 `Cargo.lock` 固定完整依赖图。
本次已实时核对发布版本并执行 `cargo update`，没有使用相邻仓的本地 patch。

| 直接依赖 | 版本 |
| --- | --- |
| robot_behavior | 0.6.1 |
| libjaka / franka_rust | 0.2.0 |
| rsbullet | 0.4.0 |
| roplat_rerun（可选） | 0.2.0 |
| anyhow | 1.0.104 |
| nalgebra | 0.34.2 |

采用当前驱动兼容的最新版本：驱动公开的 `Pose` / `Isometry3` 仍使用 nalgebra
0.34，因此不能仅将本项目改为 0.35；`roplat_rerun` 仍依赖 Rerun 0.26，锁定
0.26.2，Viewer 也需匹配。`robot_behavior` 0.6.1 明确要求 `copp =0.2.2`，
保留该上游约束。其余传递依赖按各上游约束更新。

版本核对来源：[crates.io](https://crates.io/)、
[behavior 发布包](https://docs.rs/crate/robot_behavior/0.6.1)、
[Rerun 适配包依赖](https://docs.rs/crate/roplat_rerun/0.2.0)；
旧交接中“尚未发布”的描述是当时状态。

## 环境与构建

- Rust nightly（驱动使用 nightly 特性）。
- [CMake](https://cmake.org/download/) 和 C++ 工具链；Windows 需安装
  [Visual Studio C++ Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/)。
- 可选 Viewer：`cargo install rerun-cli --version 0.26.2 --locked`。

PowerShell 中先设置下列变量，避免依赖构建脚本向用户资源目录导出模型；
例程直接读取本项目 `asserts/`，无需改成个人绝对路径：

```powershell
$env:BULLET_SKIP_ASSET_EXPORT = "1"
$env:ROPLAT_SKIP_ASSET_EXPORT = "1"
cargo check --all-targets --locked
cargo check --all-targets --features rerun --locked
```

## 例程

| 入口 | 用途 |
| --- | --- |
| `cargo run --locked` | 单 JAKA 仿真；`--features rerun` 可附加 Viewer |
| `--example basic_example` | JAKA 上电、使能、去使能、断电 |
| `--example motion` | 类型化关节/法兰运动、六关节 JSON 轨迹 |
| `--example state` | JAKA 当前状态 |
| `--example tio_vout` | JAKA 工具端 24V 输出与读取 |
| `--example sim_only_physics` | 单 JAKA 物理仿真 |
| `--example sim_add_line` | 仿真关节坐标轴 |
| `--example sim_with_renderer --features rerun` | Bullet 与 Rerun 联动 |
| `--example sim_world` | 双 JAKA 与场景物体 |
| `--example yqh_exam` | Franka 关节运动、显式 IK 后的关节目标 |

以上例程参数均接在 `cargo run --locked` 后。真机例程通过 `JAKA_IP` 设置地址，
默认沿用 `10.5.5.100`。`motion` 要求现场已完成上电/使能，并在运行目录准备
已检查的 `xx.json` 轨迹；其格式为 `[[q0,q1,q2,q3,q4,q5], ...]`，单位为弧度。
读取状态使用 `state.joint.meas.q` 与 `state.flange.meas.pose`，可用性由 `Option` 表示。

普通运动使用 `move_to::<JointSpace<N>>(target)`；真机法兰运动使用
`move_to::<FlangeSpace>(pose)`。RsBullet 0.4.0 的法兰运动入口当前不支持推断
关节数，`yqh_exam` 因此显式调用 Bullet IK，再发送七关节目标。

JAKA 仿真采用 `SimJakaMini2`：沿用驱动的 URDF 与其他模型限制，单独将规划 jerk
设为 1000 rad/s³。这是例程的仿真配置，不是厂商设备参数；它避免当前驱动默认
`f64::MAX` jerk 在短距离 S 曲线规划中溢出为 NaN。真机例程仍使用 `JakaMini2`。
Franka 第二段运动等待第一段规划时长并额外稳定 0.5 秒后启动，避免同时写入关节命令。

## 有限、无界面验证

仿真默认打开 GUI 并持续运行。设置以下变量后使用 DIRECT，跳过可选 Viewer
及仅 GUI 支持的调试坐标轴，
并在指定总步数后退出。`sim_world` 在第 100 步发送关节目标；`yqh_exam` 在第
100 步发送默认姿态，打印按规划时长计算的 IK 启动步数。覆盖 IK 时设置的总步数
须大于该输出值；以下使用 2000 步：

```powershell
$env:MINI_EXAM_DIRECT = "1"
$env:MINI_EXAM_STEPS = "2000"
cargo run --locked
cargo run --locked --example sim_world
cargo run --locked --example yqh_exam
```

`MINI_EXAM_DIRECT` 接受 `0` 或 `1`，`MINI_EXAM_STEPS` 接受非负整数。
回到交互模式前移除这两个环境变量。无界面回归与静态检查：

```powershell
cargo test --all-targets --locked
cargo clippy --all-targets --locked -- -D warnings
cargo clippy --all-targets --features rerun --locked -- -D warnings
cargo fmt --check
```

`tests/simulation.rs` 使用有限 DIRECT 仿真断言模型加载、关节状态与运动响应。
8 个设备测试均默认 ignored；普通测试不会连接机器人。编译或仿真通过不代表
真机或 Viewer 联调完成。

2026-10-08 在 Windows x86_64 MSVC、`rustc 1.99.0-nightly (c4af71034 2026-07-06)`
上的实际验证：

| 检查 | 结果 |
| --- | --- |
| 默认 / rerun 的 all-targets 编译与 Clippy `-D warnings` | 均通过 |
| 默认 `cargo test --all-targets --locked` | 1 项 DIRECT 运动回归通过，8 项真机测试按预期忽略 |
| 主程序及全部 5 个仿真例程 | DIRECT 各 2000 步退出成功；从仓库外启动也能找到模型 |
| Franka IK 路径 | 首段规划 1.259 s，额外稳定后于第 523 步进入 IK，程序正常退出；不等同于末端精度验收 |
| 格式与依赖身份 | fmt / diff 检查通过；行为、几何与渲染库均为单一 registry 包身份 |

保留的上游提示：Windows Bullet 链接产生 `LNK4098` C runtime 默认库冲突警告；
JAKA URDF 的 `dummy_tcp` 缺惯性数据；Rerun 依赖的 `binrw 0.12.1` 有未来 Rust
兼容性提示。本轮构建与 DIRECT 运行通过，未修改上游源码来消除这些提示，也未运行
GUI、Viewer 或真实设备。原始验证日志保存在被 Git 忽略的 `target/update-*.log`。

## 开发上下文

已接入 [roplat-development 技能](../roplat-skills/skills/roplat-development/SKILL.md)、
[drives 当前接口说明](../drives/AGENTS.md) 与
[paper 交接](../paper/讨论/20260921%201720%20Codex%20roplat%20内部基线合并与交接.md)。
相邻目录仅为开发参考，独立克隆本仓仍可构建。这些例程直接展示驱动能力；
新增 roplat 应用图时按技能使用 `#[roplat::system]`，并显式开启相应 roplat feature。
