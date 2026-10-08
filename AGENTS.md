# mini_exam 开发入口

这是 JAKA/Franka 驱动与 Bullet/Rerun 的应用例程仓。先读 [README](README.md)、
[paper 最新交接](../paper/讨论/20260921%201720%20Codex%20roplat%20内部基线合并与交接.md)，
再读取 [roplat-development 技能](../roplat-skills/skills/roplat-development/SKILL.md)
和 [drives 入口](../drives/AGENTS.md)。这些相邻目录是开发参考，构建使用 Cargo.lock
锁定的 crates.io 包，不要求相邻仓存在。

- 当前发布包源码和 Cargo metadata 优先于历史交接中的版本状态。
- 直接驱动例程保留原生行为接口。新增 roplat 应用编排时采用 `#[roplat::system]`，
  并先阅读技能里的语法、生命周期和节律边界；不要手工调用 process 链代替 System。
- 保持 `robot_behavior` 和公开几何类型的单一依赖身份；驱动仍使用 nalgebra 0.34。
- 构建时设置 `BULLET_SKIP_ASSET_EXPORT=1` 与 `ROPLAT_SKIP_ASSET_EXPORT=1`。
- 自动验证只运行有限 DIRECT 仿真和无硬件测试；真机测试默认 ignored。
  不无人值守运行 GUI、连接设备或执行 ignored 测试。
- 更新后检查默认与 rerun 两种配置的 all-targets、Clippy 和格式，再运行 DIRECT 回归。
  编译、仿真通过不能写成真机验证通过。
