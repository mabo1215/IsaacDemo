# Genie Sim + AgiBot Genie G2 石膏板安装仿真进度

最后更新：2026-09-02

当前状态：vGPU 3090 上的官方 G2 原生 RTX 连续任务演示已完成；官方 G2 全身 PhysX/ROS 2 联调仍待继续验证。

## 已完成

- 根据 `docs/Design.md` 建立 Isaac Sim + Python + ROS 2 + 人形机器人石膏板安装任务。
- 检查本机环境：Windows 11、RTX 3070 8 GB、32 GB RAM、Isaac Sim 4.5.0。
- 使用官方 Genie Sim 资产源获取 AgiBot Genie G2 `G2_omnipicker` USD 资产。
- 获取官方 Genie Sim 源码并完成 ROS 2 工作区构建检查。
- 在 Isaac Sim 中运行墙体、木龙骨、石膏板、气动钉枪和固定点事件模型。
- 完成 800 帧连续任务仿真，6/6 个固定点按顺序触发。
- 导出任务 USD、官方 G2 USD 组合层、轨迹 CSV 和 26.67 秒 H.264 演示视频。
- 在 vGPU 3090、Linux + Isaac Sim 5.1 上直接加载官方 G2 articulation，初始化 46 个关节。
- 使用 Isaac Sim RTX/Replicator 生成 800 张原生 RGB 帧，并同步任务 USD、运行摘要和轨迹到本地 `outputs/demo/`。
- 修正 `box()` 的 USD 变换顺序；墙体、石膏板和固定点现在处于同一工作平面，不再出现石膏板被抬到固定点上方。
- 修正运动状态机：G2 根部从起始位平滑走到工作位，双臂进入工作姿态，工具按固定点索引 0..5 依次执行 move/press/hold。
- 增加按当前固定点变化的官方 G2 双臂关节姿态 `working_0`..`working_5`，并让右夹爪对齐一体化短钉枪握把。
- 修正长时稳定性：关闭官方 USD 关节驱动，对整机 articulation 禁用重力，每个物理步清零根部 6D 速度，并锁定保留官方高度偏移的物理根姿态；v32 正式 800 帧全程保持站立、底盘水平。
- 重构任务动作：删除跨场景橙黄色代理连接，G2 依次移动到 3 个列工作位置，在每列完成上下两个钉点，再执行 `align_tool`、`move_to_fastener`、`press_fastener`、`hold_fastener`。
- 优化换点顺序：改为列优先，每个地面位置连续完成下方和上方两个固定点，再移动到下一列；v32 正式 800 帧验证机器人底盘仅移动 3 个列位置、手臂动作和 6/6 装订事件。
- 新增连续搬板前置动作：G2 走到板前，双臂抓取石膏板，抬起、搬运至墙/龙骨、贴墙并释放，再抓取短钉枪进入原有装订轨迹。
- 将工具改为夹爪附近的一体化短钉枪模型；v32 正式 800 帧验证机器人站立姿态、底盘水平、搬板、工具抓取、底盘换点、手臂动作和 6/6 装订事件。
- 增加 `scripts/sync_vgpu_outputs.ps1`，后续 vGPU 渲染结果统一从 `C:\source\.env` 的 `vGPU 3090` 条目同步。
- 保留 `/World/RenderCamera`，本阶段不依赖 RealSense D415。
- `C:\source\ExternalCalibration` 未修改。
- 基础文档、`tools/` 和 `scripts/` 已分批提交并推送到 `origin/main`。

## 已完成但有明确边界

- `outputs/demo/g2_official_drywall/genie_g2_official_drywall.usd` 是 vGPU Isaac Sim 5.1 导出的任务 USD。
- `outputs/demo/genie_g2_official_drywall_installation.mp4` 由 vGPU 生成的原生 RTX RGB 帧编码而成，不是旧版 Windows 代理证据视频。
- 正式视频包含 walk_to_board、align_to_drywall、grasp_drywall、lift_drywall、carry_drywall、place_drywall、align_and_grasp_tool、walk_to_fastener、align_tool、move_to_fastener、press_fastener、hold_fastener 状态；6 个固定点的索引 0..5 全部完成，顺序为每列下/上配对。G2 先双手搬板并释放，再由右夹爪抓取短钉枪执行握枪/压枪动作。
- 本地不长期保留全量 PNG 帧；需要复核原始帧时使用同步脚本的 `-IncludeFrames`。
- 搬板位姿、双臂抓板姿态、板材释放、地面点位轨迹、按点臂姿态、短钉枪握持视觉耦合和气钉事件已实现；真实刚性夹爪约束、钉子穿透、材料破坏和 FEM 尚未实现。

## 未完成或部分完成

- [部分完成] ROS 2 话题和示例控制节点已准备，当前 G2 视频运行没有启动实时 ROS 2 bridge。
- [部分完成] 已产出稳定的原生 RGB 帧序列；深度帧、点云和相机标定输出尚未接入。
- [未完成] 官方 G2 的完整关节物理、全身控制和真实关节状态闭环。
- [未完成] 将模拟 D415 的 RGB、深度、点云和标定参数接入控制链路。
- [未完成] 用真实机器人或 AIST HRP-5P 级别的双手协同动作替代代理轨迹。

## 遗留问题与原因

- [已识别阻塞] Windows + Isaac Sim 4.5 + RTX 3070 8 GB 加载官方 G2 高分辨率视觉/PhysX payload 时会超过 RTX TLAS 预算，并在 `world.reset()` warm-start 阶段出现 access violation。
- [已识别阻塞] Docker Desktop 中的 Genie Sim 官方启动路径缺少可用 Vulkan/`libGLX_nvidia.so`，因此不能在当前 Docker 环境完成原生图形启动。
- [设计限制] 当前固定点和钉子仍采用确定性事件模型，不应被误认为真实钉子穿透或 FEM 材料破坏。

## 下一步计划

1. 在已验证的 vGPU Isaac Sim 5.1 环境继续接入 Genie Sim 的 G2 控制器。
2. 完成官方 G2 PhysX、关节状态和 ROS 2 command/state 闭环。
3. 使用 `/World/RenderCamera` 输出 RGB、深度和点云，并通过同步脚本归档到本地。
4. 最后再加入 D415 的标定、噪声模型和真实设备联调。
