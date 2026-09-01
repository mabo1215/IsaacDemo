# Genie Sim + AgiBot Genie G2 石膏板安装仿真进度

最后更新：2026-09-02

当前状态：第一阶段可运行演示已完成；官方 G2 全身 PhysX/ROS 2 联调待切换到兼容的 Linux + Isaac Sim 5.1 环境。

## 已完成

- 根据 `docs/Design.md` 建立 Isaac Sim + Python + ROS 2 + 人形机器人石膏板安装任务。
- 检查本机环境：Windows 11、RTX 3070 8 GB、32 GB RAM、Isaac Sim 4.5.0。
- 使用官方 Genie Sim 资产源获取 AgiBot Genie G2 `G2_omnipicker` USD 资产。
- 获取官方 Genie Sim 源码并完成 ROS 2 工作区构建检查。
- 在 Isaac Sim 中运行墙体、木龙骨、石膏板、气动钉枪和固定点事件模型。
- 完成 360 帧任务仿真，6/6 个固定点触发。
- 导出任务 USD、官方 G2 USD 组合层、轨迹 CSV 和 12 秒 H.264 演示视频。
- 保留 `/World/RenderCamera`，本阶段不依赖 RealSense D415。
- `C:\source\ExternalCalibration` 未修改。
- 基础文档、`tools/` 和 `scripts/` 已分批提交并推送到 `origin/main`。

## 已完成但有明确边界

- `outputs/genie_g2_drywall/genie_g2_drywall_task.usd` 是当前 Windows Isaac Sim 4.5 实际运行的 PhysX 任务场景。
- `outputs/genie_g2_drywall/genie_g2_drywall.usda` 组合了任务场景和官方 G2 USD，供兼容环境继续打开。
- 当前 Windows 运行阶段使用低面数 G2 代理 `/World/GenieG2Proxy`；官方高分辨率 G2 被延迟到组合层，未作为当前 PhysX articulation 运行。
- `drywall_installation.mp4` 是从 Isaac Sim 轨迹生成的稳定证据视频，不是 Replicator 原生逐帧渲染视频。
- 轨迹控制和气钉事件已实现；真实钉子穿透、材料破坏和 FEM 尚未实现。

## 未完成或部分完成

- [部分完成] ROS 2 话题和示例控制节点已准备，当前 G2 视频运行没有启动实时 ROS 2 bridge。
- [部分完成] Isaac Sim 相机已写入 USD，但尚未产出稳定的原生 RGB/深度帧序列。
- [未完成] 官方 G2 的完整关节物理、全身控制和真实关节状态闭环。
- [未完成] 将模拟 D415 的 RGB、深度、点云和标定参数接入控制链路。
- [未完成] 用真实机器人或 AIST HRP-5P 级别的双手协同动作替代代理轨迹。

## 遗留问题与原因

- [已识别阻塞] Windows + Isaac Sim 4.5 + RTX 3070 8 GB 加载官方 G2 高分辨率视觉/PhysX payload 时会超过 RTX TLAS 预算，并在 `world.reset()` warm-start 阶段出现 access violation。
- [已识别阻塞] Docker Desktop 中的 Genie Sim 官方启动路径缺少可用 Vulkan/`libGLX_nvidia.so`，因此不能在当前 Docker 环境完成原生图形启动。
- [设计限制] 当前视频渲染器是轨迹证据渲染器，机器人外观为代理绘制，不应被误认为官方 G2 原生相机渲染。

## 下一步计划

1. 在 Linux + Isaac Sim 5.1 + Genie Sim 环境打开 `genie_g2_drywall.usda`。
2. 将代理 prim 替换为 `/World/GenieG2` 官方资产，启用 G2 PhysX articulation。
3. 接入 Genie Sim 的 G2 控制器和 ROS 2 joint state/command 链路。
4. 使用 Isaac Sim Replicator 从 `/World/RenderCamera` 输出 RGB/深度视频。
5. 最后再加入 D415 的标定、噪声模型和真实设备联调。
