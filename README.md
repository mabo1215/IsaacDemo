# 人形机器人安装石膏板：Isaac Sim + ROS 2

本目录实现 `docs/Design.md` 的第一版可运行演示：Isaac Sim 4.5、Python、ROS 2 Humble、PhysX 碰撞几何、人形机器人占位模型、气动钉枪，以及“搬板—贴墙—固定点打钉”的事件模型。

## 当前环境结论

- Windows 11 Pro，RTX 3070 8 GB，32 GB RAM，NVIDIA 驱动 616.56。
- Isaac Sim 6.x 的当前最低 GPU 要求高于本机，因此选用官方仍支持 RTX 3070 8 GB 的 Isaac Sim 4.5.0。
- ROS 2 Humble 安装在独立 WSL2 发行版 `IsaacUbuntu2204`，不会覆盖已有的 `Ubuntu` 或 `C:\source\ExternalCalibration`。
- `ExternalCalibration` 保持原样；两项目共用 NVIDIA 驱动、WSL2 GPU 能力和磁盘，不共用 Python/ROS 工作区，避免依赖互相污染。

## 目录

```text
sim/humanoid_drywall_demo.py                 Isaac Sim 场景、PhysX 仿真和事件模型
ros2_ws/src/isaac_drywall_demo/              ROS 2 控制节点与话题契约
tools/render_evidence_video.py                根据仿真轨迹生成稳定的 MP4 证据视频
tools/encode_video.py                         PNG 序列转 MP4 的备用工具
scripts/install_ubuntu2204_wsl.ps1           导入 Ubuntu 22.04 WSL2
scripts/install_ros2_humble.sh                安装 ROS 2 Humble 并构建工作区
scripts/run_demo.ps1                          一键运行仿真并生成视频
```

## 已安装位置

```text
Isaac Sim: C:\source\IsaacDemo\third_party\isaac-sim-4.5.0
ROS 2 WSL: IsaacUbuntu2204
WSL 根目录: C:\source\IsaacDemo\third_party\wsl\IsaacUbuntu2204
```

## 运行

在 PowerShell 中执行：

```powershell
$env:OMNI_KIT_ACCEPT_EULA = 'YES'
$env:ISAAC_SIM_ROOT = 'C:\source\IsaacDemo\third_party\isaac-sim-4.5.0'
.\scripts\run_demo.ps1
```

默认流程会运行 360 帧（约 12 秒），生成：

```text
outputs/drywall_installation.usd
outputs/run_summary.json
outputs/trajectory.csv
outputs/drywall_installation.mp4
```

视频包含石膏板、墙体、木龙骨、人形机器人、气动钉枪、固定点、碰撞状态、接触力/工具压力、已安装钉子和阶段状态。石膏板与钉子采用设计文档规定的事件模型，不宣称 FEM 材料破坏仿真。

## ROS 2 控制

先在 Isaac Sim 仿真运行后，在 WSL2 中执行：

```bash
source /opt/ros/humble/setup.bash
source /mnt/c/source/IsaacDemo/scripts/ros2_env.sh
export ROS_DOMAIN_ID=42
ros2 run isaac_drywall_demo ros_controller
```

控制节点使用：

```text
/joint_states                  Isaac Sim -> ROS 2 关节状态
/joint_command                 ROS 2 -> Isaac Sim 关节目标
/drywall_install/status        阶段状态
```

由于本机的 Windows 驱动与 Isaac Sim 4.5 Replicator/ROS bridge 组合在 warm-start 阶段会发生原生访问冲突，默认运行不加载 bridge，保证 USD/PhysX/MP4 稳定产出；`-EnableRosBridge` 仍保留用于用户后续在兼容驱动或 Linux 主机上验证原生 bridge：

```powershell
.\scripts\run_demo.ps1 -EnableRosBridge
```

## 人形机器人资产

离线 ZIP 不包含 Unitree H1 的 USD 资产。未设置 `ISAAC_ASSETS_ROOT` 时，仿真使用带碰撞几何的低成本人形占位模型，仍可完成流程和视频。若已有可访问的 Isaac 资产根目录，设置：

```powershell
$env:ISAAC_ASSETS_ROOT = 'omniverse://localhost/NVIDIA'
```

脚本会尝试加载 `Isaac/Robots/Unitree/H1/h1_with_hand.usd`；加载失败时自动回退到占位模型。

## 设计范围

第一版聚焦可复现闭环：石膏板抓取/贴墙、墙体与龙骨碰撞几何、固定点轨迹、气动钉枪压力事件、钉子安装标记、ROS 2 控制接口和视频证据。视觉识别、深度/点云、真实接触传感器标定、FEM 破坏和更高保真 H1 动力学留作下一阶段。
