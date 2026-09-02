# Genie Sim + AgiBot Genie G2 石膏板安装仿真

当前有效结果由 vGPU 3090 上的 Linux + Isaac Sim 5.1 生成。任务直接加载官方
`G2_omnipicker/robot.usda`，使用 Isaac Sim RTX/Replicator 输出原生 RGB 帧；本阶段不依赖
RealSense D415。

## 本地有效输出

- `outputs/demo/genie_g2_official_drywall_installation.mp4`：600 帧、30 FPS、20 秒 H.264 视频。
- `outputs/demo/g2_official_drywall/run_summary.json`：vGPU 运行摘要，记录官方 G2 和 46 个关节初始化。
- `outputs/demo/g2_official_drywall/trajectory.csv`：石膏板、工具和 6 个固定点事件轨迹。
- `outputs/demo/g2_official_drywall/genie_g2_official_drywall.usd`：vGPU 导出的任务 USD。
- `outputs/demo/g2_official_smoke/rgb_0000.png`：官方 G2 单帧加载/渲染检查。

## vGPU 运行与同步

远端渲染输出应写入：
`/root/autodl-tmp/IsaacDemo_g2/outputs/g2_official_drywall_final`。
在本地仓库 PowerShell 中同步摘要、轨迹和任务 USD：

```powershell
.\scripts\sync_vgpu_outputs.ps1 -IncludeTaskUsd
```

需要把 PNG 帧拉回本地并重新编码视频时使用：

```powershell
.\scripts\sync_vgpu_outputs.ps1 -IncludeTaskUsd -IncludeFrames -EncodeVideo
```

脚本从 `C:\source\.env` 的 `vGPU 3090` 条目读取登录信息；凭据不会写入仓库。

## 仿真边界

官方 G2 已在 vGPU Isaac Sim 5.1 中成功加载为 46-DOF articulation，并完成从起始位走到
石膏板前、双臂进入工作姿态、按 0..5 顺序覆盖 6 个固定点和原生 RTX 帧渲染。真实钉子穿透、
石膏板材料破坏、FEM、全身控制闭环和 ROS 2 实时桥接仍需单独验证。D415 目前由 Isaac Sim
相机替代，后续再加入其内参、噪声和深度格式映射。

`C:\source\ExternalCalibration` 未被修改。
