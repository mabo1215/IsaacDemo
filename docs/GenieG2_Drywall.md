# Genie Sim + AgiBot Genie G2 石膏板安装 Demo

本目录已经加入官方 Genie Sim G2 资产的接入路径。默认不需要 D415：任务使用 Isaac Sim 的 PhysX 场景和轨迹控制，视频使用轨迹生成确定性演示画面；导出的 USD 组合层包含官方 G2 USD 和 Isaac Sim 相机。

## 运行

在 `C:\source\IsaacDemo` PowerShell 中执行：

```powershell
.\scripts\run_genie_g2_demo.ps1
```

可选参数：

```powershell
.\scripts\run_genie_g2_demo.ps1 -Frames 360 -OutputDir outputs\genie_g2_drywall
```

输出：

- `outputs/genie_g2_drywall/genie_g2_drywall.usda`：任务场景与官方 G2 的 USD 组合层。
- `outputs/genie_g2_drywall/genie_g2_drywall_task.usd`：实际在当前 Windows Isaac Sim 4.5 中运行的 PhysX 任务场景。
- `outputs/genie_g2_drywall/trajectory.csv`：石膏板、气钉枪、接触压力和固定点事件轨迹。
- `outputs/genie_g2_drywall/drywall_installation.mp4`：6 个固定点完成的演示视频。
- `outputs/genie_g2_drywall/run_summary.json`：运行配置和限制记录。

## 当前环境边界

当前机器是 Windows + Isaac Sim 4.5 + RTX 3070 8 GB。官方 G2 高分辨率视觉/PhysX payload 在该组合下会触发 RTX TLAS 显存预算和 Windows access violation，因此当前运行阶段使用低面数 G2 代理完成 PhysX 任务；`genie_g2_drywall.usda` 保留官方 G2 引用，适合在 Genie Sim 官方推荐的 Linux/Isaac Sim 5.1 环境打开并继续启用全身物理。

这意味着当前视频是“Isaac Sim 轨迹和 PhysX 任务的证据视频”，不是 Isaac Sim Replicator 原生逐帧渲染视频。USD 中仍有 `/World/RenderCamera`，后续切换到稳定的 Isaac Sim 5.1/Linux 图形环境后可以用该相机进行原生 RGB/深度渲染。

`C:\source\ExternalCalibration` 未被修改。D415 也没有参与本阶段；后续只需把 D415 的标定和传感器参数映射到 Isaac Sim 相机即可。
