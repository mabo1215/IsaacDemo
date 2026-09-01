param(
    [string]$IsaacRoot = $env:ISAAC_SIM_ROOT,
    [string]$OutputDir = 'C:\source\IsaacDemo\outputs',
    [int]$Frames = 360,
    [switch]$EnableRosBridge
)

$ErrorActionPreference = 'Stop'
if (-not $IsaacRoot) {
    throw 'Set ISAAC_SIM_ROOT to the Isaac Sim 4.5.0 installation root.'
}
$python = Join-Path $IsaacRoot 'python.bat'
if (-not (Test-Path -LiteralPath $python)) {
    throw "Isaac Sim python.bat not found: $python"
}

New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null
$env:ROS_DOMAIN_ID = if ($env:ROS_DOMAIN_ID) { $env:ROS_DOMAIN_ID } else { '42' }
$env:RMW_IMPLEMENTATION = if ($env:RMW_IMPLEMENTATION) { $env:RMW_IMPLEMENTATION } else { 'rmw_fastrtps_cpp' }
$env:ROS_DISTRO = if ($env:ROS_DISTRO) { $env:ROS_DISTRO } else { 'humble' }
$rosBridgeLib = Join-Path $IsaacRoot 'exts\isaacsim.ros2.bridge\humble\lib'
if (Test-Path -LiteralPath $rosBridgeLib) {
    $env:PATH = $rosBridgeLib + ';' + $env:PATH
}
$env:ISAAC_DEMO_SKIP_ROS_GRAPH = if ($EnableRosBridge) { '0' } else { '1' }
$env:ISAAC_DEMO_OUTPUT = $OutputDir
$env:ISAAC_DEMO_FRAMES = [string]$Frames
$scriptPath = Join-Path (Split-Path -Parent $PSScriptRoot) 'sim\humanoid_drywall_demo.py'

$isaacArgs = @('--headless')
if ($EnableRosBridge) {
    $isaacArgs += @('--enable', 'isaacsim.ros2.bridge', '--enable', 'isaacsim.sensors.physics')
}
$savedErrorActionPreference = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
& $python $scriptPath @isaacArgs
$isaacExitCode = $LASTEXITCODE
$ErrorActionPreference = $savedErrorActionPreference
if ($isaacExitCode -ne 0) { throw "Isaac Sim demo failed with exit code $isaacExitCode" }

$video = Join-Path $OutputDir 'drywall_installation.mp4'
$trajectory = Join-Path $OutputDir 'trajectory.csv'
python (Join-Path (Split-Path -Parent $PSScriptRoot) 'tools\render_evidence_video.py') $trajectory $video --fps 30
if ($LASTEXITCODE -ne 0) { throw "Video encoding failed with exit code $LASTEXITCODE" }
Write-Output "Video: $video"
