param(
    [int]$Frames = 360,
    [string]$OutputDir = '',
    [switch]$EnableG2Physics
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$isaacRoot = if ($env:ISAAC_SIM_ROOT) {
    $env:ISAAC_SIM_ROOT
} else {
    Join-Path $projectRoot 'third_party\isaac-sim-4.5.0'
}
$isaacPython = Join-Path $isaacRoot 'python.bat'
$demoScript = Join-Path $projectRoot 'sim\genie_g2_drywall_demo.py'
$asset = Join-Path $projectRoot 'third_party\geniesim_assets\robot\G2_omnipicker\robot.usda'
$renderScript = Join-Path $projectRoot 'tools\render_evidence_video.py'
$outputPath = if ($OutputDir) {
    [System.IO.Path]::GetFullPath($OutputDir)
} else {
    Join-Path $projectRoot 'outputs\genie_g2_drywall'
}

foreach ($required in @($isaacPython, $demoScript, $asset, $renderScript)) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) {
        throw "Required file not found: $required"
    }
}

New-Item -ItemType Directory -Force -Path $outputPath | Out-Null
$env:GENIE_G2_USD = $asset
$env:ISAAC_DEMO_OUTPUT = $outputPath
$env:ISAAC_DEMO_FRAMES = [string]$Frames

$simulationArgs = @(
    $demoScript,
    '--headless',
    '--frames', [string]$Frames,
    '--output', $outputPath
)
if ($EnableG2Physics) {
    $simulationArgs += '--enable-g2-physics'
}

Write-Host "[1/2] Running Isaac Sim task: $outputPath"
& $isaacPython @simulationArgs
if (-not (Test-Path -LiteralPath (Join-Path $outputPath 'trajectory.csv') -PathType Leaf)) {
    throw "Isaac Sim did not produce trajectory.csv"
}

$hostPython = (Get-Command python -ErrorAction Stop).Source
$trajectory = Join-Path $outputPath 'trajectory.csv'
$video = Join-Path $outputPath 'drywall_installation.mp4'
Write-Host '[2/2] Rendering evidence video from the recorded Isaac trajectory'
& $hostPython $renderScript $trajectory $video '--fps' '30'

Write-Host "[OK] USD composition: $(Join-Path $outputPath 'genie_g2_drywall.usda')"
Write-Host "[OK] Video: $video"
