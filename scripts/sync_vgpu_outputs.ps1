[CmdletBinding(SupportsShouldProcess)]
param(
    [string]$EnvFile = '',
    [string]$RemoteOutputRoot = '/root/autodl-tmp/IsaacDemo_g2/outputs',
    [string]$RunName = 'g2_official_drywall_final',
    [string]$LocalRunName = 'g2_official_drywall',
    [string]$VideoName = 'genie_g2_official_drywall_installation.mp4',
    [switch]$IncludeTaskUsd,
    [switch]$IncludeFrames,
    [switch]$EncodeVideo
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path

if (-not $EnvFile) {
    $EnvFile = Join-Path (Split-Path -Parent $projectRoot) '.env'
}
if (-not (Test-Path -LiteralPath $EnvFile -PathType Leaf)) {
    throw "Credential file not found: $EnvFile"
}

foreach ($name in @('RunName', 'LocalRunName')) {
    $value = (Get-Variable -Name $name).Value
    if ($value -notmatch '^[A-Za-z0-9_-]+$') {
        throw "$name must contain only letters, numbers, '_' or '-'"
    }
}
if ($VideoName -notmatch '^[A-Za-z0-9_.-]+\.mp4$') {
    throw 'VideoName must be a simple MP4 filename'
}
if ($RemoteOutputRoot -notmatch '^/[A-Za-z0-9_./-]+$') {
    throw 'RemoteOutputRoot must be an absolute POSIX path without shell metacharacters'
}
if ($EncodeVideo -and -not $IncludeFrames) {
    throw '-EncodeVideo requires -IncludeFrames'
}

$envLines = @(Get-Content -LiteralPath $EnvFile)
$markerIndex = [Array]::IndexOf($envLines, 'vGPU 3090')
if ($markerIndex -lt 0 -or $markerIndex + 2 -ge $envLines.Count) {
    throw "The vGPU 3090 entry must be followed by an SSH login line and a password line in $EnvFile"
}
$loginLine = ([string]$envLines[$markerIndex + 1]).Trim()
$password = ([string]$envLines[$markerIndex + 2]).Trim()
$endpointMatch = [regex]::Match($loginLine, '(?<user>[^@\s]+)@(?<host>[^\s]+)')
if (-not $endpointMatch.Success) {
    throw 'Could not parse user@host from the vGPU 3090 login line'
}
$remoteUser = $endpointMatch.Groups['user'].Value
$remoteHost = $endpointMatch.Groups['host'].Value
$portMatch = [regex]::Match($loginLine, '(?:-P|-p|--port)\s+(?<port>\d+)')
$remotePort = if ($portMatch.Success) { [int]$portMatch.Groups['port'].Value } else { 22 }

$pscp = Join-Path ${env:ProgramFiles} 'PuTTY\pscp.exe'
$plink = Join-Path ${env:ProgramFiles} 'PuTTY\plink.exe'
if (-not (Test-Path -LiteralPath $pscp -PathType Leaf)) {
    throw "pscp.exe not found: $pscp"
}
if (-not (Test-Path -LiteralPath $plink -PathType Leaf)) {
    throw "plink.exe not found: $plink"
}

$hostKey = 'SHA256:liZ36vNCsNcNdXeWs4f+g5ZIhPM/ZihP834vxs8Ulqc'
$sshArgs = @('-batch', '-hostkey', $hostKey, '-P', [string]$remotePort, '-l', $remoteUser, '-pw', $password)
$remoteBase = "$($RemoteOutputRoot.TrimEnd('/'))/$RunName"
$localRun = Join-Path (Join-Path $projectRoot 'outputs\demo') $LocalRunName
$localFrames = Join-Path $localRun 'frames'
$videoPath = Join-Path (Join-Path $projectRoot 'outputs\demo') $VideoName

$probe = "test -d '$remoteBase' && test -f '$remoteBase/run_summary.json' && test -f '$remoteBase/trajectory.csv'"
$null = & $plink @sshArgs $remoteHost $probe
if ($LASTEXITCODE -ne 0) {
    throw "Remote output is missing or incomplete: $remoteBase"
}

New-Item -ItemType Directory -Force -Path $localRun | Out-Null

function Copy-RemoteFile {
    param([Parameter(Mandatory)][string]$Name)

    $remoteFile = "$remoteBase/$Name"
    $destination = Join-Path $localRun $Name
    if ($PSCmdlet.ShouldProcess($destination, "download $remoteFile from vGPU")) {
        $null = & $pscp @sshArgs "${remoteUser}@${remoteHost}:$remoteFile" $destination
        if ($LASTEXITCODE -ne 0) {
            throw "Failed to download $remoteFile"
        }
    }
}

Copy-RemoteFile -Name 'run_summary.json'
Copy-RemoteFile -Name 'trajectory.csv'

if ($IncludeTaskUsd) {
    Copy-RemoteFile -Name 'genie_g2_official_drywall.usd'
}

if ($IncludeFrames) {
    if (Test-Path -LiteralPath $localFrames) {
        $resolvedFrames = (Resolve-Path -LiteralPath $localFrames).Path
        $resolvedRun = (Resolve-Path -LiteralPath $localRun).Path
        if (-not $resolvedFrames.StartsWith($resolvedRun.TrimEnd('\') + '\', [System.StringComparison]::OrdinalIgnoreCase)) {
            throw "Refusing to remove frames outside the synchronized run directory: $resolvedFrames"
        }
        [System.IO.Directory]::Delete($resolvedFrames, $true)
    }
    New-Item -ItemType Directory -Force -Path $localFrames | Out-Null
    $remoteArchive = "$remoteBase/frames.tgz"
    $archiveCommand = "test -d '$remoteBase/frames' && tar -czf '$remoteArchive.tmp' -C '$remoteBase' frames && mv -f '$remoteArchive.tmp' '$remoteArchive'"
    if ($PSCmdlet.ShouldProcess($remoteArchive, 'create a compressed frame archive on vGPU')) {
        $null = & $plink @sshArgs $remoteHost $archiveCommand
        if ($LASTEXITCODE -ne 0) {
            throw "Failed to archive frames on vGPU: $remoteBase/frames"
        }
    }
    $localArchive = Join-Path $localRun 'frames.tgz'
    if ($PSCmdlet.ShouldProcess($localArchive, "download $remoteArchive from vGPU")) {
        $null = & $pscp @sshArgs "${remoteUser}@${remoteHost}:$remoteArchive" $localArchive
        if ($LASTEXITCODE -ne 0) {
            throw "Failed to download frame archive: $remoteArchive"
        }
    }
    $tar = (Get-Command tar -ErrorAction Stop).Source
    if ($PSCmdlet.ShouldProcess($localRun, 'extract the synchronized frame archive')) {
        & $tar '-xzf' $localArchive '-C' $localRun
        if ($LASTEXITCODE -ne 0) {
            throw "Failed to extract frame archive: $localArchive"
        }
        [System.IO.File]::Delete($localArchive)
    }
}

if ($EncodeVideo) {
    $summary = Get-Content -LiteralPath (Join-Path $localRun 'run_summary.json') -Raw | ConvertFrom-Json
    $fps = if ($summary.fps) { [int]$summary.fps } else { 30 }
    $python = (Get-Command python -ErrorAction Stop).Source
    $encoder = Join-Path $projectRoot 'tools\encode_video.py'
    if (-not (Test-Path -LiteralPath $encoder -PathType Leaf)) {
        throw "Video encoder not found: $encoder"
    }
    if ($PSCmdlet.ShouldProcess($videoPath, "encode synchronized vGPU frames at ${fps} FPS")) {
        & $python $encoder $localFrames $videoPath '--fps' ([string]$fps)
        if ($LASTEXITCODE -ne 0) {
            throw 'Local video encoding failed'
        }
    }
}

Write-Host "[OK] vGPU output synchronized to: $localRun"
if ($IncludeTaskUsd) {
    Write-Host "[OK] task USD: $(Join-Path $localRun 'genie_g2_official_drywall.usd')"
}
if ($EncodeVideo) {
    Write-Host "[OK] video: $videoPath"
}
