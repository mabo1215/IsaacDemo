param(
    [string]$DistroName = 'IsaacUbuntu2204',
    [string]$InstallRoot = 'C:\source\IsaacDemo\third_party\wsl\IsaacUbuntu2204'
)

$ErrorActionPreference = 'Stop'
$rootfsDir = 'C:\source\IsaacDemo\third_party'
$rootfs = Join-Path $rootfsDir 'ubuntu-jammy-wsl-amd64-ubuntu22.04lts.rootfs.tar.gz'
$url = 'https://cloud-images.ubuntu.com/wsl/jammy/current/ubuntu-jammy-wsl-amd64-ubuntu22.04lts.rootfs.tar.gz'

$existing = @(wsl.exe -l -q 2>$null | ForEach-Object { $_.Trim([char]0) } | Where-Object { $_ })
if ($existing -contains $DistroName) {
    Write-Output "WSL distro already exists: $DistroName"
    exit 0
}

New-Item -ItemType Directory -Force -Path $rootfsDir | Out-Null
if (-not (Test-Path -LiteralPath $rootfs)) {
    Write-Output "Downloading official Ubuntu 22.04 WSL rootfs..."
    curl.exe -L --fail --retry 5 --retry-delay 5 -C - -o $rootfs $url
}

if (Test-Path -LiteralPath $InstallRoot) {
    throw "Install root already exists but distro is not registered: $InstallRoot"
}
New-Item -ItemType Directory -Force -Path $InstallRoot | Out-Null
wsl.exe --import $DistroName $InstallRoot $rootfs --version 2
wsl.exe -d $DistroName -- bash -lc "printf '[boot]\ncommand=\n' > /etc/wsl.conf"
Write-Output "Imported $DistroName. Run: wsl.exe -d $DistroName -- bash"
