<#
.SYNOPSIS
  Installs the C++ dependencies of basalt-standalone with vcpkg (classic mode).

.DESCRIPTION
  Uses, in this order: -VcpkgRoot, $env:VCPKG_ROOT, or a private vcpkg that is
  cloned and bootstrapped into tools\vcpkg inside this repository.
  Then runs: vcpkg install eigen3 opencv4 --triplet x64-windows
  A fresh install compiles everything from source (roughly 20-40 min, ~6 GB);
  packages that are already installed are skipped within seconds.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File scripts\setup_vcpkg.ps1
  powershell -ExecutionPolicy Bypass -File scripts\setup_vcpkg.ps1 -VcpkgRoot C:\vcpkg
#>
param(
  [string]$VcpkgRoot = "",
  [string]$Triplet = "x64-windows"
)
$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
$deps = @("eigen3", "opencv4")

function Test-ClassicVcpkg([string]$root) {
  # The copy bundled with Visual Studio (...\VC\vcpkg) only supports manifest mode.
  return $root -and (Test-Path (Join-Path $root "vcpkg.exe")) -and ($root -notmatch '\\VC\\vcpkg')
}

if (-not $VcpkgRoot) {
  if (Test-ClassicVcpkg $env:VCPKG_ROOT) { $VcpkgRoot = $env:VCPKG_ROOT }
  else { $VcpkgRoot = Join-Path $repo "tools\vcpkg" }
}

if (-not (Test-Path (Join-Path $VcpkgRoot "vcpkg.exe"))) {
  if (-not (Test-Path (Join-Path $VcpkgRoot ".git"))) {
    Write-Host "Cloning vcpkg into $VcpkgRoot ..."
    git clone --depth 1 https://github.com/microsoft/vcpkg.git $VcpkgRoot
    if ($LASTEXITCODE -ne 0) { throw "git clone of vcpkg failed" }
  }
  Write-Host "Bootstrapping vcpkg ..."
  & (Join-Path $VcpkgRoot "bootstrap-vcpkg.bat") -disableMetrics
  if ($LASTEXITCODE -ne 0) { throw "bootstrap-vcpkg failed" }
}

$vcpkg = Join-Path $VcpkgRoot "vcpkg.exe"
Write-Host "Installing $($deps -join ', ') ($Triplet) with $vcpkg"
& $vcpkg install @deps --triplet $Triplet
if ($LASTEXITCODE -ne 0) { throw "vcpkg install failed (exit code $LASTEXITCODE)" }

$root = (Resolve-Path $VcpkgRoot).Path
Write-Host ""
Write-Host "Dependencies are installed in $root."
if ($root -ne (Resolve-Path (Join-Path $repo "tools\vcpkg") -ErrorAction SilentlyContinue).Path -and $root -ne $env:VCPKG_ROOT) {
  Write-Host "CMake finds vcpkg through VCPKG_ROOT; set it once with:  setx VCPKG_ROOT `"$root`"  (then open a new shell)"
} else {
  Write-Host "CMake will find this vcpkg automatically. Next step: scripts\build.ps1"
}
