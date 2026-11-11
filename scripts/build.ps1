<#
.SYNOPSIS
  Configures and builds basalt-standalone (MSVC + vcpkg, CMake preset "msvc-vcpkg")
  and runs the Jacobian unit tests.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File scripts\build.ps1
#>
param(
  [ValidateSet("Release", "Debug")]
  [string]$Config = "Release"
)
$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
Push-Location $repo
try {
  $start = Get-Date
  cmake --preset msvc-vcpkg
  if ($LASTEXITCODE -ne 0) { throw "CMake configure failed (is vcpkg installed? see scripts\setup_vcpkg.ps1)" }
  cmake --build build --config $Config --parallel
  if ($LASTEXITCODE -ne 0) { throw "Build failed" }
  $exe = Join-Path $repo "build\$Config\bw_vio.exe"
  if (-not (Test-Path $exe)) { throw "Build finished but $exe was not found" }
  Write-Host ("Built {0} in {1:N0} s" -f $exe, ((Get-Date) - $start).TotalSeconds)
  Write-Host "Running unit tests ..."
  & (Join-Path $repo "build\$Config\bw_tests.exe")
  if ($LASTEXITCODE -ne 0) { throw "Unit tests failed" }
} finally {
  Pop-Location
}
