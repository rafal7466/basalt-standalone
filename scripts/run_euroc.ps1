<#
.SYNOPSIS
  Runs bw_vio.exe (stereo or monocular VIO) on one EuRoC sequence.

.PARAMETER Dataset
  EuRoC sequence in ASL format: the folder that contains mav0\ (e.g. dataset\MH_01_easy),
  the mav0 folder itself, or a flattened folder with cam0\ cam1\ imu0\.
.PARAMETER Mode
  stereo  stereo-inertial odometry (default, as Basalt's EuRoC setup)
  mono    left camera + IMU only
.PARAMETER Name
  Name of the run; outputs go to results\<Name>\. Default: <sequence>_<mode>.
.PARAMETER Open
  Open the HTML report when the run is finished.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File scripts\run_euroc.ps1 -Dataset dataset\MH_01_easy -Mode stereo
#>
param(
  [Parameter(Mandatory = $true)][string]$Dataset,
  [ValidateSet("stereo", "mono")]
  [string]$Mode = "stereo",
  [string]$Name = "",
  [string]$OutputDir = "",
  [switch]$Open
)
$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
$exe = Join-Path $repo "build\Release\bw_vio.exe"
if (-not (Test-Path $exe)) { throw "$exe not found - build first with scripts\build.ps1" }
if (-not (Test-Path $Dataset)) { throw "Dataset folder '$Dataset' not found (see the Dataset section of the README)" }
$ds = (Resolve-Path $Dataset).Path
if ($Name -eq "") {
  $leaf = Split-Path -Leaf $ds
  if ($leaf -eq "mav0") { $leaf = Split-Path -Leaf (Split-Path -Parent $ds) }
  $Name = "${leaf}_$Mode"
}
if ($OutputDir -eq "") { $OutputDir = Join-Path $repo "results\$Name" }
$cmdArgs = @("--dataset", $ds, "--out", $OutputDir, "--name", $Name)
if ($Mode -eq "mono") { $cmdArgs += "--mono" }
Write-Host "$exe $($cmdArgs -join ' ')"
& $exe @cmdArgs
if ($LASTEXITCODE -ne 0) { throw "bw_vio.exe exited with code $LASTEXITCODE" }
$report = Join-Path $OutputDir "report.html"
if ($Open -and (Test-Path $report)) { Start-Process $report }
