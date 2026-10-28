$ErrorActionPreference = "Stop"

$LauncherToken = $env:OMNIVERSE_LAUNCH_TOKEN
if ([string]::IsNullOrWhiteSpace($LauncherToken)) {
    throw "OMNIVERSE_LAUNCH_TOKEN is not set. Use the same value in landing/.env and this PowerShell session."
}

$ProjectRoot = Split-Path -Parent $PSScriptRoot
$LaunchScript = Join-Path $PSScriptRoot "start-isaac-sim.ps1"
$LauncherServer = Join-Path $PSScriptRoot "omniverse_launcher_server.py"
$PythonExecutable = Join-Path $ProjectRoot "clean_env\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $PythonExecutable)) {
    $PythonExecutable = "python"
}

$env:OMNIVERSE_LAUNCHER_HOST = "127.0.0.1"
$env:OMNIVERSE_LAUNCHER_PORT = "9188"
$env:OMNIVERSE_LAUNCHER_TOKEN = $LauncherToken
$env:OMNIVERSE_LAUNCH_SCRIPT = $LaunchScript

Write-Host "Starting Omniverse launcher on http://127.0.0.1:9188"
& $PythonExecutable $LauncherServer
