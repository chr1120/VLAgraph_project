param(
    [string]$BackendContainer = 'openvla_backend',
    [string]$FrontendContainer = 'openvla_frontend',
    [switch]$BackendOnly
)

$ErrorActionPreference = 'Stop'
$workspaceRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path

function Get-ComposeLabels([string]$Container) {
    $raw = & docker inspect --format '{{json .Config.Labels}}' $Container
    if ($LASTEXITCODE -ne 0) { throw "Cannot inspect container: $Container" }
    return ($raw | ConvertFrom-Json)
}

$backendLabels = Get-ComposeLabels $BackendContainer
$projectName = $backendLabels.'com.docker.compose.project'
if (-not $projectName -or $backendLabels.'com.docker.compose.service' -ne 'backend') {
    throw 'Expected a backend Compose service.'
}
if (-not $BackendOnly) {
    $frontendLabels = Get-ComposeLabels $FrontendContainer
    if ($frontendLabels.'com.docker.compose.project' -ne $projectName -or $frontendLabels.'com.docker.compose.service' -ne 'frontend') {
        throw 'Backend and frontend must belong to the same Compose project.'
    }
}

# Keep the active project, networks and data mounts; select workspace build paths.
$runtimeDir = Join-Path $workspaceRoot '.tmp/docker-runtime'
New-Item -ItemType Directory -Force -Path $runtimeDir | Out-Null
$managedOverrides = @(
    (Join-Path $runtimeDir 'workspace-build-override.json'),
    (Join-Path $runtimeDir 'workspace-backend-build-override.json')
)
$overridePath = if ($BackendOnly) { $managedOverrides[1] } else { $managedOverrides[0] }
$configFiles = @($backendLabels.'com.docker.compose.project.config_files' -split ',' |
    ForEach-Object { $_.Trim() } | Where-Object { $_ -and $_ -notin $managedOverrides })
if (-not $configFiles.Count) { throw 'No active Compose configuration was found.' }
foreach ($configFile in $configFiles) {
    if (-not (Test-Path -LiteralPath $configFile -PathType Leaf)) { throw "Compose file is missing: $configFile" }
}
# Preserve configured origins and allow both local workspace UI addresses.
$backendEnvRaw = & docker inspect --format '{{json .Config.Env}}' $BackendContainer
if ($LASTEXITCODE -ne 0) { throw 'Cannot inspect backend environment.' }
$originEntry = @(($backendEnvRaw | ConvertFrom-Json) | Where-Object { $_ -like 'ALLOW_ORIGINS=*' })
$origins = @(if ($originEntry.Count) { $originEntry[0].Substring(14) -split ',' } else { 'http://localhost:9173' })
$origins = @(($origins + @('http://localhost:9174', 'http://127.0.0.1:9174')) |
    ForEach-Object { $_.Trim() } | Where-Object { $_ } | Select-Object -Unique)
$services = @{
    backend = @{
        build = @{ context = $workspaceRoot; dockerfile = 'backend/Dockerfile' }
        environment = @{ ALLOW_ORIGINS = ($origins -join ',') }
    }
}
if (-not $BackendOnly) {
    $services.frontend = @{ build = @{ context = $workspaceRoot; dockerfile = 'frontend/Dockerfile' } }
}
$override = @{ services = $services }
$override | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath $overridePath -Encoding utf8

$composeArgs = @('compose', '-p', $projectName)
foreach ($configFile in $configFiles) { $composeArgs += @('-f', $configFile) }
$composeArgs += @('-f', $overridePath, 'up', '-d', '--build', '--no-deps', 'backend')
if (-not $BackendOnly) { $composeArgs += 'frontend' }
Write-Host "Reloading $projectName from workspace: $workspaceRoot"
& docker @composeArgs
if ($LASTEXITCODE -ne 0) { throw 'Workspace Docker build/reload failed.' }
if ($BackendOnly) { Write-Host 'Backend reloaded; frontend containers and original data mounts were retained.' }
else { Write-Host 'Backend and frontend reloaded; original data mounts and project settings were retained.' }
