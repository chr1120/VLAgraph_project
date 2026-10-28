# Modify these paths before starting Omniverse Isaac Sim.
$Executable = "C:\Users\0\AppData\Local\ov\pkg\isaac-sim-4.5.0\isaac-sim.bat"
$ScenePath = ""
$ExtraArguments = @()

if (-not (Test-Path -LiteralPath $Executable)) {
    Write-Error "Isaac Sim executable was not found: $Executable"
    exit 1
}

$argumentList = @()
if (-not [string]::IsNullOrWhiteSpace($ScenePath)) {
    if (-not (Test-Path -LiteralPath $ScenePath)) {
        Write-Error "Isaac Sim scene was not found: $ScenePath"
        exit 1
    }
    $argumentList += $ScenePath
}
$argumentList += $ExtraArguments

Start-Process -FilePath $Executable -ArgumentList $argumentList
Write-Output "Isaac Sim launch requested: $Executable"
