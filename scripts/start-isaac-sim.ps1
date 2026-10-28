$FolderPath = "C:\Users\0\Desktop\vla_chr\VLAgraph_project"

if (-not (Test-Path -LiteralPath $FolderPath -PathType Container)) {
    Write-Error "Folder was not found: $FolderPath"
    exit 1
}

Start-Process -FilePath "explorer.exe" -ArgumentList $FolderPath
Write-Output "Folder opened: $FolderPath"
