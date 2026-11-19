param([switch]$NoBuild, [string]$Ffmpeg = $env:REMOLD_FFMPEG_EXE)
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
if (-not $NoBuild) {
    & (Join-Path $PSScriptRoot 'Build-Qt.ps1')
    if ($LASTEXITCODE -ne 0) { throw 'Qt build failed.' }
    & (Join-Path $repo 'drivers\modules\kinect-one-remold\windows\source\Build.ps1')
    if ($LASTEXITCODE -ne 0) { throw 'Kinect v2 build failed.' }
}
$out = Join-Path $repo 'binaries\releases'
New-Item -ItemType Directory -Force $out | Out-Null
$temp = Join-Path ([IO.Path]::GetTempPath()) ('remold-release-' + [guid]::NewGuid())
$stage = Join-Path $temp 'KinectRemoldQt'
try {
    New-Item -ItemType Directory -Force $temp | Out-Null
    Copy-Item -LiteralPath (Join-Path $repo 'binaries\windows\applications\KinectRemoldQt') -Destination $stage -Recurse
    $driver = Join-Path $stage 'drivers\kinect-one-remold'
    New-Item -ItemType Directory -Force (Split-Path $driver) | Out-Null
    Copy-Item -LiteralPath (Join-Path $repo 'binaries\windows\drivers\kinect-one-remold') -Destination $driver -Recurse
    foreach ($name in @('KinectRemoldQt.exe','onnxruntime.dll','platforms\qwindows.dll','models\movenet.onnx','models\rtmpose-hand.onnx','models\face-landmarks.onnx','models\face-blendshapes.onnx')) {
        if (-not (Test-Path -LiteralPath (Join-Path $stage $name))) { throw "Missing Qt archive input: $name" }
    }
    foreach ($name in @('KinectOneRemoldService.exe','KinectOneRemoldWinUSB.inf','KinectOneRemoldWinUSB.cat','KinectRemoldDriver.cer','Install.ps1')) {
        if (-not (Test-Path -LiteralPath (Join-Path $driver $name))) { throw "Missing driver archive input: $name" }
    }
    if ($Ffmpeg) { Copy-Item -LiteralPath $Ffmpeg -Destination (Join-Path $stage 'ffmpeg.exe') }
    @'
$ErrorActionPreference = 'Stop'
$script = Join-Path $PSScriptRoot 'Install.ps1'
$process = Start-Process -FilePath 'powershell.exe' -Verb RunAs -Wait -PassThru -ArgumentList "-NoProfile -ExecutionPolicy Bypass -File `"$script`" -Action Install -NoPause"
exit $process.ExitCode
'@ | Set-Content -LiteralPath (Join-Path $driver 'Setup-Driver.ps1') -Encoding UTF8
    "@echo off`r`npowershell.exe -NoProfile -ExecutionPolicy Bypass -File `"%~dp0Setup-Driver.ps1`"`r`nif errorlevel 1 pause`r`n" |
        Set-Content -LiteralPath (Join-Path $driver 'SETUP.cmd') -Encoding ASCII
    Copy-Item -LiteralPath (Join-Path $repo 'docs\PORTABLE-DEPLOYMENT.md') -Destination (Join-Path $stage 'README.md')
    Copy-Item -LiteralPath (Join-Path $repo 'LICENSE') -Destination $stage
    $archive = Join-Path $out 'KinectRemoldQt-windows-amd64.zip'
    Compress-Archive -Path $stage -DestinationPath $archive -Force
    $hash = (Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash.ToLowerInvariant()
    "$hash  $([IO.Path]::GetFileName($archive))" | Set-Content -LiteralPath ($archive + '.sha256') -Encoding ASCII
    Write-Host $archive
} finally {
    if (Test-Path -LiteralPath $temp) { Remove-Item -LiteralPath $temp -Recurse -Force }
}
