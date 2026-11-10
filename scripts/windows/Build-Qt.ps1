param(
    [string]$QtPrefix = $env:QT_PREFIX,
    [string]$OnnxRuntimeDir = $env:onnxruntime_DIR,
    [string]$MoveNetModel = $env:REMOLD_MOVENET_MODEL
)
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$workRoot = if ($env:REMOLD_WORK_ROOT) { $env:REMOLD_WORK_ROOT } else { Join-Path $env:LOCALAPPDATA 'Kinect Remold\Work' }
$work = Join-Path $workRoot 'windows\qt'
$output = Join-Path $repo 'binaries\windows\applications\KinectRemoldQt'
New-Item -ItemType Directory -Path @($work, $output) -Force | Out-Null
$configure = @('-S', (Join-Path $repo 'applications\qt'), '-B', $work)
if ($QtPrefix) { $configure += "-DCMAKE_PREFIX_PATH=$QtPrefix" }
if ($OnnxRuntimeDir) { $configure += "-Donnxruntime_DIR=$OnnxRuntimeDir" }
& cmake @configure
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& cmake --build $work --config Release --parallel
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$binary = @((Join-Path $work 'Release\KinectRemoldQt.exe'), (Join-Path $work 'KinectRemoldQt.exe')) |
    Where-Object { Test-Path -LiteralPath $_ -PathType Leaf } | Select-Object -First 1
if (-not $binary) { throw 'Qt executable was not produced.' }
Copy-Item -LiteralPath $binary -Destination (Join-Path $output 'KinectRemoldQt.exe') -Force
$licenseDir = Join-Path $output 'licenses\libfreenect2'
New-Item -ItemType Directory -Path $licenseDir -Force | Out-Null
Copy-Item -Path (Join-Path $repo 'third_party\licenses\libfreenect2\*') -Destination $licenseDir -Force
$deploy = if ($QtPrefix -and (Test-Path -LiteralPath (Join-Path $QtPrefix 'bin\windeployqt.exe'))) {
    Join-Path $QtPrefix 'bin\windeployqt.exe'
} else { 'windeployqt.exe' }
& $deploy --release (Join-Path $output 'KinectRemoldQt.exe')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$runtimePackage = $OnnxRuntimeDir
if (-not $runtimePackage) {
    $cacheLine = Get-Content -LiteralPath (Join-Path $work 'CMakeCache.txt') |
        Where-Object { $_ -match '^onnxruntime_DIR:PATH=' } | Select-Object -First 1
    if ($cacheLine) { $runtimePackage = ($cacheLine -split '=', 2)[1] }
}
if (-not $runtimePackage) { throw 'ONNX Runtime package path is unavailable.' }
$runtimeSearch = $runtimePackage
$runtimeDll = $null
for ($level = 0; $level -lt 4 -and -not $runtimeDll; $level++) {
    $runtimeDll = Get-ChildItem -Path $runtimeSearch -Filter onnxruntime.dll -Recurse -File |
        Select-Object -First 1
    $runtimeSearch = Split-Path $runtimeSearch -Parent
}
if (-not $runtimeDll) { throw 'ONNX Runtime DLL was not found for packaging.' }
Copy-Item -LiteralPath $runtimeDll.FullName -Destination $output -Force
$settings = @{}
Get-Content -LiteralPath (Join-Path $repo 'config\build.env') | ForEach-Object {
    if ($_ -match '^([A-Z][A-Z0-9_]*)=(.*)$') { $settings[$matches[1]] = $matches[2] }
}
$models = Join-Path $output 'models'
New-Item -ItemType Directory -Force $models | Out-Null
function Install-PinnedModel([string]$name, [string]$url, [string]$expected) {
    $dest = Join-Path $models $name
    if ((Test-Path -LiteralPath $dest -PathType Leaf) -and
        (Get-FileHash -LiteralPath $dest -Algorithm SHA256).Hash.ToLowerInvariant() -eq $expected) { return }
    $temp = Join-Path $work "$name.download"
    try {
        Invoke-WebRequest -Uri $url -OutFile $temp -MaximumRedirection 8
        if ((Get-FileHash -LiteralPath $temp -Algorithm SHA256).Hash.ToLowerInvariant() -ne $expected) {
            throw "SHA-256 mismatch: $name"
        }
        Move-Item -LiteralPath $temp -Destination $dest -Force
    } finally { Remove-Item -LiteralPath $temp -Force -ErrorAction SilentlyContinue }
}
Install-PinnedModel 'movenet.onnx' $settings.REMOLD_MOVENET_MODEL_URL $settings.REMOLD_MOVENET_MODEL_SHA256
Install-PinnedModel 'rtmpose-hand.onnx' $settings.REMOLD_RTMPOSE_HAND_MODEL_URL $settings.REMOLD_RTMPOSE_HAND_MODEL_SHA256
Install-PinnedModel 'face-landmarks.onnx' $settings.REMOLD_FACE_LANDMARKS_ONNX_URL $settings.REMOLD_FACE_LANDMARKS_ONNX_SHA256
Copy-Item -LiteralPath (Join-Path $repo 'third_party\models\face-blendshapes.onnx') -Destination (Join-Path $models 'face-blendshapes.onnx') -Force
Copy-Item -LiteralPath (Join-Path $repo 'third_party\models\README.md') -Destination (Join-Path $models 'face-model-NOTICE.txt') -Force
Copy-Item -LiteralPath (Join-Path $repo 'third_party\models\APACHE-2.0-LICENSE') -Destination (Join-Path $models 'face-model-LICENSE.txt') -Force
if ($MoveNetModel) {
    if ((Get-FileHash -LiteralPath $MoveNetModel -Algorithm SHA256).Hash.ToLowerInvariant() -ne
        $settings.REMOLD_MOVENET_MODEL_SHA256) { throw 'MoveNet override SHA-256 mismatch.' }
    Copy-Item -LiteralPath $MoveNetModel -Destination (Join-Path $models 'movenet.onnx') -Force
}
if ($env:REMOLD_SYNSKELETON_MODEL) {
    $models = Join-Path $output 'models'
    New-Item -ItemType Directory -Force $models | Out-Null
    Copy-Item -LiteralPath $env:REMOLD_SYNSKELETON_MODEL -Destination (Join-Path $models 'SynSkeleton.onnx') -Force
}
Write-Host "Qt frontend built: $(Join-Path $output 'KinectRemoldQt.exe')"
