# build_pico2w.ps1
# Script to build savia_c firmware for Raspberry Pi Pico 2 W (RP2350)

# Set PICO_SDK_PATH if not already set in environment
if (-not $env:PICO_SDK_PATH) {
    $env:PICO_SDK_PATH = "C:\Users\alexy\pico-sdk"
    Write-Host "Setting PICO_SDK_PATH to $env:PICO_SDK_PATH" -ForegroundColor Gray
}

$BuildDir = "build-pico2_w"

# VS Developer Command Prompt environment batch file path
$VcvarsPath = "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"

# Fallback to VS 2022 standard path if 18.x directory isn't found
if (-not (Test-Path $VcvarsPath)) {
    $VcvarsPath = "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
}

Write-Host "Starting build process for Pico 2 W..." -ForegroundColor Cyan

if (Test-Path $VcvarsPath) {
    # Run through cmd to ensure MSVC compiler environment is loaded correctly
    cmd.exe /c "`"$VcvarsPath`" && cmake -B $BuildDir -G Ninja -DPICO_BOARD=pico2_w -DSAVIA_ENABLE_BLE=ON -DSAVIA_ON_DEVICE_INFERENCE=ON -DPICOTOOL_FORCE_FETCH_FROM_GIT=ON && ninja -C $BuildDir"
} else {
    # Attempt direct build if already running inside VS Developer PowerShell
    Write-Host "vcvars64.bat not found. Attempting direct cmake build..." -ForegroundColor Yellow
    cmake -B $BuildDir -G Ninja -DPICO_BOARD=pico2_w -DPICOTOOL_FORCE_FETCH_FROM_GIT=ON
    ninja -C $BuildDir
}

if ($LASTEXITCODE -eq 0) {
    Write-Host "`nBuild complete! UF2 binary is located at:" -ForegroundColor Green
    Write-Host "$BuildDir/savia_c-pico2_w-mlONdevice.uf2" -ForegroundColor White
} else {
    Write-Error "Build failed with exit code $LASTEXITCODE"
}
