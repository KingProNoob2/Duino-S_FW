param(
    [string]$Preset = "Debug"
)

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$BuildDir = Join-Path $ScriptDir "build\$Preset"
$BinFile = Join-Path $BuildDir "F407PLC.bin"

Set-Location $ScriptDir

if (-not (Test-Path $BuildDir)) {
    Write-Host "==> Configuring CMake preset: $Preset"
    cmake --preset $Preset
}

Write-Host "==> Building ($Preset)..."
cmake --build --preset $Preset

if (-not (Test-Path $BinFile)) {
    throw "BIN not found at $BinFile"
}

$dfuUtil = Get-Command dfu-util -ErrorAction SilentlyContinue
$cubeCli = Get-Command STM32_Programmer_CLI -ErrorAction SilentlyContinue
if (-not $cubeCli) {
    $defaultCube = "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
    if (Test-Path $defaultCube) { $cubeCli = Get-Command $defaultCube }
}

if ($dfuUtil) {
    Write-Host "==> Flashing $BinFile via dfu-util..."
    & $dfuUtil.Source -d 0483:df11 -a 0 -s 0x08000000:leave -D $BinFile
} elseif ($cubeCli) {
    Write-Host "==> Flashing $BinFile via STM32_Programmer_CLI..."
    & $cubeCli.Source -c port=USB1 -w $BinFile 0x08000000 -v -s
} else {
    throw "Neither dfu-util nor STM32_Programmer_CLI was found. Install STM32CubeProgrammer or dfu-util."
}

Write-Host "==> Done!"
