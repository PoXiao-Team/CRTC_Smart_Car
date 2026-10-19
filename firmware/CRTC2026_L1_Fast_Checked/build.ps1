param([string]$Compiler)
$ErrorActionPreference = 'Stop'
$projectRoot = $PSScriptRoot
if (-not $Compiler) {
    $found = Get-ChildItem -LiteralPath 'C:\ST' -Filter 'arm-none-eabi-gcc.exe' -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
    if (-not $found) { throw '请使用 -Compiler 指定 CubeIDE 中 arm-none-eabi-gcc.exe 的完整路径。' }
    $Compiler = $found.FullName
}
$compilerBin = Split-Path -Parent $Compiler
$buildRoot = Join-Path $projectRoot 'build'
New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null
$includePaths = @('Core/Inc','Drivers/HAL/Inc','Drivers/CMSIS/Core/Include','Drivers/CMSIS/Device/Include')
$flags = @('-mcpu=cortex-m3','-mthumb','-std=c11','-Os','-g3','-Wall','-Wextra','-Werror', '-ffunction-sections','-fdata-sections','-DSTM32F103xB','-DUSE_HAL_DRIVER')
foreach ($dir in $includePaths) { $flags += '-I' + (Join-Path $projectRoot $dir) }
$sources = @((Join-Path $projectRoot 'Core/Src/robot.c'),(Join-Path $projectRoot 'Core/Src/main.c'),(Join-Path $projectRoot 'Core/Src/system_stm32f1xx.c'))
$sources += (Get-ChildItem -LiteralPath (Join-Path $projectRoot 'Drivers/HAL/Src') -Filter '*.c').FullName
$sources += Join-Path $projectRoot 'Core/Startup/startup_stm32f103xb.s'
$objects = @()
foreach ($source in $sources) {
    $obj = Join-Path $buildRoot (([IO.Path]::GetFileNameWithoutExtension($source)) + '.o')
    & $Compiler @flags -c $source -o $obj
    if ($LASTEXITCODE -ne 0) { throw "编译失败：$source" }
    $objects += $obj
}
$elf = Join-Path $buildRoot 'CRTC2026_F103C8_DRV8833.elf'
$map = Join-Path $buildRoot 'CRTC2026_F103C8_DRV8833.map'
$linkerScript = Join-Path $projectRoot 'STM32F103C8_FLASH.ld'
& $Compiler -mcpu=cortex-m3 -mthumb -nostartfiles @objects "-T$linkerScript" '--specs=nano.specs' '--specs=nosys.specs' '-Wl,--gc-sections' "-Wl,-Map=$map" '-Wl,--print-memory-usage' -o $elf
if ($LASTEXITCODE -ne 0) { throw '链接失败' }
& (Join-Path $compilerBin 'arm-none-eabi-objcopy.exe') -O ihex $elf (Join-Path $buildRoot 'CRTC2026_F103C8_DRV8833.hex')
if ($LASTEXITCODE -ne 0) { throw 'HEX 生成失败' }
& (Join-Path $compilerBin 'arm-none-eabi-size.exe') $elf
if ($LASTEXITCODE -ne 0) { throw '大小检查失败' }
Write-Output "翻铲接线固件构建完成：$elf；未进行实车验证；首次使能前卸下舵盘测试。"
