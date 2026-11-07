[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
if (-not $env:ARMGCC_DIR) {
    $env:ARMGCC_DIR = (Get-Item (Get-Command arm-none-eabi-gcc).Source).Directory.Parent.FullName
}
foreach ($part in @('app','loader')) {
    $source = Join-Path $PSScriptRoot 'Project/GCC'
    $build = Join-Path $PSScriptRoot "build/$part"
    $option = '-DSU7_WIRELESS_' + $part.ToUpperInvariant() + '=ON'
    & cmake -S $source -B $build -G Ninja '-DCMAKE_TOOLCHAIN_FILE=armgcc.cmake' '-DCMAKE_POLICY_VERSION_MINIMUM=3.5' '-DCMAKE_BUILD_TYPE=debug' $option
    if ($LASTEXITCODE -ne 0) { throw "CMake configure failed for $part" }
    & cmake --build $build --parallel 8
    if ($LASTEXITCODE -ne 0) { throw "Firmware build failed for $part" }
}
