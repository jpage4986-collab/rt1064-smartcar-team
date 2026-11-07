[CmdletBinding()]
param([string] $FlashScript = "$env:LOCALAPPDATA/Programs/RT1064Tools/flash-rt1064.ps1")
$ErrorActionPreference = 'Stop'
$env:PYTHONUTF8 = '1'
$flasher = (Resolve-Path -LiteralPath $FlashScript).Path
$bin = Join-Path (Split-Path -Parent $flasher) 'venv/Scripts'
$python = 'python'
$probe = 'pyocd'
if (Test-Path -LiteralPath "$bin/python.exe") {
    $python = "$bin/python.exe"
    $probe = "$bin/pyocd.exe"
}
& $probe list
if ($LASTEXITCODE -ne 0) { throw 'Probe enumeration failed' }
$project = Join-Path $PSScriptRoot 'Project'
$header = Get-Content -Raw -LiteralPath "$project/USER/inc/tune_config.h"
if ($header -notmatch '(?m)^#define\s+TRACK_ALGORITHM\s+(\d+)[uU]?\s*$') { throw 'Cannot read selected algorithm' }
$expectedAlgorithm = [int]$Matches[1]
& $python "$project/tools/wired-update.py" --elf "$project/GCC/wireless-app/tutorial.elf" --loader "$project/GCC/wireless-loader/tutorial.elf" --output-dir "$PSScriptRoot/reports/flash" --expect-algorithm $expectedAlgorithm --flash-script $flasher
if ($LASTEXITCODE -ne 0) { throw 'Firmware update or readback failed' }
