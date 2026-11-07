[CmdletBinding()]
param([string] $Destination = "$env:LOCALAPPDATA/Programs/RT1064Tools", [string] $Python = 'python')
$ErrorActionPreference = 'Stop'
$env:PYTHONUTF8 = '1'
$venv = Join-Path $Destination 'venv'
$exe = Join-Path $venv 'Scripts/python.exe'
if (-not (Test-Path -LiteralPath $exe)) {
    & $Python -m venv $venv
    if ($LASTEXITCODE) { throw 'Python venv creation failed' }
}
$wheelOptions = @()
if (Test-Path -LiteralPath "$PSScriptRoot/wheels") {
    $wheelOptions = @('--no-index','--find-links', "$PSScriptRoot/wheels")
}
& $exe -m pip install @wheelOptions -r "$PSScriptRoot/requirements-win.txt"
if ($LASTEXITCODE) { throw 'Installing the RT1064 flash dependencies failed' }
& $exe -m pip check
if ($LASTEXITCODE) { throw 'Python dependencies conflict' }
Copy-Item -LiteralPath "$PSScriptRoot/flash-rt1064.ps1" -Destination $Destination -Force
& $exe -m pyocd list --targets --name mimxrt1064
if ($LASTEXITCODE) { throw 'pyOCD target lookup failed' }
