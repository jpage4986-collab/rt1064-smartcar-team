[CmdletBinding()]
param(
    [Parameter(Mandatory = $true, Position = 0)]
    [string] $Image,

    [UInt32] $BaseAddress
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$imagePath = (Resolve-Path -LiteralPath $Image).Path
$extension = [IO.Path]::GetExtension($imagePath).ToLowerInvariant()
$format = switch ($extension) {
    '.elf' { 'elf' }
    '.axf' { 'elf' }
    '.hex' { 'hex' }
    '.bin' { 'bin' }
    default { throw "Unsupported image format '$extension'. Use ELF, AXF, HEX, or BIN." }
}

if ($format -eq 'bin' -and -not $PSBoundParameters.ContainsKey('BaseAddress')) {
    throw 'A raw BIN has no load address. Supply -BaseAddress from the project linker script.'
}

$pyocd = Join-Path $PSScriptRoot 'venv\Scripts\pyocd.exe'
if (-not (Test-Path -LiteralPath $pyocd)) {
    throw "PyOCD was not found at '$pyocd'."
}

$arguments = @('load', '--target', 'mimxrt1064', '--format', $format)
if ($PSBoundParameters.ContainsKey('BaseAddress')) {
    $arguments += @('--base-address', ('0x{0:X}' -f $BaseAddress))
}
$arguments += $imagePath

Write-Host "Flashing $imagePath to MIMXRT1064 through the connected CMSIS-DAP probe..."
& $pyocd @arguments
if ($LASTEXITCODE -ne 0) {
    throw "PyOCD flash failed with exit code $LASTEXITCODE."
}
