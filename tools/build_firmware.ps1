param(
    [ValidateSet("All", "V3F", "V5F")]
    [string]$Core = "All",
    [switch]$Rebuild
)

$ErrorActionPreference = "Stop"

# Post-competition engineering improvement; not flight-validated.
# This script only makes the generated MounRiver build entry points reproducible.
$repoRoot = Split-Path -Parent $PSScriptRoot
$makeCandidates = @(
    "D:\mingw64\bin\mingw32-make.exe",
    "mingw32-make.exe",
    "make.exe"
)
$toolchainCandidates = @(
    $env:WCH_TOOLCHAIN_BIN,
    "D:\MounRiver\MounRiver_Studio2\resources\app\resources\win32\components\WCH\Toolchain\RISC-V Embedded GCC12\bin"
)

$makeExe = $null
foreach ($candidate in $makeCandidates) {
    if (-not $candidate) { continue }
    if (Test-Path -LiteralPath $candidate -PathType Leaf) {
        $makeExe = (Resolve-Path -LiteralPath $candidate).Path
        break
    }
    $command = Get-Command $candidate -ErrorAction SilentlyContinue
    if ($command) {
        $makeExe = $command.Source
        break
    }
}
if (-not $makeExe) {
    throw "GNU Make not found. Install MounRiver Studio/MinGW or add make to PATH."
}

$toolchainBin = $null
foreach ($candidate in $toolchainCandidates) {
    if ($candidate -and (Test-Path -LiteralPath $candidate -PathType Container)) {
        $toolchainBin = (Resolve-Path -LiteralPath $candidate).Path
        break
    }
}
if (-not $toolchainBin) {
    $compiler = Get-Command "riscv-wch-elf-gcc.exe" -ErrorAction SilentlyContinue
    if ($compiler) { $toolchainBin = Split-Path -Parent $compiler.Source }
}
if (-not $toolchainBin) {
    throw "WCH RISC-V GCC not found. Set WCH_TOOLCHAIN_BIN to its bin directory."
}

$env:Path = $toolchainBin + [IO.Path]::PathSeparator + $env:Path
$cores = if ($Core -eq "All") { @("V3F", "V5F") } else { @($Core) }
$makeArgs = @()
if ($Rebuild) { $makeArgs += "-B" }
$makeArgs += @("-j1", "all")

foreach ($selectedCore in $cores) {
    $objDir = Join-Path $repoRoot "EXAM\GPIO\GPIO_Toggle\$selectedCore\obj"
    $generatedMakefile = Join-Path $objDir "Makefile"
    if (-not (Test-Path -LiteralPath $generatedMakefile -PathType Leaf)) {
        throw "$selectedCore generated Makefile not found at '$generatedMakefile'. Open GPIO_Toggle.wvsln in MounRiver Studio 2 and build/generate that core once before using this helper."
    }
    Write-Host "== Building $selectedCore =="
    & $makeExe -C $objDir @makeArgs
    if ($LASTEXITCODE -ne 0) {
        throw "$selectedCore build failed with exit code $LASTEXITCODE."
    }
}
