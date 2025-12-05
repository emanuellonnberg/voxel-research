param(
    [string]$BuildType = "Release",
    [string]$Scene = "",
    [switch]$Benchmark
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
if (-not (Test-Path $repoRoot)) {
    throw "Unable to determine repository root from script path."
}

$buildDir = Join-Path $repoRoot "build"
if (-not (Test-Path $buildDir)) {
    New-Item -Path $buildDir -ItemType Directory | Out-Null
}

$cmakeCache = Join-Path $buildDir "CMakeCache.txt"
if (-not (Test-Path $cmakeCache)) {
    Write-Host "Configuring project (cmake -S . -B build -DCMAKE_BUILD_TYPE=$BuildType) ..."
    cmake -S $repoRoot -B $buildDir -DCMAKE_BUILD_TYPE $BuildType
} else {
    Write-Host "CMake cache detected - skipping configure step."
}

Write-Host "Building VoxelDemo ($BuildType) ..."
cmake --build $buildDir --config $BuildType

$exePath = Join-Path $buildDir "bin/VoxelDemo.exe"
if (-not (Test-Path $exePath)) {
    throw "VoxelDemo executable not found at $exePath"
}

$args = @()
if ($Scene -ne "") {
    $args += $Scene
}
if ($Benchmark.IsPresent) {
    $args += "--benchmark"
}

Write-Host "Running VoxelDemo..." -ForegroundColor Green
Write-Host "$exePath $($args -join ' ')"
Push-Location (Split-Path $exePath -Parent)
try {
    & $exePath @args
} finally {
    Pop-Location
}
