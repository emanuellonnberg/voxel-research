@echo off
setlocal enabledelayedexpansion

set "BUILD_TYPE=Release"
set "SCENE="
set "BENCHMARK="
set "SCENE_SET=0"

:parse_args
set "ARG=%~1"
echo [DEBUG] Next arg: "!ARG!" (scene_set=%SCENE_SET%)
if "!ARG!"=="" goto after_args

if /I "!ARG!"=="-debug" (
    echo [DEBUG] Detected -debug
    set "BUILD_TYPE=Debug"
    shift
    goto parse_args
)

if /I "!ARG!"=="-release" (
    echo [DEBUG] Detected -release
    set "BUILD_TYPE=Release"
    shift
    goto parse_args
)

if /I "!ARG!"=="--scene" (
    echo [DEBUG] Detected --scene
    shift
    if "%~1"=="" (
        echo Missing value for --scene
        exit /b 1
    )
    set "SCENE=%~1"
    set "SCENE_SET=1"
    shift
    goto parse_args
)

if /I "!ARG!"=="--benchmark" (
    echo [DEBUG] Detected --benchmark
    set "BENCHMARK=--benchmark"
    shift
    goto parse_args
)

if "%SCENE_SET%"=="0" (
    echo [DEBUG] Assigning scene from positional argument
    set "SCENE=!ARG!"
    set "SCENE_SET=1"
    shift
    goto parse_args
) else (
    echo Unknown argument: !ARG!
    exit /b 1
)

:after_args
echo [DEBUG] Arguments parsed. BUILD_TYPE=%BUILD_TYPE%, SCENE=%SCENE%, BENCHMARK=%BENCHMARK%

set "SCRIPT_DIR=%~dp0"
set "REPO_ROOT=%SCRIPT_DIR%.."
set "BUILD_DIR=%REPO_ROOT%\build"
echo [DEBUG] SCRIPT_DIR=%SCRIPT_DIR%
echo [DEBUG] REPO_ROOT=%REPO_ROOT%
echo [DEBUG] BUILD_DIR=%BUILD_DIR%

if not exist "%BUILD_DIR%" (
    mkdir "%BUILD_DIR%"
)

if not exist "%BUILD_DIR%\CMakeCache.txt" (
    echo Configuring project with CMake (build type %BUILD_TYPE%)...
    cmake -S "%REPO_ROOT%" -B "%BUILD_DIR%" -DCMAKE_BUILD_TYPE=%BUILD_TYPE%
) else (
    echo Using existing CMake cache in build\.
)

echo Building VoxelDemo (%BUILD_TYPE%)...
cmake --build "%BUILD_DIR%" --config %BUILD_TYPE%
if errorlevel 1 exit /b 1

set "EXE=%BUILD_DIR%\bin\VoxelDemo.exe"
if not exist "%EXE%" (
    echo VoxelDemo executable not found at %EXE%
    exit /b 1
)

set "CMD=%EXE%"
if defined SCENE (
    set "CMD=%CMD% %SCENE%"
)
if defined BENCHMARK (
    set "CMD=%CMD% %BENCHMARK%"
)

echo Running %CMD%
pushd "%BUILD_DIR%\bin"
%CMD%
popd
echo [DEBUG] Demo finished with exit code %ERRORLEVEL%

endlocal
