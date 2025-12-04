cmake_minimum_required(VERSION 3.15)
project(VoxelDestruction VERSION 0.1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler warnings
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Options
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_EXAMPLES "Build example programs" ON)
option(USE_BULLET "Use Bullet Physics engine" ON)

# Bullet Physics (Week 5 Day 21)
if(USE_BULLET)
    message(STATUS "Including Bullet Physics from submodule")
    set(BUILD_BULLET2_DEMOS OFF CACHE BOOL "Build Bullet2 demos" FORCE)
    set(BUILD_BULLET3 OFF CACHE BOOL "Build Bullet3" FORCE)
    set(BUILD_CPU_DEMOS OFF CACHE BOOL "Build CPU demos" FORCE)
    set(BUILD_EXTRAS OFF CACHE BOOL "Build extras" FORCE)
    set(BUILD_OPENGL3_DEMOS OFF CACHE BOOL "Build OpenGL3 demos" FORCE)
    set(BUILD_UNIT_TESTS OFF CACHE BOOL "Build Bullet unit tests" FORCE)
    set(USE_GRAPHICAL_BENCHMARK OFF CACHE BOOL "Use graphical benchmark" FORCE)

    add_subdirectory(third_party/bullet3)
    set(BULLET_LIBRARIES BulletDynamics BulletCollision LinearMath)
    message(STATUS "Bullet Physics enabled")
else()
    message(STATUS "Bullet Physics disabled - using MockPhysicsEngine only")
endif()

# Find packages
find_package(OpenGL)
if(NOT OpenGL_FOUND)
    message(WARNING "OpenGL not found. Rendering features will be disabled (needed for Week 1 Day 3+)")
endif()

# GLM (header-only math library)
find_package(glm CONFIG)
if(NOT glm_FOUND)
    message(STATUS "GLM not found via find_package, will try include path")
    # Fallback: try to find GLM headers
    find_path(GLM_INCLUDE_DIR glm/glm.hpp
        PATHS
            /usr/include
            /usr/local/include
            ${CMAKE_SOURCE_DIR}/external/glm
    )
    if(GLM_INCLUDE_DIR)
        add_library(glm INTERFACE)
        target_include_directories(glm INTERFACE ${GLM_INCLUDE_DIR})
        message(STATUS "Found GLM at: ${GLM_INCLUDE_DIR}")
    else()
        message(WARNING "GLM not found. Will use custom Vector3 implementation.")
    endif()
endif()

# GLFW (windowing)
find_package(glfw3 CONFIG)
if(NOT glfw3_FOUND)
    message(STATUS "GLFW3 not found via find_package")
    # User needs to install GLFW3
endif()

# GoogleTest (for testing)
if(BUILD_TESTS)
    find_package(GTest CONFIG)
    if(NOT GTest_FOUND)
        message(STATUS "GoogleTest not found, downloading...")
        include(FetchContent)
        FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG release-1.12.1
        )
        FetchContent_MakeAvailable(googletest)
        enable_testing()
        include(GoogleTest)
    else()
        enable_testing()
        include(GoogleTest)
    endif()
endif()

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
)

# Core library
add_library(VoxelCore
    # Core utilities
    src/core/Vector3.cpp
    src/core/Material.cpp

    # Voxel system
    src/voxel/VoxelWorld.cpp
    src/voxel/VoxelUtils.cpp
    src/voxel/SpatialHash.cpp
    src/voxel/VoxelCluster.cpp
    src/voxel/VoxelStability.cpp

    # Rendering (Week 1 Day 3)
    src/rendering/Camera.cpp
    src/rendering/Renderer.cpp

    # Structural analysis (Week 3-4)
    src/structural/StructuralAnalyzer.cpp
    src/structural/ParameterTuning.cpp
    src/structural/VoxelFragmentation.cpp

    # Physics (Week 5 Day 22-23)
    $<$<BOOL:${USE_BULLET}>:src/physics/BulletEngine.cpp>
    src/physics/VoxelPhysicsIntegration.cpp
)

target_include_directories(VoxelCore PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
    $<$<BOOL:${USE_BULLET}>:${CMAKE_SOURCE_DIR}/third_party/bullet3/src>
)

# Link Bullet libraries if enabled
if(USE_BULLET)
    target_link_libraries(VoxelCore PUBLIC ${BULLET_LIBRARIES})
endif()

# Main executable
add_executable(VoxelDemo
    src/main.cpp
)
target_link_libraries(VoxelDemo VoxelCore)

# OpenGL/GLFW linking (if available)
if(OpenGL_FOUND AND glfw3_FOUND)
    target_link_libraries(VoxelDemo OpenGL::GL glfw)
endif()

# Tests
if(BUILD_TESTS)
    add_executable(VoxelTests
        tests/test_Vector3.cpp
        tests/test_Material.cpp
        tests/test_VoxelWorld.cpp
        tests/test_Camera.cpp
        tests/test_VoxelUtils.cpp
        tests/test_SpatialHash.cpp
        tests/test_VoxelCluster.cpp
        tests/test_VoxelStability.cpp
        tests/test_StructuralAnalyzer.cpp
        tests/test_PhysicsEngine.cpp
        tests/test_VoxelPhysicsIntegration.cpp
        tests/test_Integration.cpp
        tests/test_VoxelFragmentation.cpp
    )

    target_link_libraries(VoxelTests
        VoxelCore
        GTest::gtest
        GTest::gtest_main
    )

    # Define USE_BULLET for tests if enabled
    if(USE_BULLET)
        target_compile_definitions(VoxelTests PRIVATE USE_BULLET)
    endif()

    gtest_discover_tests(VoxelTests)
endif()

# Examples (Day 20: Benchmark suite)
if(BUILD_EXAMPLES)
    add_executable(BenchmarkStructuralAnalysis
        examples/benchmark_structural_analysis.cpp
    )
    target_link_libraries(BenchmarkStructuralAnalysis VoxelCore)

    # Day 24: Integration demo
    add_executable(IntegrationDemo
        examples/demo_integration.cpp
    )
    target_link_libraries(IntegrationDemo VoxelCore)

    # Define USE_BULLET for demo if enabled
    if(USE_BULLET)
        target_compile_definitions(IntegrationDemo PRIVATE USE_BULLET)
    endif()
endif()

# Print configuration
message(STATUS "")
message(STATUS "=== VoxelDestruction Configuration ===")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build tests: ${BUILD_TESTS}")
message(STATUS "Build examples: ${BUILD_EXAMPLES}")
message(STATUS "=====================================")
message(STATUS "")
