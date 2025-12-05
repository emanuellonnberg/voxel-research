# Voxel Destruction Engine - Quick Start Guide

**Week 8 Day 36: Final Documentation**

This guide will get you up and running with the Voxel Destruction Engine in 15 minutes.

---

## Table of Contents

1. [Installation](#installation)
2. [First Example: Simple Tower Collapse](#first-example-simple-tower-collapse)
3. [Understanding the Pipeline](#understanding-the-pipeline)
4. [Common Use Cases](#common-use-cases)
5. [Running the Demos](#running-the-demos)
6. [Next Steps](#next-steps)

---

## Installation

### Prerequisites

- C++17 compatible compiler (GCC 7+, Clang 5+, MSVC 2017+)
- CMake 3.15+
- Git (for submodules)

### Build Steps

```bash
# Clone the repository
git clone https://github.com/yourusername/voxel-research.git
cd voxel-research

# Initialize submodules (Bullet Physics)
git submodule update --init --recursive

# Build
mkdir build && cd build
cmake ..
cmake --build . -j4

# Run tests to verify installation
./bin/VoxelTests
```

**Expected output:**
```
[==========] Running 422 tests from 18 test suites.
[==========] 422 tests from 18 test suites ran.
[  PASSED  ] 422 tests.
```

---

## First Example: Simple Tower Collapse

This example demonstrates the complete destruction pipeline in ~30 lines of code.

### Create `my_first_destruction.cpp`:

```cpp
#include "VoxelWorld.h"
#include "StructuralAnalyzer.h"           // Spring-based analyzer
// #include "MaxFlowStructuralAnalyzer.h" // Max-flow analyzer (faster)
#include "VoxelPhysicsIntegration.h"
#include "BulletEngine.h"
#include <iostream>

int main() {
    // Step 1: Create a voxel world
    VoxelWorld world;

    // Step 2: Build a 10-voxel tower
    for (int y = 0; y < 10; y++) {
        world.SetVoxel(
            Vector3(0, y * 0.05f, 0),
            Voxel(MaterialDatabase::CONCRETE)
        );
    }

    std::cout << "Created tower with " << world.GetVoxelCount() << " voxels\n";

    // Step 3: Destroy the base
    std::vector<Vector3> damaged = {
        Vector3(0, 0, 0),
        Vector3(0, 0.05f, 0)
    };

    for (const auto& pos : damaged) {
        world.RemoveVoxel(pos);
    }

    std::cout << "Destroyed base, remaining: " << world.GetVoxelCount() << " voxels\n";

    // Step 4: Run structural analysis (Track 1)
    // Choose analyzer:
    // Option 1: Spring System (easy to tune, more parameters)
    StructuralAnalyzer analyzer;
    // Option 2: Max-Flow Algorithm (23x faster, deterministic)
    // MaxFlowStructuralAnalyzer analyzer;

    auto result = analyzer.Analyze(world, damaged, 500.0f);

    std::cout << "Structure failed: " << (result.structure_failed ? "YES" : "NO") << "\n";
    std::cout << "Failed clusters: " << result.failed_clusters.size() << "\n";
    std::cout << "Analysis time: " << result.calculation_time_ms << " ms\n";

    // Step 5: Simulate physics if structure failed (Track 3)
    if (result.structure_failed) {
        BulletEngine physics;
        physics.Initialize();
        physics.SetGravity(Vector3(0, -9.81f, 0));

        VoxelPhysicsIntegration integration(&physics, &world);
        integration.SpawnDebris(result.failed_clusters);

        std::cout << "\nSimulating collapse...\n";

        // Simulate 3 seconds (180 frames at 60 FPS)
        for (int i = 0; i < 180; i++) {
            integration.Step(1.0f / 60.0f);

            if (i % 60 == 0) {
                std::cout << "  " << (i / 60) << "s - Active debris: "
                          << integration.GetActiveDebrisCount() << "\n";
            }
        }

        physics.Shutdown();
        std::cout << "\nCollapse simulation complete!\n";
    }

    return 0;
}
```

### Add to CMakeLists.txt:

```cmake
add_executable(MyFirstDestruction
    my_first_destruction.cpp
)
target_link_libraries(MyFirstDestruction VoxelCore)
target_compile_definitions(MyFirstDestruction PRIVATE USE_BULLET)
```

### Build and run:

```bash
cd build
cmake --build . --target MyFirstDestruction
./bin/MyFirstDestruction
```

**Expected output:**
```
Created tower with 10 voxels
Destroyed base, remaining: 8 voxels
Structure failed: YES
Failed clusters: 1
Analysis time: 0.45 ms

Simulating collapse...
  0s - Active debris: 4
  1s - Active debris: 4
  2s - Active debris: 4

Collapse simulation complete!
```

---

## Understanding the Pipeline

The voxel destruction engine has two main tracks:

### Track 1: Structural Integrity Analysis

**Input:** Voxel world + damaged positions
**Output:** Which voxel clusters should fall
**Time Budget:** < 500ms

Two analyzer options (same interface):

```cpp
// Option 1: Spring System (more parameters, easier tuning)
StructuralAnalyzer analyzer;

// Option 2: Max-Flow (23x faster, deterministic)
// MaxFlowStructuralAnalyzer analyzer;

auto result = analyzer.Analyze(world, damaged_positions, 500.0f);

if (result.structure_failed) {
    // Structure collapsed!
    for (auto& cluster : result.failed_clusters) {
        // cluster.voxel_positions contains falling voxels
        // cluster.center_of_mass is the cluster's COM
    }
}
```

### Track 3: Physics Simulation

**Input:** Failed voxel clusters from Track 1
**Output:** Realistic debris motion and settling
**Performance:** > 30 FPS with 50+ debris

```cpp
BulletEngine physics;
physics.Initialize();

VoxelPhysicsIntegration integration(&physics, &world);
integration.SpawnDebris(result.failed_clusters);

// Simulate until settled
while (integration.GetActiveDebrisCount() > 0) {
    integration.Step(1.0f / 60.0f);
}
```

### Complete Pipeline

```
Player Damage
     ↓
VoxelWorld.RemoveVoxel()
     ↓
StructuralAnalyzer.Analyze()    ← Track 1 (< 500ms)
     ↓
VoxelPhysicsIntegration.SpawnDebris()
     ↓
Physics Simulation Loop         ← Track 3 (30+ FPS)
     ↓
Settled Debris / Rubble
```

---

## Common Use Cases

### 1. Turn-Based Game Integration

```cpp
TurnManager turn_manager;

// Player's turn
void OnPlayerDestroysVoxels(std::vector<Vector3> damaged) {
    for (auto& pos : damaged) {
        world.RemoveVoxel(pos);
    }

    // Trigger turn transition
    turn_manager.StartTurnTransition();

    // Run structural analysis
    auto result = analyzer.Analyze(world, damaged);
    turn_manager.NotifyStructuralAnalysisComplete(result.structure_failed);

    // Run physics
    if (result.structure_failed) {
        integration.SpawnDebris(result.failed_clusters);

        while (integration.GetActiveDebrisCount() > 0) {
            integration.Step(1.0f / 60.0f);
        }
    }

    turn_manager.NotifyPhysicsSimulationComplete();

    // AI turn can now start
}
```

### 2. Real-Time Action Game

```cpp
void Update(float deltaTime) {
    // Check for damage this frame
    if (ExplosionOccurred()) {
        auto damaged = GetDamagedVoxels();

        // Quick structural check
        auto result = analyzer.Analyze(world, damaged, 16.0f); // 16ms budget

        if (result.structure_failed) {
            integration.SpawnDebris(result.failed_clusters);
        }
    }

    // Always update physics
    integration.Step(deltaTime);
}
```

### 3. Pre-Computed Destruction

```cpp
// During level load, pre-compute weak points
void PrecomputeWeakPoints(Building& building) {
    for (auto& column : building.support_columns) {
        // Test if removing this column causes collapse
        auto saved_voxels = SaveColumn(column);
        RemoveColumn(column);

        auto result = analyzer.Analyze(world, column.positions);

        if (result.structure_failed) {
            building.weak_points.push_back({
                column.id,
                result.failed_clusters
            });
        }

        RestoreColumn(saved_voxels);
    }
}

// During gameplay, instant lookup
void OnColumnDestroyed(int column_id) {
    auto weak_point = building.weak_points[column_id];
    integration.SpawnDebris(weak_point.clusters);
}
```

---

## Running the Demos

The project includes several demo executables:

### 1. Scenario Tests (Validation)

```bash
./bin/ScenarioTests
```

Runs 5 comprehensive test scenarios:
- Tower collapse
- Building column destruction
- Bridge support removal
- Wall stability test
- Large-scale stress test

**Use this to:** Verify the system works correctly on your platform.

### 2. Performance Profiling

```bash
./bin/PerformanceProfiling
```

Benchmarks performance with 1K, 10K, and 50K voxel structures.

**Use this to:** Verify performance meets targets (< 500ms analysis, > 30 FPS physics).

### 3. Showcase Demo

```bash
./bin/ShowcaseDemo
```

Automated demonstration of all features with beautiful console output.

**Use this to:** See the complete system in action.

### 4. Analyzer Comparison (Week 8+)

```bash
./bin/AnalyzerComparison
```

Compares Spring System vs Max-Flow analyzer across 6 scenarios:
- Performance (speed comparison)
- Accuracy (detection agreement)
- Determinism (reproducibility)

**Use this to:** Choose which analyzer to use in your project (max-flow is 23x faster).

---

## Next Steps

### Explore the Examples

- **IntegrationDemo** - 4 destruction scenarios with physics
- **FullDestructionDemo** - Complete pipeline with particles
- **BenchmarkStructuralAnalysis** - Performance testing

### Read the Documentation

- **[Structural Analyzer API](STRUCTURAL_ANALYZER_API.md)** - Track 1 reference
- **[Physics API](PHYSICS_API.md)** - Track 3 reference
- **[Parameter Tuning](PARAMETER_TUNING_GUIDE.md)** - Optimization guide

### Customize Materials

```cpp
// Create custom material
Material custom;
custom.name = "Reinforced Concrete";
custom.density = 2500.0f;           // kg/m³
custom.max_compression = 50000000.0f; // Pa
custom.yield_strength = 40000000.0f;  // Pa

// Use in voxel world
world.SetVoxel(pos, Voxel(CUSTOM_MATERIAL_ID));
```

### Add Visual Feedback

```cpp
#include "VisualFeedback.h"

VoxelVisualizer visualizer;
DestructionUI ui;
CameraEffects camera_fx;
TimeScale time_scale;

// Highlight damaged voxels
visualizer.HighlightDamagedVoxels(world, damaged_positions);

// Show analysis results
ui.ShowStructuralAnalysis(result);

// Camera shake on impact
camera_fx.TriggerShake(0.8f, 2.0f);

// Slow motion
time_scale.SetSlowMotion(0.3f, 3.0f);
```

---

## Troubleshooting

### Tests Failing

**Issue:** Some tests fail with "Structure did not fail as expected"

**Solution:** This is usually due to material properties. Check that `MaterialDatabase` is initialized:

```cpp
MaterialDatabase::Initialize(); // Call before any tests
```

### Performance Issues

**Issue:** Structural analysis takes > 500ms

**Solutions:**
1. Reduce analysis radius (default 5m)
2. Reduce max iterations (default 100)
3. Increase timestep (default 0.01)

```cpp
analyzer.SetMaxRadius(3.0f);      // Smaller analysis area
analyzer.SetMaxIterations(50);    // Fewer spring iterations
analyzer.SetTimestep(0.02f);      // Larger timestep
```

### Physics Instability

**Issue:** Debris flies wildly or explodes

**Solutions:**
1. Check material density (should be 100-10000 kg/m³)
2. Verify voxel size (default 0.05m = 5cm)
3. Reduce initial velocities

---

## Quick Reference

### Key Classes

```cpp
VoxelWorld                 // Voxel storage and queries
StructuralAnalyzer         // Track 1: Spring-based analyzer
MaxFlowStructuralAnalyzer  // Track 1: Max-flow analyzer (faster)
BulletEngine               // Bullet Physics wrapper
VoxelPhysicsIntegration    // Track 3: Spawn and manage debris
TurnManager                // Turn-based phase control
```

### Key Methods

```cpp
world.SetVoxel(pos, voxel)           // Add voxel
world.RemoveVoxel(pos)               // Remove voxel
analyzer.Analyze(world, damaged)     // Structural analysis
integration.SpawnDebris(clusters)    // Create physics debris
integration.Step(deltaTime)          // Update physics
integration.GetActiveDebrisCount()   // Check if settled
```

### Performance Targets

- **Track 1:** < 500ms for structural analysis
- **Track 3:** > 30 FPS with 50 debris objects
- **Total:** < 5 seconds for complete destruction sequence

---

## Support

- **Documentation:** See `docs/` directory
- **Examples:** See `examples/` directory
- **Tests:** See `tests/` directory
- **Issues:** Report at GitHub repository

---

**Congratulations!** You're now ready to integrate voxel-based destruction into your game. 🎉
