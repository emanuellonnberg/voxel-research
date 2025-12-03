# Parameter Tuning Guide

**Day 19: Complete guide to optimizing StructuralAnalyzer parameters**

## Table of Contents

1. [Introduction](#introduction)
2. [Parameter Overview](#parameter-overview)
3. [Manual Tuning](#manual-tuning)
4. [Automated Tuning](#automated-tuning)
5. [Sensitivity Analysis](#sensitivity-analysis)
6. [Best Practices](#best-practices)

## Introduction

The `StructuralAnalyzer` has multiple parameters that significantly affect both **performance** (speed) and **accuracy** (correctness). This guide will help you find optimal parameters for your specific use case.

### Performance vs Accuracy Trade-off

- **Fast but less accurate**: Larger timestep, fewer iterations, smaller influence radius
- **Slow but more accurate**: Smaller timestep, more iterations, larger influence radius

### When to Tune Parameters

- **Initial setup**: Use auto-tuning to find baseline parameters
- **New scenario types**: Different structure sizes/materials may need different params
- **Performance issues**: If analysis is too slow for your target framerate
- **Accuracy issues**: If structures fail/succeed incorrectly

## Parameter Overview

### Critical Parameters (Biggest Impact)

#### 1. timestep (default: 0.01)
**What it does:** Time step for physics simulation

**Impact:**
- Smaller = More accurate but slower convergence
- Larger = Faster convergence but less stable

**Typical range:** 0.005 - 0.02

**Auto-tuned optimal:** 0.005

**When to adjust:**
- Too slow → Increase (try 0.01 or 0.02)
- Unstable/oscillating → Decrease (try 0.005)

```cpp
analyzer.params.timestep = 0.005f;  // More accurate
analyzer.params.timestep = 0.02f;   // Faster
```

#### 2. influence_radius (default: 5.0)
**What it does:** Only analyze voxels within this distance of damage

**Impact:**
- Smaller = Fewer nodes analyzed (faster) but might miss failures
- Larger = More nodes analyzed (slower) but more thorough

**Typical range:** 3.0 - 10.0

**Auto-tuned optimal:** 5.0

**When to adjust:**
- Large structures → Increase (try 8-10)
- Small localized damage → Decrease (try 3-5)
- Performance critical → Decrease

```cpp
// Small structure or localized damage
analyzer.params.influence_radius = 3.0f;

// Large interconnected structure
analyzer.params.influence_radius = 10.0f;
```

#### 3. damping (default: 0.9)
**What it does:** Velocity damping factor (energy loss)

**Impact:**
- Lower = More oscillation, slower convergence
- Higher = Less oscillation, faster convergence, more stable

**Typical range:** 0.85 - 0.95

**Auto-tuned optimal:** 0.95

**When to adjust:**
- Oscillating/not converging → Increase (try 0.95)
- Too damped/unrealistic → Decrease (try 0.85)

```cpp
analyzer.params.damping = 0.95f;  // More stable
analyzer.params.damping = 0.85f;  // More dynamic
```

### Secondary Parameters

#### 4. max_iterations (default: 100)
**What it does:** Maximum solver iterations before giving up

**Impact:**
- Lower = Faster but might not converge
- Higher = Slower but more likely to converge

**Typical range:** 50 - 200

**Auto-tuned optimal:** 100

**When to adjust:**
- Not converging → Increase (try 150-200)
- Performance critical → Decrease (try 50)

```cpp
analyzer.params.max_iterations = 150;  // More thorough
```

#### 5. convergence_threshold (default: 0.0001)
**What it does:** When to stop iterating (displacement change threshold)

**Impact:**
- Smaller = More precise but slower
- Larger = Less precise but faster

**Typical range:** 0.00001 - 0.001

**When to adjust:**
- Need more precision → Decrease (try 0.00001)
- Performance critical → Increase (try 0.001)

```cpp
analyzer.params.convergence_threshold = 0.00001f;  // More precise
```

### Optimization Flags (Day 16)

#### 6. use_parallel_mass_calc (default: true)
**What it does:** Use multi-threading for mass calculation

**Impact:** 2-3x speedup on multi-core systems

**When to disable:** Never (unless debugging threading issues)

```cpp
analyzer.params.use_parallel_mass_calc = true;  // Keep enabled!
```

#### 7. use_early_termination (default: false)
**What it does:** Abort solver on obvious failures

**Impact:** Can speed up failure detection but may be too aggressive

**When to enable:** When you're confident parameters are stable

```cpp
analyzer.params.use_early_termination = true;  // Use with caution
```

#### 8. use_surface_only (default: true)
**What it does:** Only analyze surface voxels

**Impact:** Huge speedup (10-100x)

**When to disable:** Never (unless you need internal voxel analysis)

```cpp
analyzer.params.use_surface_only = true;  // Keep enabled!
```

## Manual Tuning

### Step 1: Start with Defaults

```cpp
StructuralAnalyzer analyzer;
// Uses all default parameters
```

### Step 2: Test on Representative Scenario

```cpp
// Create typical test case
VoxelWorld test_world;
BuildTypicalStructure(test_world);
std::vector<Vector3> test_damage = CreateTypicalDamage(test_world);

// Analyze with defaults
auto result = analyzer.Analyze(test_world, test_damage);

std::cout << "Baseline performance:\n";
std::cout << "  Time: " << result.calculation_time_ms << "ms\n";
std::cout << "  Iterations: " << result.iterations_used << "\n";
std::cout << "  Converged: " << result.converged << "\n";
std::cout << "  Nodes: " << result.nodes_analyzed << "\n";
```

### Step 3: Identify Issues

```cpp
if (!result.converged) {
    std::cout << "ISSUE: Not converging\n";
    std::cout << "FIX: Increase max_iterations or damping\n";
}

if (result.calculation_time_ms > target_time_ms) {
    std::cout << "ISSUE: Too slow\n";
    std::cout << "FIX: Reduce influence_radius or increase timestep\n";
}

if (result.nodes_analyzed > 1000) {
    std::cout << "ISSUE: Too many nodes\n";
    std::cout << "FIX: Reduce influence_radius\n";
}
```

### Step 4: Adjust One Parameter at a Time

```cpp
// Try increasing damping for better convergence
analyzer.params.damping = 0.95f;
auto result2 = analyzer.Analyze(test_world, test_damage);

if (result2.converged && !result.converged) {
    std::cout << "SUCCESS: Increasing damping fixed convergence\n";
}
```

### Step 5: Save Optimal Parameters

```cpp
analyzer.SaveParameters("my_optimal_params.ini");
```

## Automated Tuning

### Using Auto-Tuning (Day 17)

The `ParameterTuning` class provides automated parameter optimization.

#### Basic Auto-Tuning

```cpp
#include "ParameterTuning.h"

// Create representative test case
VoxelWorld test_world;
BuildTypicalStructure(test_world);
std::vector<Vector3> test_damage = CreateTypicalDamage(test_world);

// Run auto-tuning (tests 50 parameter combinations)
auto result = ParameterTuning::AutoTuneParameters(test_world, test_damage, 50);

std::cout << "Auto-tuning complete!\n";
std::cout << "  Tests run: " << result.total_tests_run << "\n";
std::cout << "  Best performance: " << result.avg_calculation_time_ms << "ms\n";
std::cout << "  Best configuration:\n";
std::cout << "    timestep: " << result.optimal_params.timestep << "\n";
std::cout << "    damping: " << result.optimal_params.damping << "\n";
std::cout << "    max_iterations: " << result.optimal_params.max_iterations << "\n";
std::cout << "    influence_radius: " << result.optimal_params.influence_radius << "\n";

// Use optimal parameters
StructuralAnalyzer analyzer;
analyzer.params = result.optimal_params;
analyzer.SaveParameters("auto_tuned_params.ini");
```

#### Advanced: Custom Grid Search

```cpp
void CustomGridSearch(VoxelWorld& world, const std::vector<Vector3>& damage) {
    std::vector<float> timesteps = {0.005f, 0.01f, 0.015f, 0.02f};
    std::vector<float> dampings = {0.85f, 0.9f, 0.95f};
    std::vector<float> radii = {3.0f, 5.0f, 8.0f, 10.0f};

    float best_time = std::numeric_limits<float>::max();
    StructuralAnalyzer::Parameters best_params;

    for (float timestep : timesteps) {
        for (float damping : dampings) {
            for (float radius : radii) {
                StructuralAnalyzer analyzer;
                analyzer.params.timestep = timestep;
                analyzer.params.damping = damping;
                analyzer.params.influence_radius = radius;

                auto result = analyzer.Analyze(world, damage);

                // Score based on time + convergence
                float score = result.calculation_time_ms;
                if (!result.converged) {
                    score += 1000.0f;  // Penalize non-convergence
                }

                if (score < best_time) {
                    best_time = score;
                    best_params = analyzer.params;

                    std::cout << "New best: timestep=" << timestep
                              << ", damping=" << damping
                              << ", radius=" << radius
                              << " -> " << result.calculation_time_ms << "ms\n";
                }
            }
        }
    }

    std::cout << "\nOptimal parameters found!\n";
}
```

## Sensitivity Analysis

### Single Parameter Sensitivity

Understand how each parameter affects performance.

```cpp
#include "ParameterTuning.h"

// Test timestep sensitivity
std::vector<float> timestep_values = {0.003f, 0.005f, 0.01f, 0.015f, 0.02f, 0.03f};
auto result = ParameterTuning::AnalyzeParameterSensitivity(
    analyzer, world, damage, "timestep", timestep_values
);

std::cout << "Timestep sensitivity:\n";
std::cout << "  Time range: " << result.min_time_ms << " - " << result.max_time_ms << "ms\n";
std::cout << "  Average: " << result.avg_time_ms << "ms\n";
std::cout << "  Optimal value: " << result.optimal_value << "\n";
std::cout << "  Performance variation: " << (result.max_time_ms / result.min_time_ms) << "x\n";

// Print detailed results
ParameterTuning::PrintSensitivityResults(result);

// Save to CSV for plotting
ParameterTuning::SaveSensitivityResultsCSV(result, "timestep_sensitivity.csv");
```

### Multi-Parameter Sensitivity

```cpp
void AnalyzeAllParameters(StructuralAnalyzer& analyzer,
                          VoxelWorld& world,
                          const std::vector<Vector3>& damage) {
    // Timestep
    auto ts_result = ParameterTuning::AnalyzeParameterSensitivity(
        analyzer, world, damage, "timestep",
        {0.005f, 0.01f, 0.015f, 0.02f}
    );
    ParameterTuning::SaveSensitivityResultsCSV(ts_result, "timestep.csv");

    // Damping
    auto damp_result = ParameterTuning::AnalyzeParameterSensitivity(
        analyzer, world, damage, "damping",
        {0.8f, 0.85f, 0.9f, 0.95f, 0.99f}
    );
    ParameterTuning::SaveSensitivityResultsCSV(damp_result, "damping.csv");

    // Influence radius
    auto radius_result = ParameterTuning::AnalyzeParameterSensitivity(
        analyzer, world, damage, "influence_radius",
        {1.0f, 3.0f, 5.0f, 8.0f, 10.0f, 15.0f}
    );
    ParameterTuning::SaveSensitivityResultsCSV(radius_result, "radius.csv");

    // Max iterations
    auto iter_result = ParameterTuning::AnalyzeParameterSensitivity(
        analyzer, world, damage, "max_iterations",
        {25.0f, 50.0f, 100.0f, 150.0f, 200.0f}
    );
    ParameterTuning::SaveSensitivityResultsCSV(iter_result, "iterations.csv");

    std::cout << "Sensitivity analysis complete. CSV files generated.\n";
    std::cout << "Use spreadsheet software or Python to plot the results.\n";
}
```

### Interpreting Sensitivity Results

**High sensitivity (large variation):**
- Parameter has big impact on performance
- Worth tuning carefully
- Example: `influence_radius` often has 2-5x variation

**Low sensitivity (small variation):**
- Parameter has small impact
- Less important to tune
- Use defaults or conservative values

**Example interpretation:**
```
Timestep sensitivity:
  Time range: 8.2ms - 12.4ms
  Variation: 1.5x
  → Moderate sensitivity, worth tuning

Influence radius sensitivity:
  Time range: 5.1ms - 23.7ms
  Variation: 4.6x
  → High sensitivity, tune carefully!

Damping sensitivity:
  Time range: 9.8ms - 10.3ms
  Variation: 1.05x
  → Low sensitivity, defaults are fine
```

## Best Practices

### 1. Use Provided Configurations

```cpp
// Start with optimal parameters from auto-tuning
analyzer.LoadParameters("config/optimal_params.ini");
```

### 2. Tune for Your Scenario

```cpp
// Different scenarios need different parameters
void LoadParametersForScenario(StructuralAnalyzer& analyzer, const char* scenario) {
    if (strcmp(scenario, "small_building") == 0) {
        analyzer.params.influence_radius = 3.0f;
        analyzer.params.max_iterations = 50;
    } else if (strcmp(scenario, "large_structure") == 0) {
        analyzer.params.influence_radius = 10.0f;
        analyzer.params.max_iterations = 150;
        analyzer.params.use_early_termination = true;
    } else if (strcmp(scenario, "bridge") == 0) {
        analyzer.params.influence_radius = 15.0f;
        analyzer.params.timestep = 0.005f;
    }
}
```

### 3. Validate After Tuning

```cpp
// Always validate parameters
if (!analyzer.ValidateParameters()) {
    std::cout << "Warning: Parameters were auto-corrected\n";
}
```

### 4. Monitor Performance in Production

```cpp
class ParameterMonitor {
private:
    std::vector<float> analysis_times;
    int non_convergence_count = 0;

public:
    void RecordAnalysis(const AnalysisResult& result) {
        analysis_times.push_back(result.calculation_time_ms);
        if (!result.converged) {
            non_convergence_count++;
        }

        // Check if we need to retune
        if (analysis_times.size() >= 100) {
            float avg_time = 0.0f;
            for (float t : analysis_times) avg_time += t;
            avg_time /= analysis_times.size();

            float non_conv_rate = (float)non_convergence_count / analysis_times.size();

            if (avg_time > target_time_ms || non_conv_rate > 0.1f) {
                std::cout << "Performance degraded, consider retuning:\n";
                std::cout << "  Avg time: " << avg_time << "ms (target: "
                          << target_time_ms << "ms)\n";
                std::cout << "  Non-convergence rate: " << (non_conv_rate * 100)
                          << "%\n";
            }

            // Reset
            analysis_times.clear();
            non_convergence_count = 0;
        }
    }

private:
    float target_time_ms = 50.0f;
};
```

### 5. Document Your Parameters

```cpp
// Save parameters with descriptive filename
analyzer.SaveParameters("params_building_explosion_20voxels.ini");

// Add comments to INI file manually
/*
[StructuralAnalyzer]
# Optimized for: Medium-sized building explosions (10-30 voxel damage)
# Target performance: <20ms on 4-core CPU
# Tuned on: 2024-12-03
# Test scenarios: 50 building collapse simulations
timestep=0.005
damping=0.95
...
*/
```

## Parameter Presets

### Preset 1: Speed (Fast, Less Accurate)

```cpp
analyzer.params.timestep = 0.02f;
analyzer.params.damping = 0.85f;
analyzer.params.max_iterations = 50;
analyzer.params.influence_radius = 3.0f;
analyzer.params.convergence_threshold = 0.001f;
analyzer.params.use_early_termination = true;
```

**Use case:** Real-time games, 60+ FPS requirement

### Preset 2: Balanced (Good Speed and Accuracy)

```cpp
analyzer.params.timestep = 0.01f;
analyzer.params.damping = 0.9f;
analyzer.params.max_iterations = 100;
analyzer.params.influence_radius = 5.0f;
analyzer.params.convergence_threshold = 0.0001f;
analyzer.params.use_early_termination = false;
```

**Use case:** Most games, 30-60 FPS

### Preset 3: Accuracy (Slow, Very Accurate)

```cpp
analyzer.params.timestep = 0.005f;
analyzer.params.damping = 0.95f;
analyzer.params.max_iterations = 200;
analyzer.params.influence_radius = 10.0f;
analyzer.params.convergence_threshold = 0.00001f;
analyzer.params.use_early_termination = false;
```

**Use case:** Offline processing, cinematic sequences, validation

### Preset 4: Large Structures

```cpp
analyzer.params.timestep = 0.01f;
analyzer.params.damping = 0.9f;
analyzer.params.max_iterations = 150;
analyzer.params.influence_radius = 8.0f;
analyzer.params.use_parallel_mass_calc = true;
analyzer.params.use_early_termination = true;
```

**Use case:** Skyscrapers, bridges, large interconnected structures

## Troubleshooting Parameter Issues

### Issue: Analysis Too Slow

**Diagnosis:**
```cpp
if (result.calculation_time_ms > 100.0f) {
    std::cout << "Bottleneck analysis:\n";
    std::cout << "  Nodes: " << result.nodes_analyzed << "\n";
    std::cout << "  Mass calc: " << result.mass_calc_time_ms << "ms\n";
    std::cout << "  Solver: " << result.solver_time_ms << "ms\n";
}
```

**Solutions:**
1. Reduce `influence_radius` (biggest impact)
2. Reduce `max_iterations`
3. Increase `timestep`
4. Enable `use_early_termination`

### Issue: Not Converging

**Diagnosis:**
```cpp
if (!result.converged) {
    std::cout << "Convergence failed:\n";
    std::cout << "  Iterations used: " << result.iterations_used
              << " / " << analyzer.params.max_iterations << "\n";
}
```

**Solutions:**
1. Increase `max_iterations`
2. Increase `damping`
3. Decrease `timestep`

### Issue: Oscillating Results

**Diagnosis:**
Run same analysis multiple times:
```cpp
for (int i = 0; i < 5; i++) {
    auto result = analyzer.Analyze(world, damage);
    std::cout << "Run " << i << ": " << result.structure_failed << "\n";
}
// If results differ, parameters are unstable
```

**Solutions:**
1. Increase `damping` (to 0.95 or higher)
2. Decrease `timestep`
3. Increase `convergence_threshold`

## See Also

- [API Documentation](STRUCTURAL_ANALYZER_API.md) - Complete API reference
- [Usage Guide](USAGE_GUIDE.md) - Practical usage examples
- [PhysX Integration](PHYSX_INTEGRATION_GUIDE.md) - Physics engine integration
