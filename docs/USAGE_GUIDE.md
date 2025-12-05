# StructuralAnalyzer Usage Guide

**Day 19: Practical examples and usage patterns for structural analysis**

## Table of Contents

1. [Quick Start](#quick-start)
2. [Common Scenarios](#common-scenarios)
3. [Integration Patterns](#integration-patterns)
4. [Performance Optimization](#performance-optimization)
5. [Troubleshooting](#troubleshooting)

## Quick Start

### Minimal Example

```cpp
#include "StructuralAnalyzer.h"
#include "VoxelWorld.h"

int main() {
    // 1. Create world and analyzer
    VoxelWorld world;
    StructuralAnalyzer analyzer;

    // 2. Build a simple tower
    float voxel_size = world.GetVoxelSize();  // 0.05m
    for (int y = 0; y < 10; y++) {
        Vector3 pos(0, y * voxel_size, 0);
        world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
    }

    // 3. Apply damage (remove middle voxel)
    Vector3 damage_pos(0, 5 * voxel_size, 0);
    world.RemoveVoxel(damage_pos);

    // 4. Analyze
    std::vector<Vector3> damaged = {damage_pos};
    auto result = analyzer.Analyze(world, damaged);

    // 5. Handle results
    if (result.structure_failed) {
        std::cout << "Tower collapsed! " << result.num_failed_nodes
                  << " voxels falling\n";

        for (const auto& cluster : result.failed_clusters) {
            std::cout << "  Cluster: " << cluster.voxel_positions.size()
                      << " voxels, " << cluster.total_mass << " kg\n";
        }
    } else {
        std::cout << "Tower is stable!\n";
    }

    return 0;
}
```

### With Custom Parameters

```cpp
// Load optimal parameters
analyzer.LoadParameters("config/optimal_params.ini");

// Or set manually
analyzer.params.timestep = 0.005f;
analyzer.params.damping = 0.95f;
analyzer.params.max_iterations = 150;
analyzer.params.influence_radius = 10.0f;

// Analyze with custom params
auto result = analyzer.Analyze(world, damaged);
```

## Common Scenarios

### Scenario 1: Building Destruction

Analyze a building after explosive damage.

```cpp
void AnalyzeExplosionDamage(VoxelWorld& world, Vector3 explosion_center, float radius) {
    StructuralAnalyzer analyzer;

    // Find all voxels in explosion radius
    std::vector<Vector3> damaged_voxels;
    auto nearby = world.GetVoxelsInRadius(explosion_center, radius);

    for (const auto& pos : nearby) {
        // Calculate damage based on distance
        float dist = pos.Distance(explosion_center);
        float damage_amount = 1.0f - (dist / radius);

        if (damage_amount > 0.5f) {
            world.RemoveVoxel(pos);
            damaged_voxels.push_back(pos);
        }
    }

    // Analyze structural integrity
    auto result = analyzer.Analyze(world, damaged_voxels);

    if (result.structure_failed) {
        std::cout << "Building collapsed!\n";
        std::cout << "  " << result.failed_clusters.size() << " debris clusters\n";
        std::cout << "  " << result.num_disconnected_nodes << " disconnected voxels\n";

        // Spawn physics bodies for each cluster
        for (const auto& cluster : result.failed_clusters) {
            SpawnDebrisCluster(cluster);
        }
    }
}
```

### Scenario 2: Progressive Damage

Apply damage over time and check stability after each hit.

```cpp
class ProgressiveDamageSystem {
private:
    VoxelWorld& world;
    StructuralAnalyzer analyzer;
    std::vector<Vector3> all_damaged;

public:
    ProgressiveDamageSystem(VoxelWorld& w) : world(w) {
        // Use optimal parameters
        analyzer.LoadParameters("config/optimal_params.ini");
    }

    bool ApplyDamage(Vector3 hit_position) {
        // Remove voxel
        if (!world.HasVoxel(hit_position)) {
            return false;  // Already destroyed
        }

        world.RemoveVoxel(hit_position);
        all_damaged.push_back(hit_position);

        // Analyze cumulative damage
        // Use only recent damage for influence radius optimization
        std::vector<Vector3> recent_damage;
        int recent_count = std::min(10, (int)all_damaged.size());
        recent_damage.assign(
            all_damaged.end() - recent_count,
            all_damaged.end()
        );

        auto result = analyzer.Analyze(world, recent_damage);

        if (result.structure_failed) {
            std::cout << "Damage caused collapse after "
                      << all_damaged.size() << " hits\n";

            // Remove failed voxels from world
            for (const auto& cluster : result.failed_clusters) {
                for (const auto& pos : cluster.voxel_positions) {
                    world.RemoveVoxel(pos);
                }
            }

            // Clear cache since world changed
            analyzer.ClearMassCache();

            return true;  // Collapse occurred
        }

        return false;  // Still stable
    }
};
```

### Scenario 3: Bridge Analysis

Analyze a bridge structure with supports at both ends.

```cpp
void AnalyzeBridge(VoxelWorld& world, Vector3 bridge_start, Vector3 bridge_end) {
    StructuralAnalyzer analyzer;

    // Use larger influence radius for long structures
    analyzer.params.influence_radius = 15.0f;

    // Simulate support failure at midpoint
    Vector3 midpoint = (bridge_start + bridge_end) * 0.5f;
    float voxel_size = world.GetVoxelSize();

    std::vector<Vector3> damaged;
    for (float y = 0; y < 2.0f; y += voxel_size) {
        Vector3 damage_pos = midpoint + Vector3(0, y, 0);
        if (world.HasVoxel(damage_pos)) {
            world.RemoveVoxel(damage_pos);
            damaged.push_back(damage_pos);
        }
    }

    auto result = analyzer.Analyze(world, damaged);

    if (result.structure_failed) {
        std::cout << "Bridge collapsed!\n";
        std::cout << "  Convergence: " << result.converged << "\n";
        std::cout << "  Iterations: " << result.iterations_used << "\n";
        std::cout << "  Analysis time: " << result.calculation_time_ms << "ms\n";

        // Analyze failure pattern
        int left_failures = 0, right_failures = 0;
        for (const auto& cluster : result.failed_clusters) {
            Vector3 avg_pos = cluster.center_of_mass;
            if (avg_pos.x < midpoint.x) {
                left_failures += cluster.voxel_positions.size();
            } else {
                right_failures += cluster.voxel_positions.size();
            }
        }

        std::cout << "  Left span: " << left_failures << " voxels\n";
        std::cout << "  Right span: " << right_failures << " voxels\n";
    } else {
        std::cout << "Bridge is stable after damage\n";
    }
}
```

### Scenario 4: Multi-Story Building

Analyze a complex multi-story structure.

```cpp
struct BuildingFloor {
    float y_level;
    std::vector<Vector3> voxels;
};

void AnalyzeBuilding(VoxelWorld& world, const std::vector<BuildingFloor>& floors) {
    StructuralAnalyzer analyzer;

    // Enable parallel computation for large structures
    analyzer.params.use_parallel_mass_calc = true;
    analyzer.params.influence_radius = 8.0f;

    // Damage ground floor column
    std::vector<Vector3> damaged;
    Vector3 column_base(5.0f, 0.0f, 5.0f);
    float voxel_size = world.GetVoxelSize();

    for (float y = 0; y < floors[0].y_level + 1.0f; y += voxel_size) {
        Vector3 pos = column_base + Vector3(0, y, 0);
        if (world.HasVoxel(pos)) {
            world.RemoveVoxel(pos);
            damaged.push_back(pos);
        }
    }

    std::cout << "Analyzing " << floors.size() << " story building...\n";

    auto start = std::chrono::high_resolution_clock::now();
    auto result = analyzer.Analyze(world, damaged);
    auto elapsed = std::chrono::duration<float, std::milli>(
        std::chrono::high_resolution_clock::now() - start).count();

    std::cout << "Analysis complete in " << elapsed << "ms\n";
    std::cout << "  Nodes analyzed: " << result.nodes_analyzed << "\n";
    std::cout << "  Solver iterations: " << result.iterations_used << "\n";
    std::cout << "  Converged: " << result.converged << "\n";

    if (result.structure_failed) {
        std::cout << "Building FAILED!\n";

        // Count failures per floor
        for (size_t i = 0; i < floors.size(); i++) {
            int floor_failures = 0;
            for (const auto& cluster : result.failed_clusters) {
                for (const auto& pos : cluster.voxel_positions) {
                    if (std::abs(pos.y - floors[i].y_level) < 1.0f) {
                        floor_failures++;
                    }
                }
            }
            if (floor_failures > 0) {
                std::cout << "  Floor " << i << ": " << floor_failures
                          << " voxels failed\n";
            }
        }
    }
}
```

## Integration Patterns

### Pattern 1: Game Loop Integration

```cpp
class DestructionSystem {
private:
    VoxelWorld& world;
    StructuralAnalyzer analyzer;
    std::queue<Vector3> pending_damage;
    float analysis_cooldown = 0.0f;

public:
    DestructionSystem(VoxelWorld& w) : world(w) {
        analyzer.LoadParameters("config/optimal_params.ini");
    }

    void ApplyDamage(Vector3 position) {
        if (world.HasVoxel(position)) {
            world.RemoveVoxel(position);
            pending_damage.push(position);
        }
    }

    void Update(float delta_time) {
        // Rate-limit analysis (e.g., once per second)
        analysis_cooldown -= delta_time;

        if (analysis_cooldown <= 0.0f && !pending_damage.empty()) {
            analysis_cooldown = 1.0f;  // Cooldown 1 second

            // Collect all pending damage
            std::vector<Vector3> damaged;
            while (!pending_damage.empty()) {
                damaged.push_back(pending_damage.front());
                pending_damage.pop();
            }

            // Analyze
            auto result = analyzer.Analyze(world, damaged);

            if (result.structure_failed) {
                // Spawn physics debris
                for (const auto& cluster : result.failed_clusters) {
                    SpawnPhysicsCluster(cluster);

                    // Remove from voxel world
                    for (const auto& pos : cluster.voxel_positions) {
                        world.RemoveVoxel(pos);
                    }
                }

                // Clear cache since world changed
                analyzer.ClearMassCache();
            }
        }
    }

private:
    void SpawnPhysicsCluster(const VoxelCluster& cluster) {
        // Create physics body from cluster
        // (Implementation depends on physics engine)
    }
};
```

### Pattern 2: Async Analysis

```cpp
#include <future>
#include <mutex>

class AsyncDestructionAnalyzer {
private:
    VoxelWorld& world;
    std::future<AnalysisResult> pending_analysis;
    std::mutex world_mutex;

public:
    AsyncDestructionAnalyzer(VoxelWorld& w) : world(w) {}

    void StartAnalysis(const std::vector<Vector3>& damaged) {
        if (IsAnalyzing()) {
            return;  // Analysis already in progress
        }

        // Launch async analysis
        pending_analysis = std::async(std::launch::async, [this, damaged]() {
            // Create thread-local analyzer
            StructuralAnalyzer analyzer;
            analyzer.LoadParameters("config/optimal_params.ini");

            // Lock world for reading
            std::lock_guard<std::mutex> lock(world_mutex);
            return analyzer.Analyze(world, damaged);
        });
    }

    bool IsAnalyzing() const {
        return pending_analysis.valid() &&
               pending_analysis.wait_for(std::chrono::seconds(0)) !=
               std::future_status::ready;
    }

    std::optional<AnalysisResult> GetResult() {
        if (!pending_analysis.valid()) {
            return std::nullopt;
        }

        if (pending_analysis.wait_for(std::chrono::seconds(0)) ==
            std::future_status::ready) {
            return pending_analysis.get();
        }

        return std::nullopt;
    }
};
```

### Pattern 3: Batch Processing

```cpp
void BatchAnalyzeStructures(
    const std::vector<VoxelWorld*>& worlds,
    const std::vector<std::vector<Vector3>>& damages)
{
    std::vector<AnalysisResult> results(worlds.size());

    #pragma omp parallel for
    for (size_t i = 0; i < worlds.size(); i++) {
        // Thread-local analyzer
        StructuralAnalyzer analyzer;
        analyzer.LoadParameters("config/optimal_params.ini");

        // Analyze
        results[i] = analyzer.Analyze(*worlds[i], damages[i]);
    }

    // Process results
    for (size_t i = 0; i < results.size(); i++) {
        if (results[i].structure_failed) {
            std::cout << "World " << i << " failed: "
                      << results[i].num_failed_nodes << " nodes\n";
        }
    }
}
```

## Performance Optimization

### Optimization 1: Tune Influence Radius

```cpp
// Measure influence radius impact
void BenchmarkInfluenceRadius(VoxelWorld& world, const std::vector<Vector3>& damage) {
    std::vector<float> radii = {1.0f, 3.0f, 5.0f, 10.0f, 20.0f};

    for (float radius : radii) {
        StructuralAnalyzer analyzer;
        analyzer.params.influence_radius = radius;

        auto start = std::chrono::high_resolution_clock::now();
        auto result = analyzer.Analyze(world, damage);
        auto elapsed = std::chrono::duration<float, std::milli>(
            std::chrono::high_resolution_clock::now() - start).count();

        std::cout << "Radius " << radius << "m: "
                  << elapsed << "ms, "
                  << result.nodes_analyzed << " nodes\n";
    }
}
```

### Optimization 2: Use Parameter Tuning

```cpp
#include "ParameterTuning.h"

void OptimizeForScene(VoxelWorld& world, const std::vector<Vector3>& test_damage) {
    // Run auto-tuning
    std::cout << "Auto-tuning parameters...\n";
    auto tuning_result = ParameterTuning::AutoTuneParameters(
        world, test_damage, 50  // 50 iterations
    );

    std::cout << "Optimal parameters found:\n";
    std::cout << "  timestep: " << tuning_result.optimal_params.timestep << "\n";
    std::cout << "  damping: " << tuning_result.optimal_params.damping << "\n";
    std::cout << "  max_iterations: " << tuning_result.optimal_params.max_iterations << "\n";
    std::cout << "  influence_radius: " << tuning_result.optimal_params.influence_radius << "\n";
    std::cout << "  Performance: " << tuning_result.avg_calculation_time_ms << "ms avg\n";

    // Save for future use
    StructuralAnalyzer analyzer;
    analyzer.params = tuning_result.optimal_params;
    analyzer.SaveParameters("optimal_params_custom.ini");
}
```

### Optimization 3: Cache Management

```cpp
class SmartCacheManager {
private:
    StructuralAnalyzer& analyzer;
    int analyses_since_clear = 0;
    float cache_hit_rate_threshold = 0.5f;

public:
    SmartCacheManager(StructuralAnalyzer& a) : analyzer(a) {}

    void AfterAnalysis(const AnalysisResult& result) {
        analyses_since_clear++;

        // Check cache performance
        int hits, misses;
        analyzer.GetCacheStats(hits, misses);

        float hit_rate = (float)hits / (hits + misses + 1);

        // Clear cache if hit rate is low or after many analyses
        if (hit_rate < cache_hit_rate_threshold || analyses_since_clear > 100) {
            std::cout << "Clearing cache (hit rate: " << hit_rate << ")\n";
            analyzer.ClearMassCache();
            analyses_since_clear = 0;
        }
    }
};
```

## Troubleshooting

### Issue 1: Analysis Too Slow

**Symptoms:** Analysis takes >100ms for small structures

**Solutions:**
1. Reduce `influence_radius`
2. Enable `use_surface_only` (should be default)
3. Enable `use_parallel_mass_calc` (should be default)
4. Check if structure is actually small (print `result.nodes_analyzed`)

```cpp
if (result.calculation_time_ms > 100.0f) {
    std::cout << "Slow analysis detected!\n";
    std::cout << "  Nodes: " << result.nodes_analyzed << "\n";
    std::cout << "  Node graph: " << result.node_graph_time_ms << "ms\n";
    std::cout << "  Mass calc: " << result.mass_calc_time_ms << "ms\n";
    std::cout << "  Solver: " << result.solver_time_ms << "ms\n";

    // Bottleneck identification
    if (result.mass_calc_time_ms > 50.0f) {
        std::cout << "BOTTLENECK: Mass calculation\n";
        std::cout << "SOLUTION: Reduce influence_radius or enable parallel\n";
    }
    if (result.solver_time_ms > 50.0f) {
        std::cout << "BOTTLENECK: Solver\n";
        std::cout << "SOLUTION: Reduce max_iterations or enable early_termination\n";
    }
}
```

### Issue 2: False Positives (Stable Structures Failing)

**Symptoms:** Structures that should be stable are marked as failed

**Solutions:**
1. Increase `convergence_threshold`
2. Increase `max_iterations`
3. Reduce `timestep` (more accurate but slower)
4. Check `result.converged` - if false, solver didn't finish

```cpp
if (result.structure_failed && !result.converged) {
    std::cout << "WARNING: Structure failed but solver didn't converge!\n";
    std::cout << "  Iterations: " << result.iterations_used
              << " / " << analyzer.params.max_iterations << "\n";
    std::cout << "SOLUTION: Increase max_iterations\n";
}
```

### Issue 3: False Negatives (Unstable Structures Stable)

**Symptoms:** Structures that should collapse are marked as stable

**Solutions:**
1. Decrease `convergence_threshold` (more sensitive)
2. Increase `influence_radius` (analyze more nodes)
3. Check material properties (max_displacement might be too high)

```cpp
if (!result.structure_failed && result.nodes_analyzed < 10) {
    std::cout << "WARNING: Very few nodes analyzed!\n";
    std::cout << "  Nodes: " << result.nodes_analyzed << "\n";
    std::cout << "SOLUTION: Increase influence_radius\n";
}
```

### Issue 4: Numerical Instability

**Symptoms:** "Numerical instability detected" errors

**Solutions:**
1. Reduce `timestep` (more stable)
2. Increase `damping` (more stable)
3. Check for degenerate structures (single floating voxels)

```cpp
// The analyzer now handles this automatically (Day 18)
// But you can check the debug info
if (result.debug_info.find("instability") != std::string::npos) {
    std::cout << "Numerical instability detected!\n";
    std::cout << "  Debug: " << result.debug_info << "\n";
    std::cout << "SOLUTION: Reduce timestep or increase damping\n";
}
```

## Best Practices

1. **Always validate parameters** before analysis
   ```cpp
   if (!analyzer.ValidateParameters()) {
       std::cout << "Parameters were corrected\n";
   }
   ```

2. **Monitor performance** in production
   ```cpp
   if (result.calculation_time_ms > target_time_ms) {
       LogPerformanceIssue(result);
   }
   ```

3. **Use optimal parameters** from auto-tuning
   ```cpp
   analyzer.LoadParameters("config/optimal_params.ini");
   ```

4. **Clear cache** when world changes significantly
   ```cpp
   analyzer.ClearMassCache();
   ```

5. **Handle edge cases** gracefully
   ```cpp
   if (damaged.empty()) {
       return;  // No damage, no analysis needed
   }
   ```

## See Also

- [API Documentation](STRUCTURAL_ANALYZER_API.md) - Complete API reference
- [Parameter Tuning Guide](PARAMETER_TUNING_GUIDE.md) - Optimize parameters
- [PhysX Integration Guide](PHYSX_INTEGRATION_GUIDE.md) - Integrate with physics
