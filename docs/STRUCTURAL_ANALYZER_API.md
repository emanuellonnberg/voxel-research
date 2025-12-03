# StructuralAnalyzer API Documentation

**Day 19: Comprehensive API reference for the structural integrity analysis system**

## Overview

The `StructuralAnalyzer` uses a displacement-based spring system to analyze voxel structures for stability after damage. It determines which voxels should collapse by simulating mass distribution and structural forces.

## Core Concepts

### Spring Node System
Each voxel is represented as a **SpringNode** in the analysis:
- Connected to neighboring voxels via spring forces
- Has displacement (sagging), velocity, and supported mass
- Can be a ground anchor (infinite support) or floating

### Analysis Pipeline
1. **Build Node Graph**: Creates spring nodes for surface voxels near damage
2. **Calculate Mass**: Raycasts upward to determine supported mass
3. **Solve Displacements**: Iteratively solves spring physics
4. **Detect Failures**: Identifies failed nodes (displacement or disconnection)
5. **Find Clusters**: Groups failed nodes into collision clusters

## Classes

### StructuralAnalyzer

The main analysis class.

#### Constructor
```cpp
StructuralAnalyzer()
```
Creates a new analyzer with default parameters.

#### Core Methods

##### Analyze()
```cpp
AnalysisResult Analyze(
    VoxelWorld& voxel_world,
    const std::vector<Vector3>& damaged_positions,
    float time_budget_ms = 500.0f
)
```

Analyzes structure stability after damage.

**Parameters:**
- `voxel_world`: The voxel world to analyze
- `damaged_positions`: Positions where voxels were removed
- `time_budget_ms`: Maximum time budget (currently unused)

**Returns:** `AnalysisResult` containing failure information

**Thread Safety:** Not thread-safe. Create separate analyzers per thread.

**Example:**
```cpp
StructuralAnalyzer analyzer;
VoxelWorld world;

// Apply damage
world.RemoveVoxel(Vector3(0, 5, 0));
std::vector<Vector3> damaged = {Vector3(0, 5, 0)};

// Analyze
auto result = analyzer.Analyze(world, damaged);

if (result.structure_failed) {
    std::cout << "Structure failed! "
              << result.num_failed_nodes << " nodes failed\n";

    for (const auto& cluster : result.failed_clusters) {
        std::cout << "Cluster with " << cluster.voxel_positions.size()
                  << " voxels\n";
    }
}
```

##### Clear()
```cpp
void Clear()
```

Clears all internal state (nodes, cache, statistics).

**Usage:** Call between analyses to reset state. Automatically called by `Analyze()`.

##### Parameter Management

###### LoadParameters()
```cpp
bool LoadParameters(const std::string& filename)
```

Loads parameters from INI file (Day 17).

**Parameters:**
- `filename`: Path to INI file

**Returns:** `true` if successful

**Example:**
```cpp
analyzer.LoadParameters("config/optimal_params.ini");
```

###### SaveParameters()
```cpp
bool SaveParameters(const std::string& filename) const
```

Saves current parameters to INI file (Day 17).

**Example:**
```cpp
analyzer.params.timestep = 0.005f;
analyzer.params.damping = 0.95f;
analyzer.SaveParameters("my_params.ini");
```

###### ValidateParameters()
```cpp
bool ValidateParameters()
```

Validates and corrects parameters (Day 18).

**Returns:** `false` if corrections were made

**Automatic Corrections:**
- Timestep: Clamped to [0.0001, 1.0]
- Damping: Clamped to [0.0, 1.0]
- Max iterations: Minimum 1
- Convergence threshold: Clamped to [0.00001, 1.0]
- Influence radius: Minimum 1.0

**Note:** Automatically called by `Analyze()`.

##### Cache Management

###### ClearMassCache()
```cpp
void ClearMassCache()
```

Clears the mass calculation cache.

**Usage:** Call when voxels are added/removed to invalidate cached mass values.

###### InvalidateMassCache()
```cpp
void InvalidateMassCache(const std::vector<Vector3>& changed_positions)
```

Invalidates cache entries for specific positions and everything below them.

**Parameters:**
- `changed_positions`: Positions that changed

###### GetCacheStats()
```cpp
void GetCacheStats(int& hits, int& misses) const
```

Retrieves cache performance statistics (Day 12).

**Example:**
```cpp
int hits, misses;
analyzer.GetCacheStats(hits, misses);
float hit_rate = (float)hits / (hits + misses);
std::cout << "Cache hit rate: " << (hit_rate * 100) << "%\n";
```

##### Debugging Methods

###### GetNodeCount()
```cpp
size_t GetNodeCount() const
```

Returns number of nodes in current analysis.

###### GetNode()
```cpp
const SpringNode* GetNode(size_t index) const
```

Returns pointer to node at index, or `nullptr` if invalid.

###### CheckNumericalStability()
```cpp
bool CheckNumericalStability() const
```

Checks for numerical stability issues (Day 18).

**Returns:** `false` if NaN, Inf, or extreme values detected

**Note:** Automatically called during analysis.

#### Public Members

##### params
```cpp
Parameters params;
```

Tunable analysis parameters. Modify before calling `Analyze()`.

**Example:**
```cpp
analyzer.params.timestep = 0.005f;
analyzer.params.damping = 0.95f;
analyzer.params.max_iterations = 150;
analyzer.params.influence_radius = 10.0f;
```

## Data Structures

### Parameters

Configuration for structural analysis.

```cpp
struct Parameters {
    float timestep;                 // Simulation timestep (default 0.01s)
    float damping;                  // Velocity damping (default 0.9)
    int max_iterations;             // Max solver iterations (default 100)
    float convergence_threshold;    // Convergence check (default 0.0001m)
    float ground_level;             // Ground plane Y (default 0.0)
    bool use_surface_only;          // Analyze only surface voxels (default true)
    float influence_radius;         // Radius around damage (default 5.0m, Day 16)
    bool use_parallel_mass_calc;    // Multi-threading (default true, Day 16)
    bool use_early_termination;     // Early failure detection (default false, Day 16)
};
```

**Field Details:**

- **timestep**: Time step for physics simulation. Smaller = more accurate but slower.
  - Range: [0.0001, 1.0]
  - Typical: 0.005 - 0.02
  - Optimal: 0.005 (from auto-tuning)

- **damping**: Velocity damping factor. Higher = more stable, less oscillation.
  - Range: [0.0, 1.0]
  - Typical: 0.85 - 0.95
  - Optimal: 0.95 (from auto-tuning)

- **max_iterations**: Maximum solver iterations before giving up.
  - Range: [1, 10000]
  - Typical: 50 - 150
  - Optimal: 100 (from auto-tuning)

- **convergence_threshold**: When to stop iterating (displacement change threshold).
  - Range: [0.00001, 1.0]
  - Typical: 0.0001 - 0.001
  - Default: 0.0001

- **ground_level**: Y-coordinate of ground plane. Voxels at/below are anchored.
  - Default: 0.0
  - Set based on your world coordinate system

- **use_surface_only**: If true, only analyze surface voxels (huge speedup).
  - Default: true
  - Recommended: true (unless you need internal analysis)

- **influence_radius**: Only analyze voxels within this distance of damage.
  - Range: [1.0, 1000.0]
  - Typical: 3.0 - 10.0
  - Optimal: 5.0 (from auto-tuning)
  - Larger = more accurate but slower

- **use_parallel_mass_calc**: Use multi-threading for mass calculation.
  - Default: true
  - Speedup: 2-3x on multi-core systems (Day 16)
  - Only activates for >100 nodes

- **use_early_termination**: Abort solver on obvious failures.
  - Default: false (can be too aggressive)
  - Enable for speed when stability is guaranteed

### AnalysisResult

Output from structural analysis.

```cpp
struct AnalysisResult {
    bool structure_failed;                      // Did any nodes fail?
    std::vector<VoxelCluster> failed_clusters;  // Clusters to remove/simulate
    int num_failed_nodes;                       // Total nodes that failed
    int num_disconnected_nodes;                 // Nodes with no ground path

    // Performance metrics
    float calculation_time_ms;
    int iterations_used;
    bool converged;

    // Detailed profiling (Day 12)
    float node_graph_time_ms;
    float mass_calc_time_ms;
    float solver_time_ms;
    float failure_detection_time_ms;
    int nodes_analyzed;
    int cache_hits;
    int cache_misses;

    // Debug info
    std::string debug_info;
};
```

**Field Details:**

- **structure_failed**: `true` if any voxels should collapse
- **failed_clusters**: List of voxel clusters that failed (for physics simulation)
- **num_failed_nodes**: Count of failed nodes (displacement or disconnection)
- **num_disconnected_nodes**: Count of nodes with no path to ground
- **calculation_time_ms**: Total analysis time
- **iterations_used**: Number of solver iterations
- **converged**: Whether solver converged (vs. hitting max iterations)
- **node_graph_time_ms**: Time to build node graph
- **mass_calc_time_ms**: Time to calculate supported masses
- **solver_time_ms**: Time to solve displacements
- **failure_detection_time_ms**: Time to detect failures
- **nodes_analyzed**: Number of nodes analyzed
- **cache_hits/misses**: Cache performance statistics
- **debug_info**: Human-readable debug information

### SpringNode

Internal representation of a voxel in the spring system.

```cpp
struct SpringNode {
    Vector3 position;           // World position
    uint8_t material_id;        // Material type

    // Spring physics state
    float displacement;         // Vertical displacement (negative = sag down)
    float velocity;             // Rate of displacement change
    float mass_supported;       // Total mass this node supports (kg)

    // Connectivity
    std::vector<SpringNode*> neighbors;
    bool is_ground_anchor;      // Touching ground (infinite support)

    // Failure tracking
    bool has_failed;            // Exceeded material limits
    bool is_disconnected;       // No path to ground

    // For debugging
    int id;
};
```

**Notes:**
- `displacement` is negative when sagging downward (physically correct)
- `velocity` is damped each iteration to simulate energy loss
- `mass_supported` calculated via upward raycast (Day 11)
- `neighbors` populated based on 6-connected adjacency
- `is_ground_anchor` determined by `ground_level` parameter
- `has_failed` set when `abs(displacement) > material.max_displacement`
- `is_disconnected` set via flood-fill from ground anchors (Day 14)

## Usage Patterns

### Basic Analysis

```cpp
#include "StructuralAnalyzer.h"
#include "VoxelWorld.h"

// Create world and analyzer
VoxelWorld world;
StructuralAnalyzer analyzer;

// Build structure
for (int y = 0; y < 10; y++) {
    world.SetVoxel(Vector3(0, y * 0.05f, 0), Voxel(MaterialDatabase::BRICK));
}

// Apply damage
Vector3 damage_pos(0, 0.25f, 0);
world.RemoveVoxel(damage_pos);

// Analyze
auto result = analyzer.Analyze(world, {damage_pos});

// Handle results
if (result.structure_failed) {
    for (const auto& cluster : result.failed_clusters) {
        // Spawn physics bodies, remove voxels, etc.
    }
}
```

### Using Custom Parameters

```cpp
// Option 1: Set parameters directly
analyzer.params.timestep = 0.005f;
analyzer.params.damping = 0.95f;
analyzer.params.max_iterations = 150;

// Option 2: Load from file
analyzer.LoadParameters("config/optimal_params.ini");

// Option 3: Use ParameterTuning for auto-tuning
#include "ParameterTuning.h"
auto tuning_result = ParameterTuning::AutoTuneParameters(world, damaged, 50);
analyzer.params = tuning_result.optimal_params;
```

### Performance Monitoring

```cpp
auto result = analyzer.Analyze(world, damaged);

std::cout << "Performance Report:\n";
std::cout << "  Total time: " << result.calculation_time_ms << "ms\n";
std::cout << "  Node graph: " << result.node_graph_time_ms << "ms\n";
std::cout << "  Mass calc: " << result.mass_calc_time_ms << "ms\n";
std::cout << "  Solver: " << result.solver_time_ms << "ms\n";
std::cout << "  Failure detect: " << result.failure_detection_time_ms << "ms\n";
std::cout << "  Nodes: " << result.nodes_analyzed << "\n";
std::cout << "  Iterations: " << result.iterations_used << "\n";
std::cout << "  Converged: " << result.converged << "\n";

int hits, misses;
analyzer.GetCacheStats(hits, misses);
float hit_rate = (float)hits / (hits + misses);
std::cout << "  Cache hit rate: " << (hit_rate * 100.0f) << "%\n";
```

### Handling Edge Cases

```cpp
// Empty damage list
std::vector<Vector3> empty_damage;
auto result1 = analyzer.Analyze(world, empty_damage);
// Returns immediately with structure_failed = false

// Single voxel
world.SetVoxel(Vector3(0, 1, 0), Voxel(MaterialDatabase::WOOD));
auto result2 = analyzer.Analyze(world, {Vector3(0, 0.5f, 0)});
// Uses fast-path for single-node analysis

// Invalid parameters
analyzer.params.timestep = -0.01f;  // Invalid
analyzer.ValidateParameters();      // Auto-corrects to 0.0001f
```

## Performance Tips

1. **Use `influence_radius`**: Set to smallest radius that captures relevant structure
   - Too small: Misses important connections
   - Too large: Analyzes unnecessary nodes
   - Typical: 3-10 meters

2. **Enable parallel mass calculation**: Enabled by default, gives 2-3x speedup
   ```cpp
   analyzer.params.use_parallel_mass_calc = true;
   ```

3. **Use surface-only analysis**: Huge speedup, enabled by default
   ```cpp
   analyzer.params.use_surface_only = true;
   ```

4. **Cache management**: Clear cache when world changes significantly
   ```cpp
   analyzer.ClearMassCache();  // When adding/removing many voxels
   ```

5. **Tune parameters**: Use auto-tuning for optimal configuration
   ```cpp
   auto result = ParameterTuning::AutoTuneParameters(world, test_damage, 50);
   analyzer.params = result.optimal_params;
   ```

6. **Monitor performance**: Check profiling data to identify bottlenecks
   ```cpp
   if (result.mass_calc_time_ms > 50.0f) {
       // Mass calculation is slow, consider smaller influence_radius
   }
   ```

## Thread Safety

The `StructuralAnalyzer` is **not thread-safe**. Use one of these patterns:

### Pattern 1: One Analyzer Per Thread
```cpp
#pragma omp parallel for
for (int i = 0; i < num_structures; i++) {
    StructuralAnalyzer analyzer;  // Thread-local
    auto result = analyzer.Analyze(worlds[i], damages[i]);
    // Process result...
}
```

### Pattern 2: Analyzer Pool
```cpp
std::vector<StructuralAnalyzer> analyzer_pool(num_threads);

// In parallel region
int thread_id = omp_get_thread_num();
auto result = analyzer_pool[thread_id].Analyze(world, damage);
```

## See Also

- [Usage Guide](USAGE_GUIDE.md) - Practical examples and patterns
- [Parameter Tuning Guide](PARAMETER_TUNING_GUIDE.md) - How to optimize parameters
- [PhysX Integration Guide](PHYSX_INTEGRATION_GUIDE.md) - Integrating with PhysX
- [8-Week Plan](8-Week-Plan.md) - Development timeline and features
