# Voxel Destruction Engine - Code Examples

**Week 8 Day 36: Documentation**

Practical code examples for common use cases.

---

## Table of Contents

1. [Basic Destruction](#basic-destruction)
2. [Material-Specific Behaviors](#material-specific-behaviors)
3. [Turn-Based Integration](#turn-based-integration)
4. [Performance Optimization](#performance-optimization)
5. [Visual Feedback](#visual-feedback)
6. [Debris Management](#debris-management)
7. [Complete Game Loop](#complete-game-loop)

---

## Basic Destruction

### Example 1: Destroy a Building Column

```cpp
#include "VoxelWorld.h"
#include "StructuralAnalyzer.h"
#include "VoxelPhysicsIntegration.h"
#include "BulletEngine.h"

void DestroyBuildingColumn() {
    // Create world
    VoxelWorld world;

    // Build 10x10x15 building with corner columns
    for (int y = 0; y < 15; y++) {
        // Four corner columns
        world.SetVoxel(Vector3(0, y * 0.05f, 0), Voxel(MaterialDatabase::CONCRETE));
        world.SetVoxel(Vector3(0.45f, y * 0.05f, 0), Voxel(MaterialDatabase::CONCRETE));
        world.SetVoxel(Vector3(0, y * 0.05f, 0.45f), Voxel(MaterialDatabase::CONCRETE));
        world.SetVoxel(Vector3(0.45f, y * 0.05f, 0.45f), Voxel(MaterialDatabase::CONCRETE));

        // Add floors every 3 levels
        if (y % 3 == 0) {
            for (int x = 0; x < 10; x++) {
                for (int z = 0; z < 10; z++) {
                    world.SetVoxel(Vector3(x * 0.05f, y * 0.05f, z * 0.05f),
                                  Voxel(MaterialDatabase::CONCRETE));
                }
            }
        }
    }

    std::cout << "Building created: " << world.GetVoxelCount() << " voxels\n";

    // Destroy one corner column (explosive damage)
    std::vector<Vector3> damaged;
    for (int y = 0; y < 15; y++) {
        damaged.push_back(Vector3(0, y * 0.05f, 0));
        world.RemoveVoxel(Vector3(0, y * 0.05f, 0));
    }

    std::cout << "Column destroyed, analyzing...\n";

    // Run structural analysis
    StructuralAnalyzer analyzer;
    auto result = analyzer.Analyze(world, damaged, 500.0f);

    std::cout << "Analysis time: " << result.calculation_time_ms << " ms\n";
    std::cout << "Structure failed: " << (result.structure_failed ? "YES" : "NO") << "\n";

    if (result.structure_failed) {
        std::cout << "Failed clusters: " << result.failed_clusters.size() << "\n";

        // Simulate physics
        BulletEngine physics;
        physics.Initialize();
        VoxelPhysicsIntegration integration(&physics, &world);

        int debris_count = integration.SpawnDebris(result.failed_clusters);
        std::cout << "Spawned " << debris_count << " debris objects\n";

        // Simulate 3 seconds
        for (int i = 0; i < 180; i++) {
            integration.Step(1.0f / 60.0f);
        }

        std::cout << "Collapse complete\n";
        physics.Shutdown();
    }
}
```

---

## Material-Specific Behaviors

### Example 2: Compare Wood vs Concrete

```cpp
void CompareMaterials() {
    // Test wood tower
    {
        VoxelWorld world;
        for (int y = 0; y < 10; y++) {
            world.SetVoxel(Vector3(0, y * 0.05f, 0), Voxel(MaterialDatabase::WOOD));
        }

        std::vector<Vector3> damaged = {Vector3(0, 0, 0)};
        world.RemoveVoxel(Vector3(0, 0, 0));

        StructuralAnalyzer analyzer;
        auto result = analyzer.Analyze(world, damaged);

        std::cout << "Wood tower (base removed): "
                  << (result.structure_failed ? "COLLAPSED" : "STABLE") << "\n";
    }

    // Test concrete tower
    {
        VoxelWorld world;
        for (int y = 0; y < 10; y++) {
            world.SetVoxel(Vector3(0, y * 0.05f, 0), Voxel(MaterialDatabase::CONCRETE));
        }

        std::vector<Vector3> damaged = {Vector3(0, 0, 0)};
        world.RemoveVoxel(Vector3(0, 0, 0));

        StructuralAnalyzer analyzer;
        auto result = analyzer.Analyze(world, damaged);

        std::cout << "Concrete tower (base removed): "
                  << (result.structure_failed ? "COLLAPSED" : "STABLE") << "\n";
    }
}
```

### Example 3: Custom Material

```cpp
void UseCustomMaterial() {
    // Define custom material
    Material reinforced_concrete;
    reinforced_concrete.name = "Reinforced Concrete";
    reinforced_concrete.density = 2500.0f;            // kg/m³
    reinforced_concrete.max_compression = 50000000.0f; // Pa (50 MPa)
    reinforced_concrete.yield_strength = 40000000.0f;  // Pa (40 MPa)
    reinforced_concrete.fracture_toughness = 1000.0f;  // J/m²

    // Register with material database
    uint8_t custom_id = MaterialDatabase::Register(reinforced_concrete);

    // Use in voxel world
    VoxelWorld world;
    for (int y = 0; y < 20; y++) {
        world.SetVoxel(Vector3(0, y * 0.05f, 0), Voxel(custom_id));
    }

    // This tower will be much stronger than regular concrete
}
```

---

## Turn-Based Integration

### Example 4: XCOM-Style Turn System

```cpp
class GameTurnManager {
private:
    VoxelWorld world;
    StructuralAnalyzer analyzer;
    BulletEngine physics;
    VoxelPhysicsIntegration integration;
    TurnManager turn_manager;

public:
    GameTurnManager() : integration(&physics, &world) {
        physics.Initialize();
    }

    void PlayerTurn() {
        std::cout << "=== PLAYER TURN ===\n";

        // Player moves, shoots, etc.
        // ...

        // Player destroys voxels with explosive
        ApplyExplosiveDamage(Vector3(10, 5, 10), 2.0f);
    }

    void ApplyExplosiveDamage(Vector3 center, float radius) {
        // Find voxels in radius
        std::vector<Vector3> damaged;
        auto nearby = world.GetVoxelsInRadius(center, radius);

        for (const auto& pos : nearby) {
            if (world.HasVoxel(pos)) {
                damaged.push_back(pos);
                world.RemoveVoxel(pos);
            }
        }

        if (damaged.empty()) return;

        std::cout << "Explosive destroyed " << damaged.size() << " voxels\n";

        // Trigger turn transition
        turn_manager.StartTurnTransition();

        // Run structural analysis
        std::cout << "Analyzing structural integrity...\n";
        auto result = analyzer.Analyze(world, damaged, 500.0f);

        turn_manager.NotifyStructuralAnalysisComplete(result.structure_failed);

        if (result.structure_failed) {
            std::cout << "Structure collapsing!\n";

            // Spawn debris
            integration.SpawnDebris(result.failed_clusters);

            // Fast-forward simulation (3 seconds in 0.1 real seconds)
            for (int i = 0; i < 180; i++) {
                integration.Step(1.0f / 60.0f);
            }
        }

        turn_manager.NotifyPhysicsSimulationComplete();

        std::cout << "Destruction complete, proceeding to AI turn...\n";
    }

    void AITurn() {
        std::cout << "=== AI TURN ===\n";
        // AI logic...

        turn_manager.EndAITurn();
    }
};
```

---

## Performance Optimization

### Example 5: Adaptive Quality Settings

```cpp
class AdaptiveDestruction {
private:
    StructuralAnalyzer analyzer;
    float frame_budget_ms;

public:
    AdaptiveDestruction(float budget_ms = 16.0f)
        : frame_budget_ms(budget_ms) {}

    AnalysisResult AnalyzeAdaptive(VoxelWorld& world,
                                    const std::vector<Vector3>& damaged) {
        // Try high quality first
        analyzer.SetMaxIterations(100);
        analyzer.SetTimestep(0.01f);

        auto start = std::chrono::high_resolution_clock::now();
        auto result = analyzer.Analyze(world, damaged, frame_budget_ms);
        auto elapsed = GetElapsedMS(start);

        // If too slow, remember to use lower quality next time
        if (elapsed > frame_budget_ms) {
            std::cout << "Reducing quality for next frame\n";
            analyzer.SetMaxIterations(50);
            analyzer.SetTimestep(0.02f);
        }

        return result;
    }
};
```

### Example 6: Debris Culling

```cpp
void UpdatePhysicsWithCulling(VoxelPhysicsIntegration& integration,
                               Vector3 camera_pos,
                               float max_distance) {
    // Remove debris that's too far from camera
    integration.RemoveDebrisBeyondDistance(camera_pos, max_distance);

    // Remove very old debris
    integration.RemoveDebrisOlderThan(10.0f); // 10 seconds

    // Normal physics update
    integration.Step(1.0f / 60.0f);

    std::cout << "Active debris: " << integration.GetActiveDebrisCount() << "\n";
}
```

---

## Visual Feedback

### Example 7: Camera Shake and Slow Motion

```cpp
#include "VisualFeedback.h"

void DestructionWithFeedback() {
    VoxelWorld world;
    StructuralAnalyzer analyzer;
    CameraEffects camera_fx;
    TimeScale time_scale;

    // ... destroy voxels ...

    auto result = analyzer.Analyze(world, damaged);

    if (result.structure_failed) {
        // Large collapse = big shake
        float shake_intensity = std::min(1.0f, result.failed_clusters.size() / 10.0f);
        camera_fx.TriggerShake(shake_intensity, 3.0f);

        // Slow motion for dramatic effect
        time_scale.SetSlowMotion(0.3f, 2.0f);

        // Simulate with time scaling
        BulletEngine physics;
        physics.Initialize();
        VoxelPhysicsIntegration integration(&physics, &world);
        integration.SpawnDebris(result.failed_clusters);

        for (int i = 0; i < 180; i++) {
            float scaled_dt = time_scale.GetScaledDelta(1.0f / 60.0f);
            integration.Step(scaled_dt);

            // Update camera shake
            camera_fx.Update(1.0f / 60.0f);
            auto shake_offset = camera_fx.GetShakeOffset();

            // Apply to camera
            camera.position += shake_offset;
        }

        physics.Shutdown();
    }
}
```

### Example 8: Damage Visualization

```cpp
#include "VisualFeedback.h"

void VisualizeStructuralFailure() {
    VoxelWorld world;
    StructuralAnalyzer analyzer;
    VoxelVisualizer visualizer;
    DestructionUI ui;

    // ... create structure and damage ...

    // Highlight damaged voxels in red
    visualizer.HighlightDamagedVoxels(world, damaged_positions);

    auto result = analyzer.Analyze(world, damaged);

    if (result.structure_failed) {
        // Show failed clusters
        visualizer.ShowFailedClusters(result.failed_clusters);

        // Display analysis UI
        ui.ShowStructuralAnalysis(result);

        // Animate cracks spreading
        for (const auto& cluster : result.failed_clusters) {
            visualizer.AnimateCracking(cluster.voxel_positions);
        }
    }
}
```

---

## Debris Management

### Example 9: Convert Settled Debris to Rubble

```cpp
#include "RubbleSystem.h"

void ManageDebrisSettling() {
    VoxelPhysicsIntegration integration(&physics, &world);
    RubbleSystem rubble_system;

    // Spawn initial debris
    integration.SpawnDebris(failed_clusters);

    // Simulate and check for settling
    float time = 0.0f;
    while (integration.GetActiveDebrisCount() > 0) {
        integration.Step(1.0f / 60.0f);
        time += 1.0f / 60.0f;

        // Every second, check for settled debris
        if ((int)time % 1 == 0) {
            int settled = integration.GetSettledDebrisCount();
            if (settled > 0) {
                std::cout << settled << " debris objects settled\n";

                // Convert to static rubble for gameplay
                integration.ConvertSettledToStatic();

                // Add to rubble system for cover mechanics
                for (int i = 0; i < settled; i++) {
                    // Get debris info before conversion
                    auto& debris = integration.GetDebris(i);
                    if (debris.state == DebrisState::SETTLED) {
                        rubble_system.AddRubble(
                            debris.position,
                            debris.height,
                            debris.radius,
                            debris.material
                        );
                    }
                }
            }
        }
    }

    std::cout << "All debris settled\n";
    std::cout << "Total rubble piles: " << rubble_system.GetRubbleCount() << "\n";
}
```

### Example 10: Cover System with Rubble

```cpp
bool PlayerHasCover(Vector3 player_pos, Vector3 enemy_pos, RubbleSystem& rubble) {
    // Check if rubble provides cover between player and enemy
    float cover_height = rubble.GetCoverHeight(player_pos, 1.0f);

    if (cover_height > 0.5f) { // 50cm is enough for cover
        std::cout << "Player has " << cover_height << "m of cover\n";
        return true;
    }

    return false;
}

void UpdateCoverPositions(std::vector<Vector3>& cover_positions,
                          RubbleSystem& rubble) {
    cover_positions.clear();

    // Find all positions with good cover
    for (float x = 0; x < 100; x += 0.5f) {
        for (float z = 0; z < 100; z += 0.5f) {
            Vector3 pos(x, 0, z);

            if (rubble.HasCover(pos, 0.5f, 1.0f)) {
                cover_positions.push_back(pos);
            }
        }
    }

    std::cout << "Found " << cover_positions.size() << " cover positions\n";
}
```

---

## Complete Game Loop

### Example 11: Full Integration

```cpp
class DestructibleGame {
private:
    VoxelWorld world;
    StructuralAnalyzer analyzer;
    BulletEngine physics;
    VoxelPhysicsIntegration integration;
    RubbleSystem rubble;
    CameraEffects camera_fx;
    TimeScale time_scale;

    bool analyzing = false;
    bool simulating = false;

public:
    DestructibleGame() : integration(&physics, &world) {
        physics.Initialize();
        physics.SetGravity(Vector3(0, -9.81f, 0));

        // Load level
        LoadLevel();
    }

    void LoadLevel() {
        // Build city block
        for (int b = 0; b < 5; b++) {
            BuildBuilding(Vector3(b * 20.0f, 0, 0), 10, 10, 15);
        }

        std::cout << "Level loaded: " << world.GetVoxelCount() << " voxels\n";
    }

    void Update(float deltaTime) {
        // Always update physics
        if (simulating) {
            float scaled_dt = time_scale.GetScaledDelta(deltaTime);
            integration.Step(scaled_dt);

            // Check if settled
            if (integration.GetActiveDebrisCount() == 0) {
                simulating = false;
                std::cout << "Simulation complete\n";
            }
        }

        // Update camera effects
        camera_fx.Update(deltaTime);
    }

    void OnExplosion(Vector3 center, float radius, float damage) {
        // Apply damage
        auto damaged = DamageVoxelsInRadius(center, radius, damage);

        if (damaged.empty()) return;

        // Start structural analysis
        analyzing = true;

        auto result = analyzer.Analyze(world, damaged, 100.0f);

        analyzing = false;

        if (result.structure_failed) {
            // Trigger effects
            float shake = std::min(1.0f, result.failed_clusters.size() / 20.0f);
            camera_fx.TriggerShake(shake, 2.0f);
            time_scale.SetSlowMotion(0.5f, 1.5f);

            // Spawn debris
            integration.SpawnDebris(result.failed_clusters);
            simulating = true;

            std::cout << "Building collapsing! " << result.failed_clusters.size()
                      << " clusters falling\n";
        }
    }

    std::vector<Vector3> DamageVoxelsInRadius(Vector3 center, float radius, float damage) {
        std::vector<Vector3> damaged;
        auto nearby = world.GetVoxelsInRadius(center, radius);

        for (const auto& pos : nearby) {
            if (world.HasVoxel(pos)) {
                auto voxel = world.GetVoxel(pos);
                voxel.TakeDamage((uint8_t)damage);

                if (!voxel.is_active) {
                    world.RemoveVoxel(pos);
                    damaged.push_back(pos);
                }
            }
        }

        return damaged;
    }

    void BuildBuilding(Vector3 origin, int width, int depth, int height) {
        // Floor and corner columns
        for (int y = 0; y < height; y++) {
            // Columns
            world.SetVoxel(origin + Vector3(0, y * 0.05f, 0),
                          Voxel(MaterialDatabase::CONCRETE));
            world.SetVoxel(origin + Vector3((width-1) * 0.05f, y * 0.05f, 0),
                          Voxel(MaterialDatabase::CONCRETE));
            world.SetVoxel(origin + Vector3(0, y * 0.05f, (depth-1) * 0.05f),
                          Voxel(MaterialDatabase::CONCRETE));
            world.SetVoxel(origin + Vector3((width-1) * 0.05f, y * 0.05f, (depth-1) * 0.05f),
                          Voxel(MaterialDatabase::CONCRETE));

            // Floors every 3 levels
            if (y % 3 == 0) {
                for (int x = 0; x < width; x++) {
                    for (int z = 0; z < depth; z++) {
                        world.SetVoxel(origin + Vector3(x * 0.05f, y * 0.05f, z * 0.05f),
                                      Voxel(MaterialDatabase::CONCRETE));
                    }
                }
            }
        }
    }
};

int main() {
    DestructibleGame game;

    // Game loop (simplified)
    while (true) {
        game.Update(1.0f / 60.0f);

        // Simulate player firing explosive
        if (PlayerPressedFireButton()) {
            game.OnExplosion(GetAimPosition(), 3.0f, 255.0f);
        }

        // Render, etc...
    }
}
```

---

## Performance Tips

1. **Reduce analysis radius** for faster structural checks
2. **Use debris culling** to remove distant/old debris
3. **Convert settled debris to rubble** for static cover
4. **Disable debris-debris collisions** for 2-3x speedup
5. **Pre-compute weak points** during level load

---

## Next Steps

- Read [Parameter Tuning Guide](PARAMETER_TUNING_GUIDE.md) for optimization
- See [API Documentation](STRUCTURAL_ANALYZER_API.md) for complete reference
- Run `./bin/ShowcaseDemo` to see all features in action

---

**Happy Destroying!** 💥
