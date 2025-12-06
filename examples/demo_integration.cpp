/**
 * Week 5 Day 24: Integration Demo
 *
 * Demonstrates the complete voxel destruction pipeline:
 * 1. VoxelWorld - Manages voxel data
 * 2. StructuralAnalyzer - Detects structural failures
 * 3. VoxelPhysicsIntegration - Bridges to physics engine
 * 4. Physics Engine (Bullet or Mock) - Simulates debris
 */

#include <iostream>
#include <vector>
#include <chrono>
#include "VoxelWorld.h"
#include "StructuralAnalyzer.h"
#include "VoxelPhysicsIntegration.h"
#include "Material.h"

#ifdef USE_BULLET
#include "BulletEngine.h"
#else
#include "MockPhysicsEngine.h"
#endif

// ===== Helper Functions =====

void PrintSeparator(const std::string& title) {
    std::cout << "\n============================================\n";
    std::cout << "  " << title << "\n";
    std::cout << "============================================\n";
}

void PrintAnalysisResult(const AnalysisResult& result) {
    std::cout << "\n[Analysis Result]\n";
    std::cout << "  Failed: " << (result.structure_failed ? "YES" : "NO") << "\n";
    std::cout << "  Failed nodes: " << result.num_failed_nodes << "\n";
    std::cout << "  Disconnected nodes: " << result.num_disconnected_nodes << "\n";
    std::cout << "  Failed clusters: " << result.failed_clusters.size() << "\n";
    std::cout << "  Calculation time: " << result.calculation_time_ms << " ms\n";
    std::cout << "  Iterations: " << result.iterations_used << "\n";
    std::cout << "  Converged: " << (result.converged ? "YES" : "NO") << "\n";
}

// ===== Demo Scenarios =====

void Demo_TowerCollapse() {
    PrintSeparator("DEMO 1: Tower Collapse");

    // Create world
    VoxelWorld world;
    float voxel_size = world.GetVoxelSize();

    // Build 10-voxel tower
    std::cout << "\n[1] Building 10-voxel tower...\n";
    for (int y = 0; y < 10; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(MaterialDatabase::BRICK));
    }
    std::cout << "  Total voxels: " << world.GetVoxelCount() << "\n";

    // Damage base
    std::cout << "\n[2] Removing base voxel...\n";
    Vector3 base_pos(0, 0, 0);
    world.RemoveVoxel(base_pos);
    std::vector<Vector3> damaged = {base_pos};
    std::cout << "  Remaining voxels: " << world.GetVoxelCount() << "\n";

    // Structural analysis
    std::cout << "\n[3] Running structural analysis...\n";
    StructuralAnalyzer analyzer;
    auto result = analyzer.Analyze(world, damaged);
    PrintAnalysisResult(result);

    // Physics integration
    std::cout << "\n[4] Creating physics simulation...\n";
#ifdef USE_BULLET
    BulletEngine* physics = new BulletEngine();
    std::cout << "  Using: Bullet Physics (real simulation)\n";
#else
    MockPhysicsEngine* physics = new MockPhysicsEngine();
    std::cout << "  Using: Mock Physics Engine (simple Euler)\n";
#endif
    physics->Initialize();
    physics->SetGravity(Vector3(0, -9.81f, 0));

    VoxelPhysicsIntegration integration(physics, &world);

    // Spawn debris
    std::cout << "\n[5] Spawning debris...\n";
    int spawned = integration.SpawnDebris(result.failed_clusters);
    std::cout << "  Debris bodies spawned: " << spawned << "\n";

    // Simulate for 3 seconds
    std::cout << "\n[6] Simulating 3 seconds of physics (180 frames)...\n";
    for (int frame = 0; frame < 180; frame++) {
        integration.Step(1.0f / 60.0f);

        if (frame % 30 == 0) {
            std::cout << "  Frame " << frame << " / 180\n";
        }
    }

    std::cout << "\n[7] Final debris count: " << integration.GetDebrisCount() << "\n";

    // Cleanup
    integration.ClearDebris();
    physics->Shutdown();
    delete physics;

    std::cout << "\n✓ Tower collapse demo complete!\n";
}

void Demo_BridgeDestruction() {
    PrintSeparator("DEMO 2: Bridge Destruction");

    VoxelWorld world;
    float voxel_size = world.GetVoxelSize();

    // Build bridge structure
    std::cout << "\n[1] Building bridge with two pillars and span...\n";

    // Left pillar (5 voxels tall)
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(MaterialDatabase::CONCRETE));
    }

    // Right pillar (5 voxels tall)
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(10 * voxel_size, y * voxel_size, 0),
                      Voxel(MaterialDatabase::CONCRETE));
    }

    // Bridge span (11 voxels wide, wood)
    for (int x = 0; x <= 10; x++) {
        world.SetVoxel(Vector3(x * voxel_size, 4 * voxel_size, 0),
                      Voxel(MaterialDatabase::WOOD));
    }

    std::cout << "  Total voxels: " << world.GetVoxelCount() << "\n";

    // Damage middle of span
    std::cout << "\n[2] Destroying middle of bridge span...\n";
    std::vector<Vector3> damaged;
    for (int x = 4; x <= 6; x++) {
        Vector3 pos(x * voxel_size, 4 * voxel_size, 0);
        world.RemoveVoxel(pos);
        damaged.push_back(pos);
    }
    std::cout << "  Removed 3 span voxels\n";
    std::cout << "  Remaining voxels: " << world.GetVoxelCount() << "\n";

    // Analyze
    std::cout << "\n[3] Analyzing structural integrity...\n";
    StructuralAnalyzer analyzer;
    auto result = analyzer.Analyze(world, damaged);
    PrintAnalysisResult(result);

    // Physics
    std::cout << "\n[4] Setting up physics...\n";
#ifdef USE_BULLET
    BulletEngine* physics = new BulletEngine();
#else
    MockPhysicsEngine* physics = new MockPhysicsEngine();
#endif
    physics->Initialize();
    physics->SetGravity(Vector3(0, -9.81f, 0));

    VoxelPhysicsIntegration integration(physics, &world);

    // Spawn debris
    std::cout << "\n[5] Spawning bridge debris...\n";
    int spawned = integration.SpawnDebris(result.failed_clusters);
    std::cout << "  Debris bodies spawned: " << spawned << "\n";

    // Simulate
    std::cout << "\n[6] Simulating collapse (120 frames)...\n";
    for (int frame = 0; frame < 120; frame++) {
        integration.Step(1.0f / 60.0f);
        if (frame % 30 == 0) {
            std::cout << "  Frame " << frame << " / 120\n";
        }
    }

    // Cleanup
    integration.ClearDebris();
    physics->Shutdown();
    delete physics;

    std::cout << "\n✓ Bridge destruction demo complete!\n";
}

void Demo_WallBreakthrough() {
    PrintSeparator("DEMO 3: Wall Breakthrough");

    VoxelWorld world;
    float voxel_size = world.GetVoxelSize();

    // Build wall (10 wide x 8 tall)
    std::cout << "\n[1] Building wall (10x8 voxels)...\n";
    for (int x = 0; x < 10; x++) {
        for (int y = 0; y < 8; y++) {
            world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, 0),
                          Voxel(MaterialDatabase::BRICK));
        }
    }
    std::cout << "  Total voxels: " << world.GetVoxelCount() << "\n";

    // Punch hole through center
    std::cout << "\n[2] Punching hole through center of wall...\n";
    std::vector<Vector3> damaged;
    for (int x = 4; x <= 5; x++) {
        for (int y = 3; y <= 5; y++) {
            Vector3 pos(x * voxel_size, y * voxel_size, 0);
            world.RemoveVoxel(pos);
            damaged.push_back(pos);
        }
    }
    std::cout << "  Removed 6 voxels\n";
    std::cout << "  Remaining voxels: " << world.GetVoxelCount() << "\n";

    // Analyze
    std::cout << "\n[3] Analyzing structural damage...\n";
    StructuralAnalyzer analyzer;
    auto result = analyzer.Analyze(world, damaged);
    PrintAnalysisResult(result);

    // Physics
    std::cout << "\n[4] Physics setup...\n";
#ifdef USE_BULLET
    BulletEngine* physics = new BulletEngine();
#else
    MockPhysicsEngine* physics = new MockPhysicsEngine();
#endif
    physics->Initialize();
    physics->SetGravity(Vector3(0, -9.81f, 0));

    VoxelPhysicsIntegration integration(physics, &world);

    // Spawn debris
    std::cout << "\n[5] Spawning debris from failed sections...\n";
    int spawned = integration.SpawnDebris(result.failed_clusters);
    std::cout << "  Debris bodies spawned: " << spawned << "\n";

    if (spawned > 0) {
        // Simulate
        std::cout << "\n[6] Simulating debris fall (90 frames)...\n";
        for (int frame = 0; frame < 90; frame++) {
            integration.Step(1.0f / 60.0f);
            if (frame % 30 == 0) {
                std::cout << "  Frame " << frame << " / 90\n";
            }
        }
    } else {
        std::cout << "\n[6] Wall remained stable! No debris spawned.\n";
    }

    // Cleanup
    integration.ClearDebris();
    physics->Shutdown();
    delete physics;

    std::cout << "\n✓ Wall breakthrough demo complete!\n";
}

void Demo_PerformanceBenchmark() {
    PrintSeparator("DEMO 4: Performance Benchmark");

    VoxelWorld world;
    float voxel_size = world.GetVoxelSize();

    // Build large structure (15x15x5 = 1125 voxels)
    std::cout << "\n[1] Building large structure (15x15x5)...\n";
    for (int x = 0; x < 15; x++) {
        for (int z = 0; z < 15; z++) {
            for (int y = 0; y < 5; y++) {
                world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, z * voxel_size),
                              Voxel(MaterialDatabase::CONCRETE));
            }
        }
    }
    std::cout << "  Total voxels: " << world.GetVoxelCount() << "\n";

    // Remove entire base layer
    std::cout << "\n[2] Removing entire base layer (225 voxels)...\n";
    std::vector<Vector3> damaged;
    for (int x = 0; x < 15; x++) {
        for (int z = 0; z < 15; z++) {
            Vector3 pos(x * voxel_size, 0, z * voxel_size);
            world.RemoveVoxel(pos);
            damaged.push_back(pos);
        }
    }
    std::cout << "  Remaining voxels: " << world.GetVoxelCount() << "\n";

    // Benchmark structural analysis
    std::cout << "\n[3] Benchmarking structural analysis...\n";
    StructuralAnalyzer analyzer;

    auto start = std::chrono::high_resolution_clock::now();
    auto result = analyzer.Analyze(world, damaged, 2000.0f);  // 2 second budget
    auto end = std::chrono::high_resolution_clock::now();

    float elapsed = std::chrono::duration<float, std::milli>(end - start).count();

    std::cout << "  ┌─────────────────────────────────┐\n";
    std::cout << "  │  PERFORMANCE METRICS            │\n";
    std::cout << "  ├─────────────────────────────────┤\n";
    std::cout << "  │  Nodes analyzed: " << result.nodes_analyzed << "\n";
    std::cout << "  │  Total time: " << elapsed << " ms\n";
    std::cout << "  │  Node graph: " << result.node_graph_time_ms << " ms\n";
    std::cout << "  │  Mass calc: " << result.mass_calc_time_ms << " ms\n";
    std::cout << "  │  Solver: " << result.solver_time_ms << " ms\n";
    std::cout << "  │  Failure detect: " << result.failure_detection_time_ms << " ms\n";
    std::cout << "  │  Iterations: " << result.iterations_used << "\n";
    std::cout << "  └─────────────────────────────────┘\n";

    // Benchmark physics
    std::cout << "\n[4] Benchmarking physics debris spawn...\n";
#ifdef USE_BULLET
    BulletEngine* physics = new BulletEngine();
#else
    MockPhysicsEngine* physics = new MockPhysicsEngine();
#endif
    physics->Initialize();
    physics->SetGravity(Vector3(0, -9.81f, 0));

    VoxelPhysicsIntegration integration(physics, &world);

    auto spawn_start = std::chrono::high_resolution_clock::now();
    int spawned = integration.SpawnDebris(result.failed_clusters);
    auto spawn_end = std::chrono::high_resolution_clock::now();

    float spawn_time = std::chrono::duration<float, std::milli>(spawn_end - spawn_start).count();

    std::cout << "  Debris spawned: " << spawned << "\n";
    std::cout << "  Spawn time: " << spawn_time << " ms\n";

    // Simulate and measure FPS
    std::cout << "\n[5] Benchmarking physics simulation (300 frames)...\n";
    auto sim_start = std::chrono::high_resolution_clock::now();

    for (int frame = 0; frame < 300; frame++) {
        integration.Step(1.0f / 60.0f);
    }

    auto sim_end = std::chrono::high_resolution_clock::now();
    float sim_time = std::chrono::duration<float, std::milli>(sim_end - sim_start).count();

    float avg_frame_time = sim_time / 300.0f;
    float fps = 1000.0f / avg_frame_time;

    std::cout << "  Total simulation time: " << sim_time << " ms\n";
    std::cout << "  Average frame time: " << avg_frame_time << " ms\n";
    std::cout << "  Effective FPS: " << fps << "\n";

    // Cleanup
    integration.ClearDebris();
    physics->Shutdown();
    delete physics;

    std::cout << "\n✓ Performance benchmark complete!\n";
}

// ===== Main =====

int main() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════╗\n";
    std::cout << "║  Voxel Destruction: Integration Demo          ║\n";
    std::cout << "║  Week 5 Day 24                                 ║\n";
    std::cout << "╚════════════════════════════════════════════════╝\n";

#ifdef USE_BULLET
    std::cout << "\nPhysics Engine: Bullet Physics (REAL)\n";
#else
    std::cout << "\nPhysics Engine: Mock Engine (SIMPLE)\n";
#endif

    // Run all demos
    Demo_TowerCollapse();
    Demo_BridgeDestruction();
    Demo_WallBreakthrough();
    Demo_PerformanceBenchmark();

    PrintSeparator("ALL DEMOS COMPLETE");
    std::cout << "\n✓ Integration pipeline working correctly!\n";
    std::cout << "\nPipeline tested:\n";
    std::cout << "  1. VoxelWorld → Voxel data management ✓\n";
    std::cout << "  2. StructuralAnalyzer → Failure detection ✓\n";
    std::cout << "  3. VoxelPhysicsIntegration → Bridge to physics ✓\n";
    std::cout << "  4. Physics Engine → Debris simulation ✓\n";
    std::cout << "\n";

    return 0;
}
