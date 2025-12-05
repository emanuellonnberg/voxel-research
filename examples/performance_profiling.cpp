// Week 7 Day 34: Performance Profiling & Large-Scale Verification
//
// Tests the complete destruction pipeline with increasingly large structures:
// - Small: 1,000 voxels
// - Medium: 10,000 voxels
// - Large: 50,000 voxels
//
// Measures:
// - Track 1 (Structural Analysis) timing
// - Track 3 (Physics Simulation) timing and FPS
// - Memory usage
// - Scalability

#include "VoxelWorld.h"
#include "VoxelUtils.h"
#include "StructuralAnalyzer.h"

#ifdef USE_BULLET
#include "BulletEngine.h"
#include "VoxelPhysicsIntegration.h"
#include "ParticleSystem.h"
#include "RubbleSystem.h"
#include "VisualFeedback.h"
#else
#include "MockPhysicsEngine.h"
#endif

#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <cmath>

// ============================================================================
// Timing Utilities
// ============================================================================

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;

float GetElapsedMS(TimePoint start) {
    auto end = Clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    return duration.count() / 1000.0f;
}

// ============================================================================
// Large Structure Generation
// ============================================================================

void CreateLargeBuilding(VoxelWorld& world, int target_voxel_count) {
    std::cout << "Creating large structure with ~" << target_voxel_count << " voxels..." << std::endl;

    // Calculate dimensions to hit target count
    // Using a rectangular building shape
    int volume_per_voxel = 1;
    float scale = std::cbrt(static_cast<float>(target_voxel_count) / 0.3f); // 30% fill ratio

    int width = static_cast<int>(scale);
    int height = static_cast<int>(scale * 0.6f); // Taller than wide
    int depth = static_cast<int>(scale * 0.8f);

    std::cout << "  Dimensions: " << width << "x" << height << "x" << depth << std::endl;

    int placed = 0;

    // Create a hollow building with floors and walls
    for (int y = 0; y < height; y++) {
        bool is_floor = (y % 3 == 0); // Floor every 3 voxels

        for (int x = 0; x < width; x++) {
            for (int z = 0; z < depth; z++) {
                bool is_wall = (x == 0 || x == width - 1 || z == 0 || z == depth - 1);

                if (is_floor || is_wall) {
                    world.SetVoxel(Vector3(x, y, z), Voxel(2)); // Brick
                    placed++;

                    if (placed >= target_voxel_count) {
                        std::cout << "  Placed " << placed << " voxels" << std::endl;
                        return;
                    }
                }
            }
        }
    }

    std::cout << "  Placed " << placed << " voxels" << std::endl;
}

std::vector<Vector3> DamageStructure(VoxelWorld& world, int damage_count) {
    std::vector<Vector3> damaged;

    // Damage the base of the structure (load-bearing area)
    for (int i = 0; i < damage_count; i++) {
        Vector3 pos(i % 5, 0, i / 5);

        if (world.HasVoxel(pos)) {
            world.RemoveVoxel(pos);
            damaged.push_back(pos);
        }
    }

    return damaged;
}

// ============================================================================
// Performance Test Cases
// ============================================================================

struct TestResult {
    std::string name;
    int voxel_count;
    int damage_count;

    // Track 1 (Structural Analysis)
    float analysis_time_ms;
    bool structure_failed;
    int failed_clusters;
    int total_nodes;

    // Track 3 (Physics Simulation)
    int debris_spawned;
    float physics_time_ms;
    float fps_equivalent;
    int frames_simulated;

    // Overall
    float total_time_ms;
};

TestResult RunPerformanceTest(const std::string& name, int target_voxels, int damage_count) {
    TestResult result;
    result.name = name;
    result.voxel_count = target_voxels;
    result.damage_count = damage_count;

    std::cout << "\n╔════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  " << std::setw(45) << std::left << name << "║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════╣" << std::endl;

    auto test_start = Clock::now();

    // Create world and structure
    VoxelWorld world;
    CreateLargeBuilding(world, target_voxels);
    result.voxel_count = world.GetVoxelCount();

    // Apply damage
    auto damaged = DamageStructure(world, damage_count);
    result.damage_count = damaged.size();

    std::cout << "║ World created: " << std::setw(28)
              << (std::to_string(result.voxel_count) + " voxels") << "║" << std::endl;
    std::cout << "║ Damage applied: " << std::setw(27)
              << (std::to_string(result.damage_count) + " voxels") << "║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════╣" << std::endl;

    // ====================
    // Track 1: Structural Analysis
    // ====================
    std::cout << "║ Running Track 1 (Structural Analysis)...      ║" << std::endl;

    StructuralAnalyzer analyzer;

    auto track1_start = Clock::now();
    auto analysis_result = analyzer.Analyze(world, damaged, 500.0f);
    result.analysis_time_ms = GetElapsedMS(track1_start);

    result.structure_failed = analysis_result.structure_failed;
    result.failed_clusters = analysis_result.failed_clusters.size();
    result.total_nodes = 0; // Could extract from analyzer if exposed

    std::cout << "║ Analysis time: " << std::setw(30)
              << (std::to_string(static_cast<int>(result.analysis_time_ms)) + " ms") << "║" << std::endl;
    std::cout << "║ Status: " << std::setw(37)
              << (result.structure_failed ? "FAILED" : "STABLE") << "║" << std::endl;

    if (result.structure_failed) {
        std::cout << "║ Failed clusters: " << std::setw(28)
                  << std::to_string(result.failed_clusters) << "║" << std::endl;
    }

#ifdef USE_BULLET
    // ====================
    // Track 3: Physics Simulation
    // ====================
    if (result.structure_failed && result.failed_clusters > 0) {
        std::cout << "╠════════════════════════════════════════════════╣" << std::endl;
        std::cout << "║ Running Track 3 (Physics Simulation)...       ║" << std::endl;

        BulletEngine physics;
        physics.Initialize();

        VoxelPhysicsIntegration integration(&physics, &world);
        integration.SpawnDebris(analysis_result.failed_clusters);

        result.debris_spawned = integration.GetDebrisCount();
        std::cout << "║ Debris spawned: " << std::setw(29)
                  << std::to_string(result.debris_spawned) << "║" << std::endl;

        // Simulate 180 frames (3 seconds at 60 FPS)
        const int FRAME_COUNT = 180;
        const float TIMESTEP = 1.0f / 60.0f;

        auto track3_start = Clock::now();

        for (int i = 0; i < FRAME_COUNT; i++) {
            integration.Step(TIMESTEP);
        }

        result.physics_time_ms = GetElapsedMS(track3_start);
        result.frames_simulated = FRAME_COUNT;
        result.fps_equivalent = (FRAME_COUNT / (result.physics_time_ms / 1000.0f));

        std::cout << "║ Simulation time: " << std::setw(28)
                  << (std::to_string(static_cast<int>(result.physics_time_ms)) + " ms") << "║" << std::endl;
        std::cout << "║ FPS equivalent: " << std::setw(29)
                  << std::to_string(static_cast<int>(result.fps_equivalent)) << "║" << std::endl;

        physics.Shutdown();
    } else {
        std::cout << "╠════════════════════════════════════════════════╣" << std::endl;
        std::cout << "║ Structure stable - no physics needed           ║" << std::endl;
        result.debris_spawned = 0;
        result.physics_time_ms = 0.0f;
        result.fps_equivalent = 0.0f;
        result.frames_simulated = 0;
    }
#else
    std::cout << "╠════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Bullet Physics not available                   ║" << std::endl;
    result.debris_spawned = 0;
    result.physics_time_ms = 0.0f;
    result.fps_equivalent = 0.0f;
    result.frames_simulated = 0;
#endif

    result.total_time_ms = GetElapsedMS(test_start);

    std::cout << "╠════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Total time: " << std::setw(33)
              << (std::to_string(static_cast<int>(result.total_time_ms)) + " ms") << "║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════╝" << std::endl;

    return result;
}

// ============================================================================
// Report Generation
// ============================================================================

void GeneratePerformanceReport(const std::vector<TestResult>& results) {
    std::ofstream report("performance_report.txt");

    report << "═══════════════════════════════════════════════════════════\n";
    report << "  VOXEL DESTRUCTION ENGINE - PERFORMANCE REPORT\n";
    report << "  Week 7 Day 34: Large-Scale Verification\n";
    report << "═══════════════════════════════════════════════════════════\n\n";

    for (const auto& result : results) {
        report << "=== " << result.name << " ===\n\n";

        report << "Structure:\n";
        report << "  Voxel Count:   " << result.voxel_count << "\n";
        report << "  Damage Count:  " << result.damage_count << "\n\n";

        report << "Track 1 (Structural Analysis):\n";
        report << "  Analysis Time: " << std::fixed << std::setprecision(2)
               << result.analysis_time_ms << " ms\n";
        report << "  Status:        " << (result.structure_failed ? "FAILED" : "STABLE") << "\n";

        if (result.structure_failed) {
            report << "  Failed Clusters: " << result.failed_clusters << "\n";
        }
        report << "\n";

        if (result.debris_spawned > 0) {
            report << "Track 3 (Physics Simulation):\n";
            report << "  Debris Spawned:  " << result.debris_spawned << "\n";
            report << "  Simulation Time: " << std::fixed << std::setprecision(2)
                   << result.physics_time_ms << " ms (" << result.frames_simulated << " frames)\n";
            report << "  FPS Equivalent:  " << std::fixed << std::setprecision(1)
                   << result.fps_equivalent << " FPS\n\n";
        }

        report << "Total Time: " << std::fixed << std::setprecision(2)
               << result.total_time_ms << " ms\n";
        report << "\n───────────────────────────────────────────────────────────\n\n";
    }

    report << "\n═══════════════════════════════════════════════════════════\n";
    report << "  SUMMARY\n";
    report << "═══════════════════════════════════════════════════════════\n\n";

    report << "Test Cases: " << results.size() << "\n\n";

    report << "Performance Targets:\n";
    report << "  Track 1: < 500ms per analysis  ";
    bool track1_pass = true;
    for (const auto& r : results) {
        if (r.analysis_time_ms > 500.0f) track1_pass = false;
    }
    report << (track1_pass ? "[✓ PASS]" : "[✗ FAIL]") << "\n";

    report << "  Track 3: > 30 FPS simulation   ";
    bool track3_pass = true;
    for (const auto& r : results) {
        if (r.fps_equivalent > 0 && r.fps_equivalent < 30.0f) track3_pass = false;
    }
    report << (track3_pass ? "[✓ PASS]" : "[✗ FAIL]") << "\n";

    report << "\n";
    report.close();

    std::cout << "\n✓ Performance report saved to: performance_report.txt" << std::endl;
}

void PrintReport(const std::vector<TestResult>& results) {
    std::cout << "\n\n";
    std::cout << "╔══════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║           PERFORMANCE SUMMARY                            ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════╣" << std::endl;

    for (const auto& result : results) {
        std::cout << "║ " << std::setw(54) << std::left << result.name << " ║" << std::endl;
        std::cout << "║   Structural: " << std::setw(6) << std::right
                  << static_cast<int>(result.analysis_time_ms) << " ms"
                  << std::setw(33) << " " << "║" << std::endl;

        if (result.debris_spawned > 0) {
            std::cout << "║   Physics:    " << std::setw(6) << std::right
                      << static_cast<int>(result.physics_time_ms) << " ms ("
                      << std::setw(3) << static_cast<int>(result.fps_equivalent) << " FPS)"
                      << std::setw(15) << " " << "║" << std::endl;
        }
    }

    std::cout << "╚══════════════════════════════════════════════════════════╝" << std::endl;
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║     VOXEL DESTRUCTION ENGINE                             ║" << std::endl;
    std::cout << "║     Performance Profiling & Large-Scale Verification     ║" << std::endl;
    std::cout << "║     Week 7 Day 34                                        ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════╝" << std::endl;

    std::vector<TestResult> results;

    // Small test: 1,000 voxels
    results.push_back(RunPerformanceTest("Small Scale (1K voxels)", 1000, 25));

    // Medium test: 10,000 voxels
    results.push_back(RunPerformanceTest("Medium Scale (10K voxels)", 10000, 50));

    // Large test: 50,000 voxels
    results.push_back(RunPerformanceTest("Large Scale (50K voxels)", 50000, 100));

    // Print summary
    PrintReport(results);

    // Generate detailed report file
    GeneratePerformanceReport(results);

    std::cout << "\n✓ All performance tests complete!\n" << std::endl;

    return 0;
}
