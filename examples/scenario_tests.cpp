/**
 * Week 7 Day 33: Test Scenarios & Polish
 *
 * Comprehensive validation system that tests the complete destruction pipeline
 * with multiple realistic scenarios.
 *
 * Features:
 * - Tower collapse (complete failure)
 * - Building column removal (partial collapse)
 * - Bridge support removal (span failure)
 * - Wall with redundancy (no collapse)
 * - Large-scale stress test
 * - Automated pass/fail validation
 */

#include "VoxelWorld.h"
#include "StructuralAnalyzer.h"
#include "VoxelPhysicsIntegration.h"
#include "VoxelUtils.h"
#include "Material.h"

#ifdef USE_BULLET
#include "BulletEngine.h"
#else
#include "MockPhysicsEngine.h"
#endif

#include <iostream>
#include <functional>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <cmath>

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;

float GetElapsedMS(TimePoint start) {
    auto end = Clock::now();
    return std::chrono::duration<float, std::milli>(end - start).count();
}

// ===== Test Scenario Structure =====

struct TestScenario {
    std::string name;
    std::string description;
    std::function<void(VoxelWorld&)> setup;
    std::function<std::vector<Vector3>(const VoxelWorld&)> damage;
    bool should_collapse;
    int min_debris_count;
    int max_debris_count;
};

struct TestResult {
    std::string scenario_name;
    bool passed;
    bool structure_failed;
    int debris_count;
    float analysis_time_ms;
    float physics_time_ms;
    std::string failure_reason;
};

// ===== Structure Creation Helpers =====

void CreateTestTower(VoxelWorld& world, int height) {
    // Simple vertical tower (1x1 base, height voxels tall)
    for (int y = 0; y < height; y++) {
        world.SetVoxel(Vector3(0, y * 0.05f, 0),
                      Voxel(MaterialDatabase::CONCRETE));
    }
}

void CreateTestBuilding(VoxelWorld& world, int width, int depth, int height) {
    // Building with 4 corner columns and floors
    uint8_t material = MaterialDatabase::CONCRETE;

    // Corner columns
    std::vector<Vector3> corners = {
        Vector3(0, 0, 0),
        Vector3((width-1) * 0.05f, 0, 0),
        Vector3(0, 0, (depth-1) * 0.05f),
        Vector3((width-1) * 0.05f, 0, (depth-1) * 0.05f)
    };

    for (auto& corner : corners) {
        for (int y = 0; y < height; y++) {
            Vector3 pos = corner + Vector3(0, y * 0.05f, 0);
            world.SetVoxel(pos, Voxel(material));
        }
    }

    // Floors every 3 voxels
    for (int y = 0; y < height; y += 3) {
        for (int x = 0; x < width; x++) {
            for (int z = 0; z < depth; z++) {
                Vector3 pos(x * 0.05f, y * 0.05f, z * 0.05f);
                world.SetVoxel(pos, Voxel(material));
            }
        }
    }
}

void CreateTestBridge(VoxelWorld& world, int span_length) {
    // Bridge with 2 support pillars and a deck
    uint8_t material = MaterialDatabase::CONCRETE;
    int pillar_height = 10;

    // Left pillar
    for (int y = 0; y < pillar_height; y++) {
        world.SetVoxel(Vector3(0, y * 0.05f, 0),
                      Voxel(material));
    }

    // Right pillar
    for (int y = 0; y < pillar_height; y++) {
        world.SetVoxel(Vector3((span_length-1) * 0.05f, y * 0.05f, 0),
                      Voxel(material));
    }

    // Deck spanning between pillars
    for (int x = 0; x < span_length; x++) {
        world.SetVoxel(Vector3(x * 0.05f, pillar_height * 0.05f, 0),
                      Voxel(material));
        world.SetVoxel(Vector3(x * 0.05f, (pillar_height + 1) * 0.05f, 0),
                      Voxel(material));
    }
}

void CreateTestWall(VoxelWorld& world, int width, int height) {
    // Redundant wall (2D grid with many connections)
    uint8_t material = MaterialDatabase::BRICK;

    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            world.SetVoxel(Vector3(x * 0.05f, y * 0.05f, 0),
                          Voxel(material));
        }
    }
}

void CreateLargeStructure(VoxelWorld& world, int target_voxels) {
    // Hollow building that scales to target voxel count
    int dimension = std::max(5, (int)std::cbrt(target_voxels / 0.3f));
    int height = dimension / 2;

    uint8_t material = MaterialDatabase::CONCRETE;

    // Hollow box with floors
    for (int y = 0; y < height; y++) {
        bool is_floor = (y % 3 == 0);

        for (int x = 0; x < dimension; x++) {
            for (int z = 0; z < dimension; z++) {
                bool is_wall = (x == 0 || x == dimension-1 ||
                               z == 0 || z == dimension-1);

                if (is_floor || is_wall) {
                    Vector3 pos(x * 0.05f, y * 0.05f, z * 0.05f);
                    world.SetVoxel(pos, Voxel(material));

                    if (world.GetVoxelCount() >= target_voxels) {
                        return;
                    }
                }
            }
        }
    }
}

// ===== Test Scenarios =====

std::vector<TestScenario> CreateTestScenarios() {
    std::vector<TestScenario> scenarios;

    // Scenario 1: Tower Collapse
    scenarios.push_back({
        "Tower Collapse",
        "20-voxel vertical tower with base removed",
        [](VoxelWorld& world) {
            CreateTestTower(world, 20);
        },
        [](const VoxelWorld& world) {
            // Remove bottom 2 voxels
            return std::vector<Vector3>{
                Vector3(0, 0, 0),
                Vector3(0, 0.05f, 0)
            };
        },
        true,  // Should collapse
        1,     // At least 1 debris
        2      // At most 2 debris clusters
    });

    // Scenario 2: Building Corner Column
    scenarios.push_back({
        "Building Corner Column",
        "Building with corner column destroyed",
        [](VoxelWorld& world) {
            CreateTestBuilding(world, 10, 10, 12);
        },
        [](const VoxelWorld& world) {
            // Remove entire corner column
            std::vector<Vector3> damaged;
            for (int y = 0; y < 12; y++) {
                damaged.push_back(Vector3(0, y * 0.05f, 0));
            }
            return damaged;
        },
        true,  // Should collapse
        1,     // At least 1 debris cluster
        5      // At most 5 (could fragment)
    });

    // Scenario 3: Bridge Partial Collapse
    scenarios.push_back({
        "Bridge Support Removal",
        "Bridge with one support pillar removed",
        [](VoxelWorld& world) {
            CreateTestBridge(world, 20);
        },
        [](const VoxelWorld& world) {
            // Remove entire left pillar
            std::vector<Vector3> damaged;
            for (int y = 0; y < 10; y++) {
                damaged.push_back(Vector3(0, y * 0.05f, 0));
            }
            return damaged;
        },
        true,  // Should collapse
        1,     // At least 1 debris cluster
        3      // At most 3 (deck fragments)
    });

    // Scenario 4: Redundant Wall (Should NOT Collapse)
    scenarios.push_back({
        "Redundant Wall Stability",
        "Wall with minor damage remains standing",
        [](VoxelWorld& world) {
            CreateTestWall(world, 20, 10);
        },
        [](const VoxelWorld& world) {
            // Remove just 5 random interior voxels
            return std::vector<Vector3>{
                Vector3(5 * 0.05f, 3 * 0.05f, 0),
                Vector3(8 * 0.05f, 5 * 0.05f, 0),
                Vector3(12 * 0.05f, 7 * 0.05f, 0),
                Vector3(15 * 0.05f, 4 * 0.05f, 0),
                Vector3(10 * 0.05f, 6 * 0.05f, 0)
            };
        },
        false, // Should NOT collapse
        0,     // No debris
        5      // Small fragments at most
    });

    // Scenario 5: Large-Scale Stress Test
    scenarios.push_back({
        "Large-Scale Building",
        "1000+ voxel structure stress test",
        [](VoxelWorld& world) {
            CreateLargeStructure(world, 1200);
        },
        [](const VoxelWorld& world) {
            // Remove bottom corner section
            std::vector<Vector3> damaged;
            for (int x = 0; x < 3; x++) {
                for (int y = 0; y < 3; y++) {
                    for (int z = 0; z < 3; z++) {
                        damaged.push_back(Vector3(x * 0.05f, y * 0.05f, z * 0.05f));
                    }
                }
            }
            return damaged;
        },
        true,  // Should collapse
        1,     // At least 1 debris
        20     // Could fragment significantly
    });

    return scenarios;
}

// ===== Scenario Runner =====

TestResult RunScenario(const TestScenario& scenario) {
    TestResult result;
    result.scenario_name = scenario.name;
    result.passed = false;
    result.failure_reason = "";

    try {
        // Setup world
        VoxelWorld world;
        scenario.setup(world);

        int initial_voxels = world.GetVoxelCount();

        // Apply damage
        auto damaged_positions = scenario.damage(world);
        for (const auto& pos : damaged_positions) {
            world.RemoveVoxel(pos);
        }

        int voxels_after_damage = world.GetVoxelCount();

        // Track 1: Structural Analysis
        StructuralAnalyzer analyzer;
        auto start_analysis = Clock::now();
        auto analysis_result = analyzer.Analyze(world, damaged_positions, 500.0f);
        result.analysis_time_ms = GetElapsedMS(start_analysis);

        result.structure_failed = analysis_result.structure_failed;
        result.debris_count = analysis_result.failed_clusters.size();

        // Track 3: Physics Simulation (if structure failed)
        if (analysis_result.structure_failed) {
#ifdef USE_BULLET
            BulletEngine physics;
#else
            MockPhysicsEngine physics;
#endif
            physics.Initialize();
            physics.SetGravity(Vector3(0, -9.81f, 0));

            VoxelPhysicsIntegration integration(&physics, &world);

            auto start_physics = Clock::now();

            // Spawn debris
            integration.SpawnDebris(analysis_result.failed_clusters);

            // Simulate 3 seconds (180 frames at 60 FPS)
            for (int i = 0; i < 180; i++) {
                integration.Step(1.0f / 60.0f);
            }

            result.physics_time_ms = GetElapsedMS(start_physics);

            physics.Shutdown();
        } else {
            result.physics_time_ms = 0.0f;
        }

        // Validate results
        if (scenario.should_collapse) {
            // Expected collapse
            if (!result.structure_failed) {
                result.failure_reason = "Expected collapse, but structure remained stable";
            } else if (result.debris_count < scenario.min_debris_count) {
                result.failure_reason = "Too few debris clusters (got " +
                    std::to_string(result.debris_count) + ", expected >= " +
                    std::to_string(scenario.min_debris_count) + ")";
            } else if (result.debris_count > scenario.max_debris_count) {
                result.failure_reason = "Too many debris clusters (got " +
                    std::to_string(result.debris_count) + ", expected <= " +
                    std::to_string(scenario.max_debris_count) + ")";
            } else {
                result.passed = true;
            }
        } else {
            // Expected stability
            if (result.structure_failed && result.debris_count > scenario.max_debris_count) {
                result.failure_reason = "Expected stability, but structure collapsed with " +
                    std::to_string(result.debris_count) + " debris clusters";
            } else {
                result.passed = true;
            }
        }

    } catch (const std::exception& e) {
        result.passed = false;
        result.failure_reason = std::string("Exception: ") + e.what();
    }

    return result;
}

void PrintScenarioHeader(const TestScenario& scenario, int scenario_num, int total) {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  Scenario " << scenario_num << "/" << total << ": "
              << std::left << std::setw(43) << scenario.name << "║\n";
    std::cout << "╠══════════════════════════════════════════════════════════╣\n";
    std::cout << "║ " << std::left << std::setw(57) << scenario.description << "║\n";
    std::cout << "╚══════════════════════════════════════════════════════════╝\n";
}

void PrintTestResult(const TestResult& result) {
    std::cout << "\n";
    std::cout << "Results:\n";
    std::cout << "  Structure Failed: " << (result.structure_failed ? "YES" : "NO") << "\n";
    std::cout << "  Debris Clusters:  " << result.debris_count << "\n";
    std::cout << "  Analysis Time:    " << std::fixed << std::setprecision(2)
              << result.analysis_time_ms << " ms\n";

    if (result.structure_failed && result.physics_time_ms > 0) {
        std::cout << "  Physics Time:     " << std::fixed << std::setprecision(2)
                  << result.physics_time_ms << " ms\n";
        float fps = 180.0f / (result.physics_time_ms / 1000.0f);
        std::cout << "  Physics FPS:      " << std::fixed << std::setprecision(1)
                  << fps << " FPS\n";
    }

    std::cout << "\n";
    if (result.passed) {
        std::cout << "  ✓ PASSED\n";
    } else {
        std::cout << "  ✗ FAILED\n";
        if (!result.failure_reason.empty()) {
            std::cout << "  Reason: " << result.failure_reason << "\n";
        }
    }
}

void RunAllScenarios() {
    auto scenarios = CreateTestScenarios();
    std::vector<TestResult> results;

    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════╗\n";
    std::cout << "║     VOXEL DESTRUCTION ENGINE                             ║\n";
    std::cout << "║     Test Scenario Suite                                  ║\n";
    std::cout << "║     Week 7 Day 33                                        ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════╝\n";

    int scenario_num = 1;
    for (const auto& scenario : scenarios) {
        PrintScenarioHeader(scenario, scenario_num, scenarios.size());

        std::cout << "Running test...\n";
        auto result = RunScenario(scenario);
        results.push_back(result);

        PrintTestResult(result);

        scenario_num++;
    }

    // Summary
    int passed = 0;
    int failed = 0;
    float total_analysis_time = 0;
    float total_physics_time = 0;

    for (const auto& result : results) {
        if (result.passed) passed++;
        else failed++;

        total_analysis_time += result.analysis_time_ms;
        total_physics_time += result.physics_time_ms;
    }

    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════╗\n";
    std::cout << "║           TEST SUMMARY                                   ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Total Scenarios:     " << std::setw(34) << scenarios.size() << "║\n";
    std::cout << "║ Passed:              " << std::setw(34) << passed << "║\n";
    std::cout << "║ Failed:              " << std::setw(34) << failed << "║\n";
    std::cout << "║                                                          ║\n";
    std::cout << "║ Total Analysis Time: " << std::setw(28) << std::fixed << std::setprecision(2)
              << total_analysis_time << " ms  ║\n";
    std::cout << "║ Total Physics Time:  " << std::setw(28) << std::fixed << std::setprecision(2)
              << total_physics_time << " ms  ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════╝\n";

    if (failed == 0) {
        std::cout << "\n✓ All scenarios passed!\n\n";
    } else {
        std::cout << "\n✗ Some scenarios failed. Review results above.\n\n";
    }
}

// ===== Main =====

int main() {
    std::cout << "Voxel Destruction Engine - Scenario Test Suite\n";
    std::cout << "Week 7 Day 33: Test Scenarios & Polish\n";
    std::cout << "===========================================\n";

    RunAllScenarios();

    return 0;
}
