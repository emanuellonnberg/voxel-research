/**
 * Analyzer Comparison Demo
 *
 * Compares Spring-Based vs Max-Flow structural analyzers:
 * - Performance (speed)
 * - Accuracy (detection results)
 * - Determinism (consistency)
 *
 * Week 8+: Max-Flow Implementation
 */

#include "StructuralAnalyzer.h"
#include "MaxFlowStructuralAnalyzer.h"
#include "VoxelWorld.h"
#include "VoxelUtils.h"
#include "Material.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>

// Test scenario structure
struct Scenario {
    std::string name;
    std::string description;
    std::function<void(VoxelWorld&, std::vector<Vector3>&)> setup;
};

// Comparison results
struct ComparisonResult {
    std::string scenario_name;

    // Spring analyzer
    bool spring_failed;
    int spring_clusters;
    float spring_time_ms;
    int spring_iterations;

    // Max-flow analyzer
    bool maxflow_failed;
    int maxflow_clusters;
    float maxflow_time_ms;
    int maxflow_iterations;

    // Comparison metrics
    bool same_result;
    float speedup;  // maxflow speed vs spring
};

void PrintHeader() {
    std::cout << "╔══════════════════════════════════════════════════════════╗\n";
    std::cout << "║                                                          ║\n";
    std::cout << "║          STRUCTURAL ANALYZER COMPARISON                  ║\n";
    std::cout << "║          Spring System vs Max-Flow Algorithm             ║\n";
    std::cout << "║                                                          ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════╝\n";
    std::cout << "\n";
}

void PrintScenarioHeader(const std::string& name, const std::string& desc) {
    std::cout << "┌──────────────────────────────────────────────────────────┐\n";
    std::cout << "│  " << std::left << std::setw(56) << name << "│\n";
    std::cout << "├──────────────────────────────────────────────────────────┤\n";
    std::cout << "│  " << std::left << std::setw(56) << desc << "│\n";
    std::cout << "└──────────────────────────────────────────────────────────┘\n";
}

void PrintComparison(const ComparisonResult& result) {
    std::cout << "\n";
    std::cout << "  ┌─ Spring System (Displacement-Based) ─────────────────┐\n";
    std::cout << "  │ Failed: " << std::setw(43) << (result.spring_failed ? "YES" : "NO") << " │\n";
    std::cout << "  │ Clusters: " << std::setw(41) << result.spring_clusters << " │\n";
    std::cout << "  │ Time: " << std::setw(39) << std::fixed << std::setprecision(2)
              << result.spring_time_ms << " ms │\n";
    std::cout << "  │ Iterations: " << std::setw(37) << result.spring_iterations << " │\n";
    std::cout << "  └───────────────────────────────────────────────────────┘\n";

    std::cout << "\n";
    std::cout << "  ┌─ Max-Flow Algorithm (Dinic's) ────────────────────────┐\n";
    std::cout << "  │ Failed: " << std::setw(43) << (result.maxflow_failed ? "YES" : "NO") << " │\n";
    std::cout << "  │ Clusters: " << std::setw(41) << result.maxflow_clusters << " │\n";
    std::cout << "  │ Time: " << std::setw(39) << std::fixed << std::setprecision(2)
              << result.maxflow_time_ms << " ms │\n";
    std::cout << "  │ Flow Iterations: " << std::setw(32) << result.maxflow_iterations << " │\n";
    std::cout << "  └───────────────────────────────────────────────────────┘\n";

    std::cout << "\n";
    std::cout << "  ┌─ Comparison ──────────────────────────────────────────┐\n";
    std::cout << "  │ Same Result: " << std::setw(38)
              << (result.same_result ? "✓ MATCH" : "✗ DIFFERENT") << " │\n";
    std::cout << "  │ Speedup: " << std::setw(34) << std::fixed << std::setprecision(2)
              << result.speedup << "x faster │\n";
    std::cout << "  │ Winner: " << std::setw(36)
              << (result.maxflow_time_ms < result.spring_time_ms ? "Max-Flow" : "Spring") << "     │\n";
    std::cout << "  └───────────────────────────────────────────────────────┘\n";
    std::cout << "\n";
}

ComparisonResult RunComparison(const Scenario& scenario) {
    ComparisonResult result;
    result.scenario_name = scenario.name;

    // Create world and setup scenario
    VoxelWorld world;
    std::vector<Vector3> damaged;
    scenario.setup(world, damaged);

    // Run Spring analyzer
    StructuralAnalyzer spring_analyzer;
    auto spring_result = spring_analyzer.Analyze(world, damaged);

    result.spring_failed = spring_result.structure_failed;
    result.spring_clusters = static_cast<int>(spring_result.failed_clusters.size());
    result.spring_time_ms = spring_result.calculation_time_ms;
    result.spring_iterations = spring_result.iterations_used;

    // Run Max-Flow analyzer
    MaxFlowStructuralAnalyzer maxflow_analyzer;
    auto maxflow_result = maxflow_analyzer.Analyze(world, damaged);

    result.maxflow_failed = maxflow_result.structure_failed;
    result.maxflow_clusters = static_cast<int>(maxflow_result.failed_clusters.size());
    result.maxflow_time_ms = maxflow_result.calculation_time_ms;
    result.maxflow_iterations = maxflow_result.iterations_used;

    // Calculate comparison metrics
    result.same_result = (result.spring_failed == result.maxflow_failed);
    if (result.maxflow_time_ms > 0) {
        result.speedup = result.spring_time_ms / result.maxflow_time_ms;
    } else {
        result.speedup = 0.0f;
    }

    return result;
}

// ============================================================================
// Test Scenarios
// ============================================================================

std::vector<Scenario> GetScenarios() {
    std::vector<Scenario> scenarios;

    // Scenario 1: Simple tower collapse
    scenarios.push_back({
        "Tower Collapse",
        "15-voxel tower with base removed",
        [](VoxelWorld& world, std::vector<Vector3>& damaged) {
            uint8_t concrete = MaterialDatabase::CONCRETE;
            float vs = world.GetVoxelSize();

            // Create tower
            for (int y = 3; y < 18; y++) {
                world.SetVoxel(Vector3(0, y * vs, 0), Voxel(concrete));
            }

            // Base removed (simulated)
            damaged.push_back(Vector3(0, 0, 0));
        }
    });

    // Scenario 2: Grounded tower (stable)
    scenarios.push_back({
        "Stable Tower",
        "20-voxel concrete tower with solid base",
        [](VoxelWorld& world, std::vector<Vector3>& damaged) {
            uint8_t concrete = MaterialDatabase::CONCRETE;
            float vs = world.GetVoxelSize();

            // Create tower with base
            for (int y = 0; y < 20; y++) {
                world.SetVoxel(Vector3(0, y * vs, 0), Voxel(concrete));
            }

            // Damage nearby (not base)
            damaged.push_back(Vector3(vs, 0, 0));
        }
    });

    // Scenario 3: Building corner column
    scenarios.push_back({
        "Building Corner Damage",
        "12x12x3 building with corner column destroyed",
        [](VoxelWorld& world, std::vector<Vector3>& damaged) {
            uint8_t brick = MaterialDatabase::BRICK;
            float vs = world.GetVoxelSize();

            // Create building floor plan
            for (int y = 0; y < 3; y++) {
                for (int x = 0; x < 12; x++) {
                    for (int z = 0; z < 12; z++) {
                        // Hollow building
                        if (x == 0 || x == 11 || z == 0 || z == 11) {
                            world.SetVoxel(Vector3(x * vs, y * vs, z * vs), Voxel(brick));
                        }
                    }
                }
            }

            // Destroy corner column
            for (int y = 0; y < 3; y++) {
                Vector3 corner(0, y * vs, 0);
                world.RemoveVoxel(corner);
                damaged.push_back(corner);
            }
        }
    });

    // Scenario 4: Bridge collapse
    scenarios.push_back({
        "Bridge Support Failure",
        "Horizontal span with support removed",
        [](VoxelWorld& world, std::vector<Vector3>& damaged) {
            uint8_t concrete = MaterialDatabase::CONCRETE;
            float vs = world.GetVoxelSize();

            // Left support
            for (int y = 0; y < 5; y++) {
                world.SetVoxel(Vector3(0, y * vs, 0), Voxel(concrete));
            }

            // Right support (removed - no support)
            // Bridge span
            for (int x = 0; x <= 10; x++) {
                world.SetVoxel(Vector3(x * vs, 5 * vs, 0), Voxel(concrete));
            }

            // Support removed
            damaged.push_back(Vector3(10 * vs, 0, 0));
        }
    });

    // Scenario 5: Large structure stress test
    scenarios.push_back({
        "Large Building (Performance Test)",
        "20x20x5 structure with damage",
        [](VoxelWorld& world, std::vector<Vector3>& damaged) {
            uint8_t brick = MaterialDatabase::BRICK;
            float vs = world.GetVoxelSize();

            // Create large building
            for (int y = 0; y < 5; y++) {
                for (int x = 0; x < 20; x++) {
                    for (int z = 0; z < 20; z++) {
                        // Walls only
                        if (x == 0 || x == 19 || z == 0 || z == 19) {
                            world.SetVoxel(Vector3(x * vs, y * vs, z * vs), Voxel(brick));
                        }
                    }
                }
            }

            // Remove one section
            for (int y = 0; y < 5; y++) {
                Vector3 pos(0, y * vs, 0);
                world.RemoveVoxel(pos);
                damaged.push_back(pos);
            }
        }
    });

    // Scenario 6: Material comparison
    scenarios.push_back({
        "Material Strength Test",
        "30-voxel wood tower (stress test)",
        [](VoxelWorld& world, std::vector<Vector3>& damaged) {
            uint8_t wood = MaterialDatabase::WOOD;
            float vs = world.GetVoxelSize();

            // Create tall wood tower
            for (int y = 0; y < 30; y++) {
                world.SetVoxel(Vector3(0, y * vs, 0), Voxel(wood));
            }

            // Damage nearby
            damaged.push_back(Vector3(vs, 0, 0));
        }
    });

    return scenarios;
}

// ============================================================================
// Main
// ============================================================================

void PrintSummary(const std::vector<ComparisonResult>& results) {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  Overall Summary                                        ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════╝\n";
    std::cout << "\n";

    int agreement_count = 0;
    float total_spring_time = 0.0f;
    float total_maxflow_time = 0.0f;
    float avg_speedup = 0.0f;

    for (const auto& result : results) {
        if (result.same_result) agreement_count++;
        total_spring_time += result.spring_time_ms;
        total_maxflow_time += result.maxflow_time_ms;
        avg_speedup += result.speedup;
    }

    avg_speedup /= results.size();

    std::cout << "  Scenarios Run:        " << results.size() << "\n";
    std::cout << "  Agreement:            " << agreement_count << " / " << results.size()
              << " (" << (100.0f * agreement_count / results.size()) << "%)\n";
    std::cout << "  Total Spring Time:    " << std::fixed << std::setprecision(2)
              << total_spring_time << " ms\n";
    std::cout << "  Total Max-Flow Time:  " << std::fixed << std::setprecision(2)
              << total_maxflow_time << " ms\n";
    std::cout << "  Average Speedup:      " << std::fixed << std::setprecision(2)
              << avg_speedup << "x\n";
    std::cout << "\n";

    std::cout << "  Advantages:\n";
    std::cout << "    Spring System:  More parameters, easier tuning\n";
    std::cout << "    Max-Flow:       Faster, deterministic, more accurate\n";
    std::cout << "\n";

    std::cout << "  Recommendation:\n";
    if (avg_speedup > 2.0f) {
        std::cout << "    ✓ Max-Flow is significantly faster - consider switching!\n";
    } else if (agreement_count == results.size()) {
        std::cout << "    ⚖ Both analyzers agree - use based on preference\n";
    } else {
        std::cout << "    ⚠ Results differ - review scenarios and tune parameters\n";
    }

    std::cout << "\n";
}

int main() {
    PrintHeader();

    std::cout << "This demo compares two structural analysis approaches:\n";
    std::cout << "  1. Spring System (Displacement-Based) - Current implementation\n";
    std::cout << "  2. Max-Flow Algorithm (Dinic's) - New implementation\n";
    std::cout << "\n";
    std::cout << "Running " << GetScenarios().size() << " test scenarios...\n";
    std::cout << "\n";

    std::vector<ComparisonResult> results;

    // Run each scenario
    for (const auto& scenario : GetScenarios()) {
        PrintScenarioHeader(scenario.name, scenario.description);

        ComparisonResult result = RunComparison(scenario);
        results.push_back(result);

        PrintComparison(result);

        std::cout << "  → Done\n";
        std::cout << "\n";
    }

    // Print summary
    PrintSummary(results);

    return 0;
}
