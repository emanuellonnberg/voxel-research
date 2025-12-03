/**
 * Performance Benchmark Suite for StructuralAnalyzer
 * Day 20: Comprehensive benchmarking across various scenarios
 *
 * This benchmark measures:
 * - Analysis time vs structure size
 * - Impact of different parameters
 * - Cache performance
 * - Optimization effectiveness (Day 16 features)
 */

#include "StructuralAnalyzer.h"
#include "ParameterTuning.h"
#include "VoxelWorld.h"
#include "VoxelUtils.h"
#include "Material.h"
#include <chrono>
#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>

using Clock = std::chrono::high_resolution_clock;

// Benchmark result structure
struct BenchmarkResult {
    std::string scenario_name;
    int structure_size;
    int nodes_analyzed;
    float avg_time_ms;
    float min_time_ms;
    float max_time_ms;
    float std_dev_ms;
    int avg_iterations;
    bool avg_converged;
    float cache_hit_rate;
};

// Build test structures
void BuildTower(VoxelWorld& world, int height, uint8_t material_id) {
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < height; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(material_id));
    }
}

void BuildWall(VoxelWorld& world, int width, int height, uint8_t material_id) {
    float voxel_size = world.GetVoxelSize();
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, 0), Voxel(material_id));
        }
    }
}

void BuildCube(VoxelWorld& world, int size, uint8_t material_id) {
    float voxel_size = world.GetVoxelSize();
    for (int x = 0; x < size; x++) {
        for (int y = 0; y < size; y++) {
            for (int z = 0; z < size; z++) {
                world.SetVoxel(
                    Vector3(x * voxel_size, y * voxel_size, z * voxel_size),
                    Voxel(material_id)
                );
            }
        }
    }
}

void BuildBridge(VoxelWorld& world, int length, uint8_t material_id) {
    float voxel_size = world.GetVoxelSize();
    // Two pillars
    for (int y = 0; y < 10; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(material_id));
        world.SetVoxel(Vector3(length * voxel_size, y * voxel_size, 0), Voxel(material_id));
    }
    // Span
    for (int x = 0; x <= length; x++) {
        world.SetVoxel(Vector3(x * voxel_size, 10 * voxel_size, 0), Voxel(material_id));
    }
}

// Run benchmark for a scenario
BenchmarkResult RunBenchmark(
    const std::string& name,
    VoxelWorld& world,
    const std::vector<Vector3>& damage,
    StructuralAnalyzer& analyzer,
    int iterations = 10)
{
    BenchmarkResult result;
    result.scenario_name = name;
    result.structure_size = static_cast<int>(world.GetAllVoxelPositions().size());

    std::vector<float> times;
    std::vector<int> iteration_counts;
    std::vector<bool> converged_list;

    for (int i = 0; i < iterations; i++) {
        analyzer.Clear();
        analyzer.ClearMassCache();

        auto start = Clock::now();
        auto analysis_result = analyzer.Analyze(world, damage);
        auto elapsed = std::chrono::duration<float, std::milli>(Clock::now() - start).count();

        times.push_back(elapsed);
        iteration_counts.push_back(analysis_result.iterations_used);
        converged_list.push_back(analysis_result.converged);

        if (i == 0) {
            result.nodes_analyzed = analysis_result.nodes_analyzed;

            int hits, misses;
            analyzer.GetCacheStats(hits, misses);
            result.cache_hit_rate = (hits + misses > 0) ?
                (float)hits / (hits + misses) : 0.0f;
        }
    }

    // Calculate statistics
    result.min_time_ms = *std::min_element(times.begin(), times.end());
    result.max_time_ms = *std::max_element(times.begin(), times.end());

    float sum = 0.0f;
    for (float t : times) sum += t;
    result.avg_time_ms = sum / times.size();

    float sum_iter = 0.0f;
    for (int iter : iteration_counts) sum_iter += iter;
    result.avg_iterations = static_cast<int>(sum_iter / iteration_counts.size());

    int converged_count = 0;
    for (bool c : converged_list) if (c) converged_count++;
    result.avg_converged = (converged_count > iterations / 2);

    // Calculate standard deviation
    float variance = 0.0f;
    for (float t : times) {
        float diff = t - result.avg_time_ms;
        variance += diff * diff;
    }
    result.std_dev_ms = std::sqrt(variance / times.size());

    return result;
}

// Print benchmark results
void PrintBenchmarkResult(const BenchmarkResult& result) {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "\n" << result.scenario_name << ":\n";
    std::cout << "  Structure: " << result.structure_size << " voxels\n";
    std::cout << "  Nodes analyzed: " << result.nodes_analyzed << "\n";
    std::cout << "  Time: " << result.avg_time_ms << " ± " << result.std_dev_ms << " ms\n";
    std::cout << "  Range: [" << result.min_time_ms << ", " << result.max_time_ms << "] ms\n";
    std::cout << "  Iterations: " << result.avg_iterations << "\n";
    std::cout << "  Converged: " << (result.avg_converged ? "YES" : "NO") << "\n";
    std::cout << "  Cache hit rate: " << (result.cache_hit_rate * 100.0f) << "%\n";
}

// Save results to CSV
void SaveResultsCSV(const std::vector<BenchmarkResult>& results, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename << " for writing\n";
        return;
    }

    file << "Scenario,Structure Size,Nodes Analyzed,Avg Time (ms),Std Dev (ms),Min Time (ms),Max Time (ms),Avg Iterations,Converged,Cache Hit Rate\n";

    for (const auto& result : results) {
        file << result.scenario_name << ","
             << result.structure_size << ","
             << result.nodes_analyzed << ","
             << result.avg_time_ms << ","
             << result.std_dev_ms << ","
             << result.min_time_ms << ","
             << result.max_time_ms << ","
             << result.avg_iterations << ","
             << (result.avg_converged ? 1 : 0) << ","
             << result.cache_hit_rate << "\n";
    }

    file.close();
    std::cout << "\nResults saved to " << filename << "\n";
}

int main() {
    std::cout << "==================================================\n";
    std::cout << "  StructuralAnalyzer Performance Benchmark Suite\n";
    std::cout << "  Day 20: Week 4 Final Integration Testing\n";
    std::cout << "==================================================\n";

    std::vector<BenchmarkResult> all_results;

    // Setup analyzer with optimal parameters
    StructuralAnalyzer analyzer;
    analyzer.LoadParameters("../config/optimal_params.ini");

    float voxel_size = 0.05f;
    uint8_t brick_id = MaterialDatabase::BRICK;

    // ==================================================
    // Benchmark 1: Small Tower (10 voxels)
    // ==================================================
    {
        VoxelWorld world;
        BuildTower(world, 10, brick_id);
        Vector3 damage_pos(0, 5 * voxel_size, 0);
        world.RemoveVoxel(damage_pos);

        auto result = RunBenchmark("Small Tower (10 voxels)", world, {damage_pos}, analyzer, 20);
        PrintBenchmarkResult(result);
        all_results.push_back(result);
    }

    // ==================================================
    // Benchmark 2: Medium Tower (20 voxels)
    // ==================================================
    {
        VoxelWorld world;
        BuildTower(world, 20, brick_id);
        Vector3 damage_pos(0, 10 * voxel_size, 0);
        world.RemoveVoxel(damage_pos);

        auto result = RunBenchmark("Medium Tower (20 voxels)", world, {damage_pos}, analyzer, 20);
        PrintBenchmarkResult(result);
        all_results.push_back(result);
    }

    // ==================================================
    // Benchmark 3: Large Tower (50 voxels)
    // ==================================================
    {
        VoxelWorld world;
        BuildTower(world, 50, brick_id);
        Vector3 damage_pos(0, 25 * voxel_size, 0);
        world.RemoveVoxel(damage_pos);

        auto result = RunBenchmark("Large Tower (50 voxels)", world, {damage_pos}, analyzer, 10);
        PrintBenchmarkResult(result);
        all_results.push_back(result);
    }

    // ==================================================
    // Benchmark 4: Small Wall (5x5 = 25 voxels)
    // ==================================================
    {
        VoxelWorld world;
        BuildWall(world, 5, 5, brick_id);
        Vector3 damage_pos(2 * voxel_size, 2 * voxel_size, 0);
        world.RemoveVoxel(damage_pos);

        auto result = RunBenchmark("Small Wall (5x5)", world, {damage_pos}, analyzer, 20);
        PrintBenchmarkResult(result);
        all_results.push_back(result);
    }

    // ==================================================
    // Benchmark 5: Medium Wall (10x10 = 100 voxels)
    // ==================================================
    {
        VoxelWorld world;
        BuildWall(world, 10, 10, brick_id);
        Vector3 damage_pos(5 * voxel_size, 5 * voxel_size, 0);
        world.RemoveVoxel(damage_pos);

        auto result = RunBenchmark("Medium Wall (10x10)", world, {damage_pos}, analyzer, 10);
        PrintBenchmarkResult(result);
        all_results.push_back(result);
    }

    // ==================================================
    // Benchmark 6: Large Wall (20x10 = 200 voxels)
    // ==================================================
    {
        VoxelWorld world;
        BuildWall(world, 20, 10, brick_id);
        Vector3 damage_pos(10 * voxel_size, 5 * voxel_size, 0);
        world.RemoveVoxel(damage_pos);

        auto result = RunBenchmark("Large Wall (20x10)", world, {damage_pos}, analyzer, 10);
        PrintBenchmarkResult(result);
        all_results.push_back(result);
    }

    // ==================================================
    // Benchmark 7: Small Cube (5x5x5 = 125 voxels)
    // ==================================================
    {
        VoxelWorld world;
        BuildCube(world, 5, brick_id);
        Vector3 damage_pos(2 * voxel_size, 2 * voxel_size, 2 * voxel_size);
        world.RemoveVoxel(damage_pos);

        auto result = RunBenchmark("Small Cube (5x5x5)", world, {damage_pos}, analyzer, 10);
        PrintBenchmarkResult(result);
        all_results.push_back(result);
    }

    // ==================================================
    // Benchmark 8: Medium Cube (10x10x10 = 1000 voxels)
    // ==================================================
    {
        VoxelWorld world;
        BuildCube(world, 10, brick_id);
        Vector3 damage_pos(5 * voxel_size, 5 * voxel_size, 5 * voxel_size);
        world.RemoveVoxel(damage_pos);

        auto result = RunBenchmark("Medium Cube (10x10x10)", world, {damage_pos}, analyzer, 5);
        PrintBenchmarkResult(result);
        all_results.push_back(result);
    }

    // ==================================================
    // Benchmark 9: Bridge (20 span)
    // ==================================================
    {
        VoxelWorld world;
        BuildBridge(world, 20, brick_id);
        Vector3 damage_pos(10 * voxel_size, 10 * voxel_size, 0);
        world.RemoveVoxel(damage_pos);

        auto result = RunBenchmark("Bridge (20 span)", world, {damage_pos}, analyzer, 10);
        PrintBenchmarkResult(result);
        all_results.push_back(result);
    }

    // ==================================================
    // Benchmark 10: Optimization Impact Test
    // ==================================================
    std::cout << "\n==================================================\n";
    std::cout << "  Testing Day 16 Optimizations Impact\n";
    std::cout << "==================================================\n";

    {
        VoxelWorld world;
        BuildWall(world, 15, 10, brick_id);
        Vector3 damage_pos(7 * voxel_size, 5 * voxel_size, 0);
        world.RemoveVoxel(damage_pos);
        std::vector<Vector3> damage = {damage_pos};

        // Test with all optimizations
        analyzer.params.use_parallel_mass_calc = true;
        analyzer.params.influence_radius = 5.0f;
        auto result_optimized = RunBenchmark(
            "Optimized (parallel + radius=5)",
            world, damage, analyzer, 10
        );
        PrintBenchmarkResult(result_optimized);
        all_results.push_back(result_optimized);

        // Test without parallel
        analyzer.params.use_parallel_mass_calc = false;
        auto result_no_parallel = RunBenchmark(
            "No Parallel (sequential only)",
            world, damage, analyzer, 10
        );
        PrintBenchmarkResult(result_no_parallel);
        all_results.push_back(result_no_parallel);

        std::cout << "\nParallel speedup: "
                  << (result_no_parallel.avg_time_ms / result_optimized.avg_time_ms)
                  << "x\n";

        // Test with larger radius
        analyzer.params.use_parallel_mass_calc = true;
        analyzer.params.influence_radius = 20.0f;
        auto result_large_radius = RunBenchmark(
            "Large Radius (radius=20)",
            world, damage, analyzer, 10
        );
        PrintBenchmarkResult(result_large_radius);
        all_results.push_back(result_large_radius);

        std::cout << "\nInfluence radius impact: "
                  << (result_large_radius.avg_time_ms / result_optimized.avg_time_ms)
                  << "x slower with 4x radius\n";
    }

    // ==================================================
    // Summary
    // ==================================================
    std::cout << "\n==================================================\n";
    std::cout << "  Benchmark Summary\n";
    std::cout << "==================================================\n";

    float total_avg_time = 0.0f;
    for (const auto& result : all_results) {
        total_avg_time += result.avg_time_ms;
    }

    std::cout << "Total scenarios tested: " << all_results.size() << "\n";
    std::cout << "Average analysis time: "
              << (total_avg_time / all_results.size()) << " ms\n";

    // Find fastest and slowest
    auto fastest = std::min_element(all_results.begin(), all_results.end(),
        [](const BenchmarkResult& a, const BenchmarkResult& b) {
            return a.avg_time_ms < b.avg_time_ms;
        });

    auto slowest = std::max_element(all_results.begin(), all_results.end(),
        [](const BenchmarkResult& a, const BenchmarkResult& b) {
            return a.avg_time_ms < b.avg_time_ms;
        });

    std::cout << "\nFastest: " << fastest->scenario_name
              << " (" << fastest->avg_time_ms << " ms)\n";
    std::cout << "Slowest: " << slowest->scenario_name
              << " (" << slowest->avg_time_ms << " ms)\n";
    std::cout << "Performance range: "
              << (slowest->avg_time_ms / fastest->avg_time_ms) << "x\n";

    // Save results
    SaveResultsCSV(all_results, "benchmark_results.csv");

    std::cout << "\n==================================================\n";
    std::cout << "  Benchmark Complete!\n";
    std::cout << "==================================================\n";

    return 0;
}
