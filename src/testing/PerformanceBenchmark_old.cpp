/**
 * PerformanceBenchmark.cpp
 *
 * Week 13 Day 45: Performance Benchmark Implementation
 */

#include "PerformanceBenchmark.h"
#include "Material.h"

#ifdef USE_BULLET
#include "BulletEngine.h"
#else
#include "MockPhysicsEngine.h"
#endif

#include <iostream>
#include <fstream>
#include <iomanip>

PerformanceBenchmark::PerformanceBenchmark() {
}

PerformanceBenchmark::~PerformanceBenchmark() {
}

void PerformanceBenchmark::RunAll() {
    std::cout << "========================================\n";
    std::cout << "  Week 13 Performance Benchmark Suite\n";
    std::cout << "========================================\n\n";

    results.clear();

    // Run each benchmark
    std::cout << "[1/4] Benchmarking Structural Analysis...\n";
    results.push_back(BenchmarkStructuralAnalysis());

    std::cout << "[2/4] Benchmarking Debris Spawning...\n";
    results.push_back(BenchmarkDebrisSpawning());

    std::cout << "[3/4] Benchmarking Multiple Collapses...\n";
    results.push_back(BenchmarkMultipleCollapses());

    std::cout << "[4/4] Benchmarking Full Destruction...\n";
    results.push_back(BenchmarkFullDestruction());

    std::cout << "\n";
    PrintResults();
}

BenchmarkResult PerformanceBenchmark::BenchmarkStructuralAnalysis() {
    BenchmarkResult result;
    result.test_name = "Structural Analysis";
    result.target_speedup = 2.0f;  // Target: 2-3x speedup
    result.num_threads = g_JobSystem ? g_JobSystem->GetWorkerCount() : 1;

    // Build large structure (20x20x10 = 4000 voxels)
    VoxelWorld world;
    BuildLargeStructure(world, 20, 10, 20);
    result.num_voxels = world.GetVoxelCount();

    // Damage entire base layer (200 voxels)
    std::vector<Vector3> damaged;
    float voxel_size = world.GetVoxelSize();
    for (int x = 0; x < 20; x++) {
        for (int z = 0; z < 20; z++) {
            Vector3 pos(x * voxel_size, 0, z * voxel_size);
            world.RemoveVoxel(pos);
            damaged.push_back(pos);
        }
    }

    // Serial benchmark (disable JobSystem)
    bool job_system_was_enabled = (g_JobSystem != nullptr && g_JobSystem->IsRunning());
    if (job_system_was_enabled) {
        g_JobSystem->Shutdown();
    }

    StructuralAnalyzer serial_analyzer;
    result.serial_time_ms = TimeFunction([&]() {
        serial_analyzer.Analyze(world, damaged);
    });

    // Parallel benchmark (enable JobSystem)
    if (job_system_was_enabled) {
        g_JobSystem->Initialize(result.num_threads);
    }

    StructuralAnalyzer parallel_analyzer;
    result.parallel_time_ms = TimeFunction([&]() {
        parallel_analyzer.Analyze(world, damaged);
    });

    result.speedup = result.serial_time_ms / result.parallel_time_ms;
    result.target_met = (result.speedup >= result.target_speedup);

    return result;
}

BenchmarkResult PerformanceBenchmark::BenchmarkDebrisSpawning() {
    BenchmarkResult result;
    result.test_name = "Debris Spawning";
    result.target_speedup = 1.5f;  // Target: 1.5-2x speedup
    result.num_threads = g_JobSystem ? g_JobSystem->GetWorkerCount() : 1;

    // Build structure and get failed clusters
    VoxelWorld world;
    BuildLargeStructure(world, 15, 10, 15);
    result.num_voxels = world.GetVoxelCount();

    auto damaged = DamageBase(world, Vector3(0, 0, 0), 7);

    StructuralAnalyzer analyzer;
    auto analysis_result = analyzer.Analyze(world, damaged);

    // Need physics engine
#ifdef USE_BULLET
    IPhysicsEngine* physics = new BulletEngine();
#else
    IPhysicsEngine* physics = new MockPhysicsEngine();
#endif
    physics->Initialize();

    // Serial benchmark
    bool job_system_was_enabled = (g_JobSystem != nullptr && g_JobSystem->IsRunning());
    if (job_system_was_enabled) {
        g_JobSystem->Shutdown();
    }

    VoxelPhysicsIntegration serial_integration(physics, &world);
    result.serial_time_ms = TimeFunction([&]() {
        serial_integration.SpawnDebris(analysis_result.failed_clusters);
    });
    serial_integration.ClearDebris();

    // Parallel benchmark
    if (job_system_was_enabled) {
        g_JobSystem->Initialize(result.num_threads);
    }

    VoxelPhysicsIntegration parallel_integration(physics, &world);
    result.parallel_time_ms = TimeFunction([&]() {
        parallel_integration.SpawnDebris(analysis_result.failed_clusters);
    });

    result.speedup = result.serial_time_ms / result.parallel_time_ms;
    result.target_met = (result.speedup >= result.target_speedup);

    physics->Shutdown();
    delete physics;

    return result;
}

BenchmarkResult PerformanceBenchmark::BenchmarkMultipleCollapses() {
    BenchmarkResult result;
    result.test_name = "Multiple Simultaneous Collapses";
    result.target_speedup = 3.0f;  // Target: 3-4x speedup
    result.num_threads = g_JobSystem ? g_JobSystem->GetWorkerCount() : 1;

    // Build 4 separate towers
    VoxelWorld world;
    float voxel_size = world.GetVoxelSize();

    BuildTower(world, Vector3(0, 0, 0), 15);
    BuildTower(world, Vector3(20 * voxel_size, 0, 0), 15);
    BuildTower(world, Vector3(0, 0, 20 * voxel_size), 15);
    BuildTower(world, Vector3(20 * voxel_size, 0, 20 * voxel_size), 15);

    result.num_voxels = world.GetVoxelCount();

    // Damage all 4 bases simultaneously
    std::vector<Vector3> damaged;
    damaged.push_back(Vector3(0, 0, 0));
    damaged.push_back(Vector3(20 * voxel_size, 0, 0));
    damaged.push_back(Vector3(0, 0, 20 * voxel_size));
    damaged.push_back(Vector3(20 * voxel_size, 0, 20 * voxel_size));

    for (const auto& pos : damaged) {
        world.RemoveVoxel(pos);
    }

    // Serial benchmark
    bool job_system_was_enabled = (g_JobSystem != nullptr && g_JobSystem->IsRunning());
    if (job_system_was_enabled) {
        g_JobSystem->Shutdown();
    }

    StructuralAnalyzer serial_analyzer;
    result.serial_time_ms = TimeFunction([&]() {
        serial_analyzer.Analyze(world, damaged);
    });

    // Parallel benchmark
    if (job_system_was_enabled) {
        g_JobSystem->Initialize(result.num_threads);
    }

    StructuralAnalyzer parallel_analyzer;
    result.parallel_time_ms = TimeFunction([&]() {
        parallel_analyzer.Analyze(world, damaged);
    });

    result.speedup = result.serial_time_ms / result.parallel_time_ms;
    result.target_met = (result.speedup >= result.target_speedup);

    return result;
}

BenchmarkResult PerformanceBenchmark::BenchmarkFullDestruction() {
    BenchmarkResult result;
    result.test_name = "Full Destruction (End-to-End)";
    result.target_speedup = 0.0f;  // No comparison, just absolute time
    result.num_threads = g_JobSystem ? g_JobSystem->GetWorkerCount() : 1;

    // Ensure JobSystem is running
    bool job_system_was_enabled = (g_JobSystem != nullptr && g_JobSystem->IsRunning());
    if (!job_system_was_enabled && g_JobSystem) {
        g_JobSystem->Initialize(result.num_threads);
    }

    // Build massive structure (30x20x30 = 18000 voxels)
    VoxelWorld world;
    BuildLargeStructure(world, 30, 20, 30);
    result.num_voxels = world.GetVoxelCount();

    // Damage entire base (900 voxels)
    std::vector<Vector3> damaged;
    float voxel_size = world.GetVoxelSize();
    for (int x = 0; x < 30; x++) {
        for (int z = 0; z < 30; z++) {
            Vector3 pos(x * voxel_size, 0, z * voxel_size);
            world.RemoveVoxel(pos);
            damaged.push_back(pos);
        }
    }

    // Full pipeline: Analyze + Spawn Debris + Simulate
#ifdef USE_BULLET
    IPhysicsEngine* physics = new BulletEngine();
#else
    IPhysicsEngine* physics = new MockPhysicsEngine();
#endif
    physics->Initialize();

    result.parallel_time_ms = TimeFunction([&]() {
        // Step 1: Structural Analysis
        StructuralAnalyzer analyzer;
        auto analysis_result = analyzer.Analyze(world, damaged);

        // Step 2: Spawn Debris
        VoxelPhysicsIntegration integration(physics, &world);
        integration.SpawnDebris(analysis_result.failed_clusters);

        // Step 3: Simulate physics for 2 seconds
        for (int i = 0; i < 120; i++) {
            integration.Step(1.0f / 60.0f);
        }
    });

    result.serial_time_ms = 0.0f;  // Not applicable
    result.speedup = 0.0f;
    result.target_met = (result.parallel_time_ms < 5000.0f);  // Target: < 5 seconds

    physics->Shutdown();
    delete physics;

    return result;
}

// ===== Helper Functions =====

void PerformanceBenchmark::BuildTower(VoxelWorld& world, const Vector3& base, int height) {
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < height; y++) {
        Vector3 pos = base + Vector3(0, y * voxel_size, 0);
        world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
    }
}

void PerformanceBenchmark::BuildWall(VoxelWorld& world, const Vector3& base, int width, int height) {
    float voxel_size = world.GetVoxelSize();
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            Vector3 pos = base + Vector3(x * voxel_size, y * voxel_size, 0);
            world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
        }
    }
}

void PerformanceBenchmark::BuildLargeStructure(VoxelWorld& world, int size_x, int size_y, int size_z) {
    float voxel_size = world.GetVoxelSize();
    for (int x = 0; x < size_x; x++) {
        for (int y = 0; y < size_y; y++) {
            for (int z = 0; z < size_z; z++) {
                Vector3 pos(x * voxel_size, y * voxel_size, z * voxel_size);
                world.SetVoxel(pos, Voxel(MaterialDatabase::CONCRETE));
            }
        }
    }
}

std::vector<Vector3> PerformanceBenchmark::DamageBase(VoxelWorld& world, const Vector3& base, int radius) {
    std::vector<Vector3> damaged;
    float voxel_size = world.GetVoxelSize();

    for (int x = -radius; x <= radius; x++) {
        for (int z = -radius; z <= radius; z++) {
            Vector3 pos = base + Vector3(x * voxel_size, 0, z * voxel_size);
            if (world.HasVoxel(pos)) {
                world.RemoveVoxel(pos);
                damaged.push_back(pos);
            }
        }
    }
    return damaged;
}

std::vector<Vector3> PerformanceBenchmark::DamageWallCenter(VoxelWorld& world, const Vector3& center, int radius) {
    std::vector<Vector3> damaged;
    float voxel_size = world.GetVoxelSize();

    for (int x = -radius; x <= radius; x++) {
        for (int y = -radius; y <= radius; y++) {
            Vector3 pos = center + Vector3(x * voxel_size, y * voxel_size, 0);
            if (world.HasVoxel(pos)) {
                world.RemoveVoxel(pos);
                damaged.push_back(pos);
            }
        }
    }
    return damaged;
}

template<typename Func>
float PerformanceBenchmark::TimeFunction(Func func) {
    auto start = std::chrono::high_resolution_clock::now();
    func();
    auto end = std::chrono::high_resolution_clock::now();

    return std::chrono::duration<float, std::milli>(end - start).count();
}

void PerformanceBenchmark::PrintResults() const {
    std::cout << "========================================\n";
    std::cout << "         BENCHMARK RESULTS\n";
    std::cout << "========================================\n\n";

    for (const auto& result : results) {
        std::cout << "Test: " << result.test_name << "\n";
        std::cout << "  Voxels:        " << result.num_voxels << "\n";
        std::cout << "  Threads:       " << result.num_threads << "\n";

        if (result.serial_time_ms > 0.0f) {
            std::cout << "  Serial Time:   " << std::fixed << std::setprecision(2)
                      << result.serial_time_ms << " ms\n";
            std::cout << "  Parallel Time: " << result.parallel_time_ms << " ms\n";
            std::cout << "  Speedup:       " << std::setprecision(2) << result.speedup << "x\n";
            std::cout << "  Target:        " << result.target_speedup << "x\n";
        } else {
            std::cout << "  Time:          " << std::fixed << std::setprecision(2)
                      << result.parallel_time_ms << " ms ("
                      << (result.parallel_time_ms / 1000.0f) << " sec)\n";
            std::cout << "  Target:        < 5000 ms (5 sec)\n";
        }

        std::cout << "  Status:        " << (result.target_met ? "✓ PASS" : "✗ FAIL") << "\n\n";
    }

    // Summary
    int passed = 0;
    for (const auto& result : results) {
        if (result.target_met) passed++;
    }

    std::cout << "========================================\n";
    std::cout << "Summary: " << passed << "/" << results.size() << " benchmarks met targets\n";
    std::cout << "========================================\n";
}

void PerformanceBenchmark::SaveReport(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << "\n";
        return;
    }

    file << "========================================\n";
    file << "  Week 13 Performance Benchmark Report\n";
    file << "========================================\n\n";

    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    file << "Date: " << std::ctime(&now_time) << "\n";

    for (const auto& result : results) {
        file << "----------------------------------------\n";
        file << "Test: " << result.test_name << "\n";
        file << "----------------------------------------\n";
        file << "Voxels:        " << result.num_voxels << "\n";
        file << "Threads:       " << result.num_threads << "\n";

        if (result.serial_time_ms > 0.0f) {
            file << "Serial Time:   " << std::fixed << std::setprecision(2)
                 << result.serial_time_ms << " ms\n";
            file << "Parallel Time: " << result.parallel_time_ms << " ms\n";
            file << "Speedup:       " << std::setprecision(2) << result.speedup << "x\n";
            file << "Target:        " << result.target_speedup << "x\n";
        } else {
            file << "Time:          " << std::fixed << std::setprecision(2)
                 << result.parallel_time_ms << " ms ("
                 << (result.parallel_time_ms / 1000.0f) << " sec)\n";
            file << "Target:        < 5000 ms (5 sec)\n";
        }

        file << "Status:        " << (result.target_met ? "PASS" : "FAIL") << "\n\n";
    }

    // Summary
    int passed = 0;
    for (const auto& result : results) {
        if (result.target_met) passed++;
    }

    file << "========================================\n";
    file << "Summary: " << passed << "/" << results.size() << " benchmarks met targets\n";
    file << "========================================\n";

    file.close();
    std::cout << "Report saved to: " << filename << "\n";
}
