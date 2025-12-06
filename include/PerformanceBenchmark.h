/**
 * PerformanceBenchmark.h
 *
 * Week 13 Day 45: Comprehensive Performance Benchmark Suite
 *
 * Tests all parallelized systems and generates performance reports.
 */

#pragma once

#include "VoxelWorld.h"
#include "StructuralAnalyzer.h"
#include "VoxelPhysicsIntegration.h"
#include "IPhysicsEngine.h"
#include "JobSystem.h"
#include <string>
#include <chrono>
#include <vector>

struct BenchmarkResult {
    std::string test_name;
    float serial_time_ms;
    float parallel_time_ms;
    float speedup;
    int num_voxels;
    int num_threads;
    bool target_met;
    float target_speedup;
};

class PerformanceBenchmark {
public:
    PerformanceBenchmark();
    ~PerformanceBenchmark();

    // Run all benchmarks
    void RunAll();

    // Individual benchmarks
    BenchmarkResult BenchmarkStructuralAnalysis();
    BenchmarkResult BenchmarkDebrisSpawning();
    BenchmarkResult BenchmarkMultipleCollapses();
    BenchmarkResult BenchmarkFullDestruction();

    // Results
    void PrintResults() const;
    void SaveReport(const std::string& filename) const;
    const std::vector<BenchmarkResult>& GetResults() const { return results; }

private:
    // Helper: Build test structures
    void BuildTower(VoxelWorld& world, const Vector3& base, int height);
    void BuildWall(VoxelWorld& world, const Vector3& base, int width, int height);
    void BuildLargeStructure(VoxelWorld& world, int size_x, int size_y, int size_z);

    // Helper: Damage structures
    std::vector<Vector3> DamageBase(VoxelWorld& world, const Vector3& base, int radius);
    std::vector<Vector3> DamageWallCenter(VoxelWorld& world, const Vector3& center, int radius);

    // Timing utilities
    template<typename Func>
    float TimeFunction(Func func);

    std::vector<BenchmarkResult> results;
};
