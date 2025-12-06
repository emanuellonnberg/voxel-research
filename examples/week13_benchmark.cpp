/**
 * week13_benchmark.cpp
 *
 * Week 13 Day 45: Multi-Threading Benchmark Executable
 *
 * Runs comprehensive parallel vs serial benchmarks for Week 13 optimization work.
 */

#include "PerformanceBenchmark.h"
#include "JobSystem.h"
#include <iostream>

int main(int argc, char** argv) {
    std::cout << "╔══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  Voxel Destruction Performance Profiling                ║\n";
    std::cout << "║  Week 13 Day 45: Multi-Threading Benchmarks              ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════╝\n\n";

    // Initialize JobSystem with hardware concurrency
    int num_threads = 4;  // Default to 4 threads
    if (argc > 1) {
        num_threads = std::atoi(argv[1]);
        if (num_threads < 1 || num_threads > 16) {
            std::cerr << "Invalid thread count. Using default (4).\n";
            num_threads = 4;
        }
    }

    std::cout << "Initializing JobSystem with " << num_threads << " threads...\n\n";
    g_JobSystem = new JobSystem();
    g_JobSystem->Initialize(num_threads);

    // Create benchmark suite
    PerformanceBenchmark benchmark;

    // Run all benchmarks
    benchmark.RunAll();

    // Save report
    benchmark.SaveReport("performance_week13.txt");

    std::cout << "\n✓ All Week 13 benchmarks complete!\n";
    std::cout << "  Report saved to: performance_week13.txt\n\n";

    // Cleanup
    g_JobSystem->Shutdown();
    delete g_JobSystem;
    g_JobSystem = nullptr;

    return 0;
}
