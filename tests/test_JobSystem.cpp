/**
 * JobSystem Tests
 * Week 13 Day 41: Multi-Threading Foundation
 */

#include <gtest/gtest.h>
#include "JobSystem.h"
#include <chrono>
#include <vector>
#include <atomic>
#include <algorithm>

using Clock = std::chrono::high_resolution_clock;

float GetElapsedMS(Clock::time_point start) {
    auto end = Clock::now();
    return std::chrono::duration<float, std::milli>(end - start).count();
}

class JobSystemTest : public ::testing::Test {
protected:
    void SetUp() override {
        jobs.Initialize(4);  // Use 4 threads for predictable testing
    }

    void TearDown() override {
        jobs.Shutdown();
    }

    JobSystem jobs;
};

// Test: Basic job submission and waiting
TEST_F(JobSystemTest, BasicJobSubmission) {
    std::atomic<int> counter(0);

    auto handle = jobs.Submit([&counter]() {
        counter++;
    });

    jobs.Wait(handle);

    EXPECT_EQ(counter.load(), 1);
}

// Test: Multiple jobs
TEST_F(JobSystemTest, MultipleJobs) {
    std::atomic<int> counter(0);

    std::vector<JobHandle> handles;
    for (int i = 0; i < 10; i++) {
        handles.push_back(jobs.Submit([&counter]() {
            counter++;
        }));
    }

    // Wait for all
    for (auto& handle : handles) {
        jobs.Wait(handle);
    }

    EXPECT_EQ(counter.load(), 10);
}

// Test: ParallelFor basic functionality
TEST_F(JobSystemTest, ParallelForBasic) {
    std::vector<int> data(1000, 0);

    jobs.ParallelFor(static_cast<int>(data.size()), [&](int i) {
        data[i] = i * 2;
    });

    // Verify all elements were processed
    for (size_t i = 0; i < data.size(); i++) {
        EXPECT_EQ(data[i], static_cast<int>(i * 2));
    }
}

// Test: ParallelFor with large dataset
TEST_F(JobSystemTest, ParallelForLarge) {
    std::vector<int> data(10000, 0);

    auto start = Clock::now();

    jobs.ParallelFor(static_cast<int>(data.size()), [&](int i) {
        data[i] = i * 2;
    });

    float elapsed = GetElapsedMS(start);

    // Verify correctness
    for (size_t i = 0; i < data.size(); i++) {
        EXPECT_EQ(data[i], static_cast<int>(i * 2));
    }

    std::cout << "ParallelFor 10K items took: " << elapsed << "ms\n";
}

// Test: Thread safety with atomic counter
TEST_F(JobSystemTest, ThreadSafety) {
    std::atomic<int> counter(0);
    const int num_increments = 10000;

    jobs.ParallelFor(num_increments, [&](int) {
        counter++;
    });

    EXPECT_EQ(counter.load(), num_increments);
}

// Test: Performance scaling with core count
TEST_F(JobSystemTest, PerformanceScaling) {
    // Simulate expensive work
    auto expensive_work = [](int i) {
        volatile int sum = 0;
        for (int j = 0; j < 1000; j++) {
            sum += i * j;
        }
        return sum;
    };

    const int work_count = 10000;

    // Serial execution (baseline)
    auto serial_start = Clock::now();
    for (int i = 0; i < work_count; i++) {
        expensive_work(i);
    }
    float serial_time = GetElapsedMS(serial_start);

    // Parallel execution
    auto parallel_start = Clock::now();
    jobs.ParallelFor(work_count, expensive_work);
    float parallel_time = GetElapsedMS(parallel_start);

    float speedup = serial_time / parallel_time;

    std::cout << "\nPerformance Scaling Test:\n";
    std::cout << "  Serial:   " << serial_time << "ms\n";
    std::cout << "  Parallel: " << parallel_time << "ms (4 threads)\n";
    std::cout << "  Speedup:  " << speedup << "x\n";

    // Expect at least 2x speedup with 4 threads (conservative)
    EXPECT_GT(speedup, 2.0f) << "Expected speedup > 2x with 4 threads";
}

// Test: Empty ParallelFor
TEST_F(JobSystemTest, ParallelForEmpty) {
    // Should not crash with zero iterations
    jobs.ParallelFor(0, [](int) {
        FAIL() << "Should not execute with count=0";
    });
}

// Test: Single iteration ParallelFor
TEST_F(JobSystemTest, ParallelForSingle) {
    std::atomic<int> counter(0);

    jobs.ParallelFor(1, [&](int i) {
        EXPECT_EQ(i, 0);
        counter++;
    });

    EXPECT_EQ(counter.load(), 1);
}

// Test: Nested jobs (not recommended, but should work)
TEST_F(JobSystemTest, NestedJobs) {
    std::atomic<int> counter(0);

    auto outer_handle = jobs.Submit([&]() {
        counter += 10;

        // Submit inner job
        auto inner_handle = jobs.Submit([&]() {
            counter += 5;
        });

        jobs.Wait(inner_handle);
    });

    jobs.Wait(outer_handle);

    EXPECT_EQ(counter.load(), 15);
}

// Test: Job system state
TEST_F(JobSystemTest, SystemState) {
    EXPECT_TRUE(jobs.IsRunning());
    EXPECT_EQ(jobs.GetWorkerCount(), 4);

    jobs.Shutdown();

    EXPECT_FALSE(jobs.IsRunning());
}

// Test: Reinitialization
TEST_F(JobSystemTest, Reinitialization) {
    jobs.Shutdown();
    EXPECT_FALSE(jobs.IsRunning());

    jobs.Initialize(2);
    EXPECT_TRUE(jobs.IsRunning());
    EXPECT_EQ(jobs.GetWorkerCount(), 2);

    std::atomic<int> counter(0);
    jobs.ParallelFor(100, [&](int) { counter++; });

    EXPECT_EQ(counter.load(), 100);
}

// Test: Large batch size
TEST_F(JobSystemTest, LargeBatch) {
    std::vector<int> data(100000, 0);

    jobs.ParallelFor(static_cast<int>(data.size()), [&](int i) {
        data[i] = i % 256;
    });

    // Spot check
    EXPECT_EQ(data[0], 0);
    EXPECT_EQ(data[255], 255);
    EXPECT_EQ(data[256], 0);
    EXPECT_EQ(data[99999], 99999 % 256);
}

// Test: Global instance (if set)
TEST(JobSystemGlobalTest, GlobalInstance) {
    // This test runs separately from JobSystemTest fixture

    if (g_JobSystem == nullptr) {
        g_JobSystem = new JobSystem();
        g_JobSystem->Initialize();
    }

    std::atomic<int> counter(0);

    PARALLEL_FOR(100, [&](int) {
        counter++;
    });

    EXPECT_EQ(counter.load(), 100);

    // Note: Don't delete g_JobSystem here, it might be used by other tests
}

// Benchmark: Compare parallel vs serial for realistic workload
TEST_F(JobSystemTest, BenchmarkRealistic) {
    // Simulate structural analysis mass calculation
    struct Node {
        float mass = 0.0f;
        float x, y, z;
    };

    std::vector<Node> nodes(5000);

    // Initialize positions
    for (size_t i = 0; i < nodes.size(); i++) {
        nodes[i].x = (i % 100) * 0.05f;
        nodes[i].y = (i / 100) * 0.05f;
        nodes[i].z = 0.0f;
    }

    auto calculate_mass = [](Node& node) {
        // Simulate raycast + mass calculation
        float total_mass = 0.0f;
        for (int i = 0; i < 100; i++) {
            total_mass += node.y * 2400.0f;  // Density calculation
        }
        node.mass = total_mass;
    };

    // Serial
    auto serial_start = Clock::now();
    for (auto& node : nodes) {
        calculate_mass(node);
    }
    float serial_time = GetElapsedMS(serial_start);

    // Reset
    for (auto& node : nodes) {
        node.mass = 0.0f;
    }

    // Parallel
    auto parallel_start = Clock::now();
    jobs.ParallelFor(static_cast<int>(nodes.size()), [&](int i) {
        calculate_mass(nodes[i]);
    });
    float parallel_time = GetElapsedMS(parallel_start);

    float speedup = serial_time / parallel_time;

    std::cout << "\nRealistic Workload Benchmark (5000 nodes):\n";
    std::cout << "  Serial:   " << serial_time << "ms\n";
    std::cout << "  Parallel: " << parallel_time << "ms\n";
    std::cout << "  Speedup:  " << speedup << "x\n";

    // Expect at least 1.5x speedup (light workload may not scale perfectly)
    EXPECT_GT(speedup, 1.5f);

    // Verify results are correct (each node has different y, so different mass)
    for (const auto& node : nodes) {
        float expected_mass = node.y * 2400.0f * 100;
        EXPECT_FLOAT_EQ(node.mass, expected_mass);
    }
}
