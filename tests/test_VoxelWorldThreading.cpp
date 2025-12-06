#include <gtest/gtest.h>
#include "VoxelWorld.h"
#include "Material.h"
#include <thread>
#include <atomic>
#include <vector>
#include <chrono>
#include <random>

/**
 * Threading safety tests for VoxelWorld
 * Week 13 Day 44: Thread-Safe Data Structures
 *
 * These tests verify that VoxelWorld is safe for concurrent access:
 * - Multiple concurrent readers
 * - Multiple concurrent writers
 * - Mixed read/write scenarios
 * - Surface cache under concurrent modification
 */

class VoxelWorldThreadingTest : public ::testing::Test {
protected:
    const float voxel_size = 0.05f;
    VoxelWorld world{voxel_size};

    void SetUp() override {
        world.Clear();
    }
};

// Test concurrent reads from multiple threads
TEST_F(VoxelWorldThreadingTest, ConcurrentReads) {
    // Populate world with a 10x10x10 cube
    for (int x = 0; x < 10; x++) {
        for (int y = 0; y < 10; y++) {
            for (int z = 0; z < 10; z++) {
                Vector3 pos(x * voxel_size, y * voxel_size, z * voxel_size);
                world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
            }
        }
    }

    EXPECT_EQ(world.GetVoxelCount(), 1000);

    std::atomic<int> read_count(0);
    const int num_readers = 8;
    const int reads_per_thread = 100;

    std::vector<std::thread> readers;

    // Spawn multiple reader threads
    for (int t = 0; t < num_readers; t++) {
        readers.emplace_back([&, t]() {
            for (int i = 0; i < reads_per_thread; i++) {
                // Read a voxel
                Vector3 pos((i % 10) * voxel_size,
                           ((i / 10) % 10) * voxel_size,
                           ((i / 100) % 10) * voxel_size);

                if (world.HasVoxel(pos)) {
                    Voxel v = world.GetVoxel(pos);
                    if (v.material_id == MaterialDatabase::BRICK) {
                        read_count++;
                    }
                }
            }
        });
    }

    // Wait for all readers
    for (auto& reader : readers) {
        reader.join();
    }

    // All reads should have succeeded
    EXPECT_EQ(read_count.load(), num_readers * reads_per_thread);
}

// Test concurrent writes from multiple threads
TEST_F(VoxelWorldThreadingTest, ConcurrentWrites) {
    const int num_writers = 4;
    const int writes_per_thread = 250;

    std::vector<std::thread> writers;

    // Spawn multiple writer threads, each writing to different regions
    for (int t = 0; t < num_writers; t++) {
        writers.emplace_back([&, t]() {
            for (int i = 0; i < writes_per_thread; i++) {
                // Each thread writes to its own X coordinate to avoid conflicts
                Vector3 pos(t * 10 * voxel_size + (i % 10) * voxel_size,
                           ((i / 10) % 5) * voxel_size,
                           ((i / 50) % 5) * voxel_size);

                world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
            }
        });
    }

    // Wait for all writers
    for (auto& writer : writers) {
        writer.join();
    }

    // Verify total count
    EXPECT_EQ(world.GetVoxelCount(), num_writers * writes_per_thread);
}

// Test mixed concurrent reads and writes
TEST_F(VoxelWorldThreadingTest, MixedReadWrite) {
    // Pre-populate with some voxels
    for (int i = 0; i < 100; i++) {
        Vector3 pos(i * voxel_size, 0, 0);
        world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
    }

    std::atomic<bool> stop_flag(false);
    std::atomic<int> write_count(0);
    std::atomic<int> read_count(0);

    // Writer thread
    std::thread writer([&]() {
        for (int i = 100; i < 300; i++) {
            Vector3 pos(i * voxel_size, 0, 0);
            world.SetVoxel(pos, Voxel(MaterialDatabase::CONCRETE));
            write_count++;
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        stop_flag = true;
    });

    // Multiple reader threads
    std::vector<std::thread> readers;
    for (int t = 0; t < 4; t++) {
        readers.emplace_back([&]() {
            while (!stop_flag.load()) {
                size_t count = world.GetVoxelCount();
                if (count >= 100) {  // Should always have at least initial voxels
                    read_count++;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(200));
            }
        });
    }

    // Wait for completion
    writer.join();
    for (auto& reader : readers) {
        reader.join();
    }

    // Verify final state
    EXPECT_EQ(write_count.load(), 200);
    EXPECT_GT(read_count.load(), 0);
    EXPECT_EQ(world.GetVoxelCount(), 300);
}

// Test concurrent modifications to the same region
TEST_F(VoxelWorldThreadingTest, ConcurrentModificationsSameRegion) {
    const int num_threads = 4;
    const int operations_per_thread = 100;

    std::vector<std::thread> threads;

    // Each thread adds and removes voxels in the same region
    for (int t = 0; t < num_threads; t++) {
        threads.emplace_back([&, t]() {
            std::mt19937 rng(t);  // Each thread has its own RNG seed
            std::uniform_int_distribution<int> dist(0, 9);

            for (int i = 0; i < operations_per_thread; i++) {
                int x = dist(rng);
                int y = dist(rng);
                int z = dist(rng);
                Vector3 pos(x * voxel_size, y * voxel_size, z * voxel_size);

                if (i % 2 == 0) {
                    world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
                } else {
                    world.RemoveVoxel(pos);
                }
            }
        });
    }

    // Wait for all threads
    for (auto& thread : threads) {
        thread.join();
    }

    // The exact count is non-deterministic due to race conditions,
    // but the operations should complete without crashes
    size_t final_count = world.GetVoxelCount();
    EXPECT_LE(final_count, 1000);  // Can't exceed total possible voxels
}

// Test concurrent surface cache access
TEST_F(VoxelWorldThreadingTest, ConcurrentSurfaceCacheAccess) {
    // Create a structure with known surface voxels
    for (int x = 0; x < 5; x++) {
        for (int y = 0; y < 5; y++) {
            for (int z = 0; z < 5; z++) {
                Vector3 pos(x * voxel_size, y * voxel_size, z * voxel_size);
                world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
            }
        }
    }

    std::atomic<int> surface_read_count(0);
    std::atomic<bool> stop_flag(false);

    // Writer thread that modifies the structure
    std::thread writer([&]() {
        for (int i = 0; i < 50; i++) {
            Vector3 pos((i % 5) * voxel_size, 0, 0);
            world.RemoveVoxel(pos);  // This invalidates surface cache
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        stop_flag = true;
    });

    // Multiple reader threads accessing surface cache
    std::vector<std::thread> readers;
    for (int t = 0; t < 4; t++) {
        readers.emplace_back([&]() {
            while (!stop_flag.load()) {
                auto surface = world.GetSurfaceVoxels();
                if (surface.size() > 0) {
                    surface_read_count++;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        });
    }

    // Wait for completion
    writer.join();
    for (auto& reader : readers) {
        reader.join();
    }

    // Verify reads happened
    EXPECT_GT(surface_read_count.load(), 0);
}

// Test GetNeighbors under concurrent access
TEST_F(VoxelWorldThreadingTest, ConcurrentNeighborQueries) {
    // Create a 3D grid
    for (int x = 0; x < 10; x++) {
        for (int y = 0; y < 10; y++) {
            for (int z = 0; z < 10; z++) {
                Vector3 pos(x * voxel_size, y * voxel_size, z * voxel_size);
                world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
            }
        }
    }

    std::atomic<int> query_count(0);
    const int num_threads = 8;

    std::vector<std::thread> threads;

    // Multiple threads querying neighbors
    for (int t = 0; t < num_threads; t++) {
        threads.emplace_back([&, t]() {
            for (int i = 0; i < 100; i++) {
                Vector3 pos(5 * voxel_size, 5 * voxel_size, 5 * voxel_size);
                auto neighbors = world.GetNeighbors(pos);

                // Interior voxel should have 6 neighbors
                if (neighbors.size() == 6) {
                    query_count++;
                }
            }
        });
    }

    // Wait for all threads
    for (auto& thread : threads) {
        thread.join();
    }

    EXPECT_EQ(query_count.load(), num_threads * 100);
}

// Test spatial queries under concurrent access
TEST_F(VoxelWorldThreadingTest, ConcurrentSpatialQueries) {
    // Create scattered voxels
    for (int i = 0; i < 100; i++) {
        Vector3 pos(i * voxel_size, (i % 10) * voxel_size, 0);
        world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
    }

    std::atomic<int> query_count(0);
    const int num_threads = 4;

    std::vector<std::thread> threads;

    // Multiple threads performing spatial queries
    for (int t = 0; t < num_threads; t++) {
        threads.emplace_back([&]() {
            for (int i = 0; i < 50; i++) {
                Vector3 center(2.5f, 0.25f, 0);
                float radius = 1.0f;

                auto voxels = world.GetVoxelsInRadius(center, radius);
                if (voxels.size() > 0) {
                    query_count++;
                }
            }
        });
    }

    // Wait for all threads
    for (auto& thread : threads) {
        thread.join();
    }

    EXPECT_GT(query_count.load(), 0);
}

// Performance test: measure overhead of locking
TEST_F(VoxelWorldThreadingTest, LockingOverhead) {
    const int iterations = 10000;

    // Sequential baseline
    auto start_seq = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; i++) {
        Vector3 pos(i * voxel_size, 0, 0);
        world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
    }
    auto end_seq = std::chrono::high_resolution_clock::now();
    auto duration_seq = std::chrono::duration_cast<std::chrono::microseconds>(end_seq - start_seq);

    world.Clear();

    // Parallel with 4 threads
    auto start_par = std::chrono::high_resolution_clock::now();
    std::vector<std::thread> threads;
    const int num_threads = 4;
    const int per_thread = iterations / num_threads;

    for (int t = 0; t < num_threads; t++) {
        threads.emplace_back([&, t]() {
            for (int i = 0; i < per_thread; i++) {
                Vector3 pos((t * per_thread + i) * voxel_size, 0, 0);
                world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
            }
        });
    }

    for (auto& thread : threads) {
        thread.join();
    }

    auto end_par = std::chrono::high_resolution_clock::now();
    auto duration_par = std::chrono::duration_cast<std::chrono::microseconds>(end_par - start_par);

    // Parallel should be reasonable compared to sequential
    // Note: With heavy write contention to a shared hash map, parallel can be slower
    // due to lock overhead. This test verifies locks work correctly, not performance.
    float ratio = static_cast<float>(duration_par.count()) / duration_seq.count();

    // Expect parallel to be at most 4x slower (acceptable for high lock contention)
    // In practice, parallel writes are slower than sequential for this workload
    EXPECT_LT(ratio, 4.0f);

    // Verify correct count
    EXPECT_EQ(world.GetVoxelCount(), iterations);
}
