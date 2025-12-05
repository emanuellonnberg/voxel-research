#include <gtest/gtest.h>
#include <chrono>
#include "SpatialHash.h"
#include "VoxelWorld.h"
#include "Material.h"

// Test fixture for SpatialHash tests
class SpatialHashTest : public ::testing::Test {
protected:
    SpatialHash hash{1.0f};  // 1m cell size
    const float cell_size = 1.0f;
};

// Basic GridCell tests
TEST(GridCellTest, Construction) {
    GridCell cell1;
    EXPECT_EQ(cell1.x, 0);
    EXPECT_EQ(cell1.y, 0);
    EXPECT_EQ(cell1.z, 0);

    GridCell cell2(1, 2, 3);
    EXPECT_EQ(cell2.x, 1);
    EXPECT_EQ(cell2.y, 2);
    EXPECT_EQ(cell2.z, 3);
}

TEST(GridCellTest, Equality) {
    GridCell cell1(1, 2, 3);
    GridCell cell2(1, 2, 3);
    GridCell cell3(4, 5, 6);

    EXPECT_TRUE(cell1 == cell2);
    EXPECT_FALSE(cell1 == cell3);
}

TEST(GridCellTest, Hash) {
    GridCell cell1(1, 2, 3);
    GridCell cell2(1, 2, 3);
    GridCell cell3(4, 5, 6);

    GridCell::Hash hasher;
    EXPECT_EQ(hasher(cell1), hasher(cell2));
    EXPECT_NE(hasher(cell1), hasher(cell3));
}

// SpatialHash basic tests
TEST_F(SpatialHashTest, GetCell) {
    // Test cell calculation
    EXPECT_EQ(hash.GetCell(Vector3(0.5f, 0.5f, 0.5f)), GridCell(0, 0, 0));
    EXPECT_EQ(hash.GetCell(Vector3(1.5f, 1.5f, 1.5f)), GridCell(1, 1, 1));
    EXPECT_EQ(hash.GetCell(Vector3(-0.5f, -0.5f, -0.5f)), GridCell(-1, -1, -1));

    // Test boundaries
    EXPECT_EQ(hash.GetCell(Vector3(0.0f, 0.0f, 0.0f)), GridCell(0, 0, 0));
    EXPECT_EQ(hash.GetCell(Vector3(1.0f, 1.0f, 1.0f)), GridCell(1, 1, 1));
}

TEST_F(SpatialHashTest, InsertAndQuery) {
    Vector3 pos1(0.5f, 0.5f, 0.5f);
    Vector3 pos2(1.5f, 1.5f, 1.5f);

    hash.Insert(pos1);
    hash.Insert(pos2);

    EXPECT_EQ(hash.GetTotalPositions(), 2);
    EXPECT_EQ(hash.GetCellCount(), 2);  // Two different cells
}

TEST_F(SpatialHashTest, QueryRadius_Single) {
    Vector3 pos(0.5f, 0.5f, 0.5f);
    hash.Insert(pos);

    // Query at exact position
    auto results = hash.QueryRadius(pos, 0.1f);
    EXPECT_EQ(results.size(), 1);

    // Query nearby
    auto results2 = hash.QueryRadius(Vector3(0.6f, 0.6f, 0.6f), 0.2f);
    EXPECT_EQ(results2.size(), 1);

    // Query far away
    auto results3 = hash.QueryRadius(Vector3(10.0f, 10.0f, 10.0f), 0.5f);
    EXPECT_EQ(results3.size(), 0);
}

TEST_F(SpatialHashTest, QueryRadius_Multiple) {
    // Insert positions in a line
    for (int i = 0; i < 10; i++) {
        hash.Insert(Vector3(i * 0.1f, 0, 0));
    }

    EXPECT_EQ(hash.GetTotalPositions(), 10);

    // Query center - should find multiple
    auto results = hash.QueryRadius(Vector3(0.5f, 0, 0), 0.3f);
    EXPECT_GT(results.size(), 3);
    EXPECT_LE(results.size(), 7);  // Approx 6-7 within 0.3m
}

TEST_F(SpatialHashTest, QueryBox) {
    // Insert positions in a grid
    for (int x = 0; x < 5; x++) {
        for (int y = 0; y < 5; y++) {
            hash.Insert(Vector3(x * 0.5f, y * 0.5f, 0));
        }
    }

    EXPECT_EQ(hash.GetTotalPositions(), 25);

    // Query a small box
    auto results = hash.QueryBox(Vector3(0, 0, -0.1f), Vector3(1.0f, 1.0f, 0.1f));
    EXPECT_GT(results.size(), 4);  // At least 2x2 = 4
    EXPECT_LE(results.size(), 9);  // At most 3x3 = 9
}

TEST_F(SpatialHashTest, Remove) {
    Vector3 pos1(0.5f, 0.5f, 0.5f);
    Vector3 pos2(1.5f, 1.5f, 1.5f);

    hash.Insert(pos1);
    hash.Insert(pos2);
    EXPECT_EQ(hash.GetTotalPositions(), 2);

    hash.Remove(pos1);
    EXPECT_EQ(hash.GetTotalPositions(), 1);

    // Verify pos1 is gone
    auto results = hash.QueryRadius(pos1, 0.1f);
    EXPECT_EQ(results.size(), 0);

    // Verify pos2 is still there
    auto results2 = hash.QueryRadius(pos2, 0.1f);
    EXPECT_EQ(results2.size(), 1);
}

TEST_F(SpatialHashTest, Clear) {
    for (int i = 0; i < 100; i++) {
        hash.Insert(Vector3(i * 0.1f, 0, 0));
    }

    EXPECT_EQ(hash.GetTotalPositions(), 100);

    hash.Clear();
    EXPECT_EQ(hash.GetTotalPositions(), 0);
    EXPECT_EQ(hash.GetCellCount(), 0);
}

// VoxelWorld integration tests
TEST(VoxelWorldSpatialHashTest, EnableDisable) {
    VoxelWorld world(0.05f);

    EXPECT_FALSE(world.IsSpatialHashingEnabled());

    world.EnableSpatialHashing(1.0f);
    EXPECT_TRUE(world.IsSpatialHashingEnabled());

    world.DisableSpatialHashing();
    EXPECT_FALSE(world.IsSpatialHashingEnabled());
}

TEST(VoxelWorldSpatialHashTest, EnableWithExistingVoxels) {
    VoxelWorld world(0.05f);

    // Add voxels before enabling spatial hash
    for (int i = 0; i < 10; i++) {
        world.SetVoxel(Vector3(i * 0.05f, 0, 0), Voxel(MaterialDatabase::BRICK));
    }

    EXPECT_EQ(world.GetVoxelCount(), 10);

    // Enable spatial hash - should populate with existing voxels
    world.EnableSpatialHashing(1.0f);
    EXPECT_TRUE(world.IsSpatialHashingEnabled());

    // Query should work
    auto results = world.GetVoxelsInRadius(Vector3(0.25f, 0, 0), 0.5f);
    EXPECT_GT(results.size(), 0);
}

TEST(VoxelWorldSpatialHashTest, QueryConsistency) {
    VoxelWorld world(0.05f);

    // Create structure
    for (int x = 0; x < 10; x++) {
        for (int y = 0; y < 10; y++) {
            world.SetVoxel(Vector3(x * 0.05f, y * 0.05f, 0), Voxel(MaterialDatabase::BRICK));
        }
    }

    // Query without spatial hash
    auto results_without = world.GetVoxelsInRadius(Vector3(0.25f, 0.25f, 0), 0.2f);

    // Enable spatial hash and query again
    world.EnableSpatialHashing(0.5f);
    auto results_with = world.GetVoxelsInRadius(Vector3(0.25f, 0.25f, 0), 0.2f);

    // Results should be the same (order may differ)
    EXPECT_EQ(results_without.size(), results_with.size());
}

TEST(VoxelWorldSpatialHashTest, SetVoxelUpdatesHash) {
    VoxelWorld world(0.05f);
    world.EnableSpatialHashing(1.0f);

    Vector3 pos(0.5f, 0.5f, 0.5f);
    world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));

    // Should be able to find it
    auto results = world.GetVoxelsInRadius(pos, 0.1f);
    EXPECT_EQ(results.size(), 1);
}

TEST(VoxelWorldSpatialHashTest, RemoveVoxelUpdatesHash) {
    VoxelWorld world(0.05f);
    world.EnableSpatialHashing(1.0f);

    Vector3 pos(0.5f, 0.5f, 0.5f);
    world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
    EXPECT_EQ(world.GetVoxelCount(), 1);

    world.RemoveVoxel(pos);
    EXPECT_EQ(world.GetVoxelCount(), 0);

    // Should not find it
    auto results = world.GetVoxelsInRadius(pos, 0.1f);
    EXPECT_EQ(results.size(), 0);
}

TEST(VoxelWorldSpatialHashTest, ClearUpdatesHash) {
    VoxelWorld world(0.05f);
    world.EnableSpatialHashing(1.0f);

    // Add many voxels
    for (int i = 0; i < 100; i++) {
        world.SetVoxel(Vector3(i * 0.05f, 0, 0), Voxel(MaterialDatabase::BRICK));
    }

    EXPECT_EQ(world.GetVoxelCount(), 100);

    world.Clear();
    EXPECT_EQ(world.GetVoxelCount(), 0);

    // Should find nothing
    auto results = world.GetVoxelsInRadius(Vector3(0, 0, 0), 10.0f);
    EXPECT_EQ(results.size(), 0);
}

// Performance tests
TEST(VoxelWorldSpatialHashTest, Performance_LargeStructure) {
    using namespace std::chrono;

    VoxelWorld world(0.05f);

    // Create large structure (50x50x20 = 50,000 voxels)
    for (int x = 0; x < 50; x++) {
        for (int y = 0; y < 50; y++) {
            for (int z = 0; z < 20; z++) {
                world.SetVoxel(Vector3(x * 0.05f, y * 0.05f, z * 0.05f),
                              Voxel(MaterialDatabase::CONCRETE));
            }
        }
    }

    EXPECT_EQ(world.GetVoxelCount(), 50000);

    Vector3 query_center(1.25f, 1.25f, 0.5f);
    float query_radius = 0.5f;

    // Benchmark WITHOUT spatial hash
    auto start1 = high_resolution_clock::now();
    for (int i = 0; i < 100; i++) {
        auto results = world.GetVoxelsInRadius(query_center, query_radius);
    }
    auto end1 = high_resolution_clock::now();
    auto duration_without = duration_cast<microseconds>(end1 - start1);

    // Enable spatial hash
    world.EnableSpatialHashing(1.0f);

    // Benchmark WITH spatial hash
    auto start2 = high_resolution_clock::now();
    for (int i = 0; i < 100; i++) {
        auto results = world.GetVoxelsInRadius(query_center, query_radius);
    }
    auto end2 = high_resolution_clock::now();
    auto duration_with = duration_cast<microseconds>(end2 - start2);

    // Spatial hash should be at least 5x faster for large structures
    // (actual speedup is typically 8-15x depending on structure and query)
    EXPECT_LT(duration_with.count(), duration_without.count() / 5);

    std::cout << "  WITHOUT spatial hash: " << duration_without.count() << " μs\n";
    std::cout << "  WITH spatial hash:    " << duration_with.count() << " μs\n";
    std::cout << "  Speedup: " << (static_cast<double>(duration_without.count()) / duration_with.count()) << "x\n";
}
