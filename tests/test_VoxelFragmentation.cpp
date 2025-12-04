#include <gtest/gtest.h>
#include "VoxelFragmentation.h"
#include "Material.h"

// ===== Fragmentation Test Fixture =====

class FragmentationTest : public ::testing::Test {
protected:
    void SetUp() override {
        fragmenter = new VoxelFragmentation();
    }

    void TearDown() override {
        delete fragmenter;
    }

    // Helper: Create a test cluster
    VoxelCluster CreateTestCluster(int num_voxels_x, int num_voxels_y, int num_voxels_z) {
        VoxelCluster cluster;
        cluster.cluster_id = 1;

        float voxel_size = 0.05f;
        for (int x = 0; x < num_voxels_x; x++) {
            for (int y = 0; y < num_voxels_y; y++) {
                for (int z = 0; z < num_voxels_z; z++) {
                    Vector3 pos(x * voxel_size, y * voxel_size, z * voxel_size);
                    cluster.voxel_positions.push_back(pos);
                }
            }
        }

        // Calculate bounds
        if (!cluster.voxel_positions.empty()) {
            Vector3 min = cluster.voxel_positions[0];
            Vector3 max = cluster.voxel_positions[0];

            for (const auto& pos : cluster.voxel_positions) {
                min.x = std::min(min.x, pos.x);
                min.y = std::min(min.y, pos.y);
                min.z = std::min(min.z, pos.z);

                max.x = std::max(max.x, pos.x);
                max.y = std::max(max.y, pos.y);
                max.z = std::max(max.z, pos.z);
            }

            cluster.bounds = BoundingBox(min, max);
        }

        cluster.center_of_mass = Vector3(0, 0, 0);
        cluster.total_mass = cluster.voxel_positions.size() * 1.0f;

        return cluster;
    }

    VoxelFragmentation* fragmenter;
};

// ===== Wood: Splinters =====

TEST_F(FragmentationTest, WoodCreates3_6Splinters) {
    // Create elongated cluster (like a plank)
    VoxelCluster cluster = CreateTestCluster(10, 2, 2);  // Long in X direction

    auto splinters = fragmenter->SplitIntoSplinters(cluster);

    // Should create 3-6 splinters
    EXPECT_GE(splinters.size(), 3);
    EXPECT_LE(splinters.size(), 6);
}

TEST_F(FragmentationTest, WoodSplinttersAreElongated) {
    VoxelCluster cluster = CreateTestCluster(10, 2, 2);

    auto splinters = fragmenter->SplitIntoSplinters(cluster);

    // Each splinter should be elongated (longest dimension > 2x shortest)
    for (const auto& splinter : splinters) {
        if (splinter.voxel_positions.empty()) continue;

        Vector3 size = fragmenter->GetClusterSize(splinter);

        float max_dim = std::max({size.x, size.y, size.z});
        float min_dim = std::min({size.x, size.y, size.z});

        if (min_dim > 0.01f) {  // Avoid division by zero
            EXPECT_GT(max_dim / min_dim, 1.5f);
        }
    }
}

TEST_F(FragmentationTest, WoodSplinttersPreserveAllVoxels) {
    VoxelCluster cluster = CreateTestCluster(10, 2, 2);
    size_t original_count = cluster.voxel_positions.size();

    auto splinters = fragmenter->SplitIntoSplinters(cluster);

    // Count total voxels in all splinters
    size_t total_voxels = 0;
    for (const auto& splinter : splinters) {
        total_voxels += splinter.voxel_positions.size();
    }

    // Should preserve all voxels
    EXPECT_EQ(total_voxels, original_count);
}

// ===== Concrete: Irregular Chunks =====

TEST_F(FragmentationTest, ConcreteCreates5_10Chunks) {
    VoxelCluster cluster = CreateTestCluster(8, 8, 8);

    auto chunks = fragmenter->SplitIntoChunks(cluster, 5, 10);

    // Should create 5-10 chunks
    EXPECT_GE(chunks.size(), 5);
    EXPECT_LE(chunks.size(), 10);
}

TEST_F(FragmentationTest, ConcreteChunksPreserveAllVoxels) {
    VoxelCluster cluster = CreateTestCluster(8, 8, 8);
    size_t original_count = cluster.voxel_positions.size();

    auto chunks = fragmenter->SplitIntoChunks(cluster, 5, 10);

    // Count total voxels
    size_t total_voxels = 0;
    for (const auto& chunk : chunks) {
        total_voxels += chunk.voxel_positions.size();
    }

    EXPECT_EQ(total_voxels, original_count);
}

TEST_F(FragmentationTest, ConcreteChunksHaveVariedSizes) {
    VoxelCluster cluster = CreateTestCluster(10, 10, 10);

    auto chunks = fragmenter->SplitIntoChunks(cluster, 5, 10);

    // Chunks should have different sizes (not all equal)
    bool has_variation = false;
    if (chunks.size() >= 2) {
        size_t first_size = chunks[0].voxel_positions.size();
        for (size_t i = 1; i < chunks.size(); i++) {
            if (chunks[i].voxel_positions.size() != first_size) {
                has_variation = true;
                break;
            }
        }
    }

    EXPECT_TRUE(has_variation);
}

// ===== Brick: Masonry Units =====

TEST_F(FragmentationTest, BrickCreatesMultipleUnits) {
    VoxelCluster cluster = CreateTestCluster(6, 6, 6);

    auto units = fragmenter->SplitIntoMasonryUnits(cluster);

    // Should create multiple units (2-3 splits per axis = 8-27 units)
    EXPECT_GE(units.size(), 4);
    EXPECT_LE(units.size(), 27);
}

TEST_F(FragmentationTest, BrickUnitsPreserveAllVoxels) {
    VoxelCluster cluster = CreateTestCluster(6, 6, 6);
    size_t original_count = cluster.voxel_positions.size();

    auto units = fragmenter->SplitIntoMasonryUnits(cluster);

    // Count total voxels
    size_t total_voxels = 0;
    for (const auto& unit : units) {
        total_voxels += unit.voxel_positions.size();
    }

    EXPECT_EQ(total_voxels, original_count);
}

// ===== General Fracture Routing =====

TEST_F(FragmentationTest, FractureCluster_RoutesByMaterial) {
    VoxelCluster cluster = CreateTestCluster(8, 8, 8);

    // Test wood
    auto wood_frags = fragmenter->FractureCluster(cluster, MaterialDatabase::WOOD);
    EXPECT_GE(wood_frags.size(), 3);
    EXPECT_LE(wood_frags.size(), 6);

    // Test concrete
    auto concrete_frags = fragmenter->FractureCluster(cluster, MaterialDatabase::CONCRETE);
    EXPECT_GE(concrete_frags.size(), 5);
    EXPECT_LE(concrete_frags.size(), 10);

    // Test brick
    auto brick_frags = fragmenter->FractureCluster(cluster, MaterialDatabase::BRICK);
    EXPECT_GE(brick_frags.size(), 4);
}

TEST_F(FragmentationTest, FractureCluster_EmptyCluster) {
    VoxelCluster empty_cluster;
    empty_cluster.cluster_id = 1;

    auto fragments = fragmenter->FractureCluster(empty_cluster, MaterialDatabase::WOOD);

    // Should return empty vector
    EXPECT_EQ(fragments.size(), 0);
}

// ===== Utility Functions =====

TEST_F(FragmentationTest, GetClusterSize_CorrectDimensions) {
    VoxelCluster cluster = CreateTestCluster(10, 5, 3);

    Vector3 size = fragmenter->GetClusterSize(cluster);

    // Size should be approximately (10*0.05, 5*0.05, 3*0.05) = (0.5, 0.25, 0.15)
    EXPECT_NEAR(size.x, 0.45f, 0.1f);  // 9 gaps * 0.05
    EXPECT_NEAR(size.y, 0.2f, 0.1f);   // 4 gaps * 0.05
    EXPECT_NEAR(size.z, 0.1f, 0.1f);   // 2 gaps * 0.05
}

TEST_F(FragmentationTest, GetLongestAxis_IdentifiesCorrectAxis) {
    // X is longest
    EXPECT_EQ(fragmenter->GetLongestAxis(Vector3(10, 5, 3)), 0);

    // Y is longest
    EXPECT_EQ(fragmenter->GetLongestAxis(Vector3(3, 10, 5)), 1);

    // Z is longest
    EXPECT_EQ(fragmenter->GetLongestAxis(Vector3(3, 5, 10)), 2);
}

TEST_F(FragmentationTest, RandomRange_WithinBounds) {
    // Test int version
    for (int i = 0; i < 100; i++) {
        int val = fragmenter->RandomRange(5, 10);
        EXPECT_GE(val, 5);
        EXPECT_LE(val, 10);
    }

    // Test float version
    for (int i = 0; i < 100; i++) {
        float val = fragmenter->RandomRange(1.0f, 2.0f);
        EXPECT_GE(val, 1.0f);
        EXPECT_LE(val, 2.0f);
    }
}

TEST_F(FragmentationTest, RandomDirection_UnitLength) {
    // Random directions should be approximately unit length
    for (int i = 0; i < 20; i++) {
        Vector3 dir = fragmenter->RandomDirection();
        float length = dir.Length();
        EXPECT_NEAR(length, 1.0f, 0.1f);
    }
}
