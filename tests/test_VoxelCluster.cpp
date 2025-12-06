#include <gtest/gtest.h>
#include "VoxelCluster.h"
#include "VoxelWorld.h"
#include "Material.h"

// Test fixture for VoxelCluster tests
class VoxelClusterTest : public ::testing::Test {
protected:
    const float voxel_size = 0.05f;
    VoxelWorld world{voxel_size};  // Initialize directly (VoxelWorld not copyable due to mutexes)

    void SetUp() override {
        // World is already initialized with voxel_size
        world.Clear();  // Clear for each test
    }
};

// Basic cluster structure tests
TEST_F(VoxelClusterTest, EmptyCluster) {
    VoxelCluster cluster;
    EXPECT_TRUE(cluster.IsEmpty());
    EXPECT_EQ(cluster.GetVoxelCount(), 0);
    EXPECT_FLOAT_EQ(cluster.total_mass, 0.0f);
}

TEST_F(VoxelClusterTest, SingleVoxelCluster) {
    Vector3 pos(0, 0, 0);
    world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));

    auto clusters = VoxelClustering::FindAllClusters(world);

    ASSERT_EQ(clusters.size(), 1);
    EXPECT_EQ(clusters[0].GetVoxelCount(), 1);
    EXPECT_GT(clusters[0].total_mass, 0.0f);
}

TEST_F(VoxelClusterTest, TwoConnectedVoxels) {
    // Create two adjacent voxels
    world.SetVoxel(Vector3(0, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));

    auto clusters = VoxelClustering::FindAllClusters(world);

    // Should be one cluster (connected)
    ASSERT_EQ(clusters.size(), 1);
    EXPECT_EQ(clusters[0].GetVoxelCount(), 2);
}

TEST_F(VoxelClusterTest, TwoDisconnectedVoxels) {
    // Create two separated voxels
    world.SetVoxel(Vector3(0, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(1.0f, 0, 0), Voxel(MaterialDatabase::BRICK));  // Far away

    auto clusters = VoxelClustering::FindAllClusters(world);

    // Should be two separate clusters
    ASSERT_EQ(clusters.size(), 2);
    EXPECT_EQ(clusters[0].GetVoxelCount(), 1);
    EXPECT_EQ(clusters[1].GetVoxelCount(), 1);
}

TEST_F(VoxelClusterTest, LineOfVoxels) {
    // Create a line of connected voxels
    for (int i = 0; i < 10; i++) {
        world.SetVoxel(Vector3(i * voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));
    }

    auto clusters = VoxelClustering::FindAllClusters(world);

    // Should be one cluster
    ASSERT_EQ(clusters.size(), 1);
    EXPECT_EQ(clusters[0].GetVoxelCount(), 10);
}

TEST_F(VoxelClusterTest, CubeCluster) {
    // Create a 3x3x3 cube
    for (int x = 0; x < 3; x++) {
        for (int y = 0; y < 3; y++) {
            for (int z = 0; z < 3; z++) {
                world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, z * voxel_size),
                              Voxel(MaterialDatabase::CONCRETE));
            }
        }
    }

    auto clusters = VoxelClustering::FindAllClusters(world);

    // Should be one cluster
    ASSERT_EQ(clusters.size(), 1);
    EXPECT_EQ(clusters[0].GetVoxelCount(), 27);
}

TEST_F(VoxelClusterTest, TwoSeparateStructures) {
    // Create structure 1: 3x3x1 at origin
    for (int x = 0; x < 3; x++) {
        for (int y = 0; y < 3; y++) {
            world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, 0),
                          Voxel(MaterialDatabase::BRICK));
        }
    }

    // Create structure 2: 2x2x1 far away
    for (int x = 0; x < 2; x++) {
        for (int y = 0; y < 2; y++) {
            world.SetVoxel(Vector3((x + 10) * voxel_size, y * voxel_size, 0),
                          Voxel(MaterialDatabase::CONCRETE));
        }
    }

    auto clusters = VoxelClustering::FindAllClusters(world);

    // Should be two separate clusters
    ASSERT_EQ(clusters.size(), 2);

    // Sort by voxel count for consistent testing
    if (clusters[0].GetVoxelCount() < clusters[1].GetVoxelCount()) {
        std::swap(clusters[0], clusters[1]);
    }

    EXPECT_EQ(clusters[0].GetVoxelCount(), 9);   // 3x3
    EXPECT_EQ(clusters[1].GetVoxelCount(), 4);   // 2x2
}

TEST_F(VoxelClusterTest, BrokenBridge) {
    // Create bridge pillars
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(MaterialDatabase::BRICK));
        world.SetVoxel(Vector3(10 * voxel_size, y * voxel_size, 0), Voxel(MaterialDatabase::BRICK));
    }

    // Create partial bridge deck (broken in middle)
    for (int x = 0; x < 4; x++) {
        world.SetVoxel(Vector3(x * voxel_size, 5 * voxel_size, 0), Voxel(MaterialDatabase::CONCRETE));
    }
    for (int x = 7; x < 11; x++) {
        world.SetVoxel(Vector3(x * voxel_size, 5 * voxel_size, 0), Voxel(MaterialDatabase::CONCRETE));
    }

    auto clusters = VoxelClustering::FindAllClusters(world);

    // Should be two separate clusters (one per pillar + deck section)
    ASSERT_EQ(clusters.size(), 2);
}

// Cluster property tests
TEST_F(VoxelClusterTest, ClusterCenterOfMass) {
    // Create symmetric structure
    world.SetVoxel(Vector3(0, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(0, voxel_size, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(voxel_size, voxel_size, 0), Voxel(MaterialDatabase::BRICK));

    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 1);

    // Center of mass should be at center of square
    Vector3 expected_com(voxel_size * 0.5f, voxel_size * 0.5f, 0);
    EXPECT_NEAR(clusters[0].center_of_mass.x, expected_com.x, 0.001f);
    EXPECT_NEAR(clusters[0].center_of_mass.y, expected_com.y, 0.001f);
    EXPECT_NEAR(clusters[0].center_of_mass.z, expected_com.z, 0.001f);
}

TEST_F(VoxelClusterTest, ClusterTotalMass) {
    // Create cluster with known material
    for (int i = 0; i < 5; i++) {
        world.SetVoxel(Vector3(i * voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));
    }

    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 1);

    // Calculate expected mass
    const Material& brick = g_materials.GetMaterial(MaterialDatabase::BRICK);
    float voxel_volume = voxel_size * voxel_size * voxel_size;
    float expected_mass = brick.density * voxel_volume * 5;  // 5 voxels

    EXPECT_NEAR(clusters[0].total_mass, expected_mass, 0.001f);
}

TEST_F(VoxelClusterTest, ClusterBounds) {
    // Create L-shaped structure
    world.SetVoxel(Vector3(0, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(2 * voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(0, voxel_size, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(0, 2 * voxel_size, 0), Voxel(MaterialDatabase::BRICK));

    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 1);

    // Check bounds
    EXPECT_FLOAT_EQ(clusters[0].bounds.min.x, 0.0f);
    EXPECT_FLOAT_EQ(clusters[0].bounds.min.y, 0.0f);
    EXPECT_FLOAT_EQ(clusters[0].bounds.max.x, 2 * voxel_size);
    EXPECT_FLOAT_EQ(clusters[0].bounds.max.y, 2 * voxel_size);
}

// Ground detection tests
TEST_F(VoxelClusterTest, GroundedCluster) {
    float ground_level = 0.0f;

    // Create structure on ground
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(MaterialDatabase::BRICK));
    }

    std::vector<bool> grounded;
    auto clusters = VoxelClustering::FindClustersWithGrounding(world, ground_level, grounded);

    ASSERT_EQ(clusters.size(), 1);
    ASSERT_EQ(grounded.size(), 1);
    EXPECT_TRUE(grounded[0]);  // Should be grounded
}

TEST_F(VoxelClusterTest, FloatingCluster) {
    float ground_level = 0.0f;

    // Create structure floating above ground
    for (int y = 10; y < 15; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(MaterialDatabase::BRICK));
    }

    std::vector<bool> grounded;
    auto clusters = VoxelClustering::FindClustersWithGrounding(world, ground_level, grounded);

    ASSERT_EQ(clusters.size(), 1);
    ASSERT_EQ(grounded.size(), 1);
    EXPECT_FALSE(grounded[0]);  // Should NOT be grounded
}

TEST_F(VoxelClusterTest, MixedGroundedAndFloating) {
    float ground_level = 0.0f;

    // Create grounded structure
    for (int y = 0; y < 3; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(MaterialDatabase::BRICK));
    }

    // Create floating structure
    for (int y = 10; y < 13; y++) {
        world.SetVoxel(Vector3(5 * voxel_size, y * voxel_size, 0), Voxel(MaterialDatabase::CONCRETE));
    }

    std::vector<bool> grounded;
    auto clusters = VoxelClustering::FindClustersWithGrounding(world, ground_level, grounded);

    ASSERT_EQ(clusters.size(), 2);
    ASSERT_EQ(grounded.size(), 2);

    // One should be grounded, one should not (order may vary)
    int grounded_count = (grounded[0] ? 1 : 0) + (grounded[1] ? 1 : 0);
    EXPECT_EQ(grounded_count, 1);
}

// Edge case tests
TEST_F(VoxelClusterTest, EmptyWorld) {
    auto clusters = VoxelClustering::FindAllClusters(world);
    EXPECT_EQ(clusters.size(), 0);
}

TEST_F(VoxelClusterTest, DiagonalNotConnected) {
    // Diagonal voxels are not connected (6-connectivity)
    world.SetVoxel(Vector3(0, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(voxel_size, voxel_size, 0), Voxel(MaterialDatabase::BRICK));

    auto clusters = VoxelClustering::FindAllClusters(world);

    // Should be two separate clusters (diagonal not connected in 6-connectivity)
    ASSERT_EQ(clusters.size(), 2);
}

TEST_F(VoxelClusterTest, LargeConnectedStructure) {
    // Create 10x10x10 cube
    for (int x = 0; x < 10; x++) {
        for (int y = 0; y < 10; y++) {
            for (int z = 0; z < 10; z++) {
                world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, z * voxel_size),
                              Voxel(MaterialDatabase::CONCRETE));
            }
        }
    }

    auto clusters = VoxelClustering::FindAllClusters(world);

    ASSERT_EQ(clusters.size(), 1);
    EXPECT_EQ(clusters[0].GetVoxelCount(), 1000);
    EXPECT_GT(clusters[0].total_mass, 0.0f);
}

TEST_F(VoxelClusterTest, ClusterIDs) {
    // Create three separate structures
    world.SetVoxel(Vector3(0, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(5 * voxel_size, 0, 0), Voxel(MaterialDatabase::CONCRETE));
    world.SetVoxel(Vector3(10 * voxel_size, 0, 0), Voxel(MaterialDatabase::WOOD));

    auto clusters = VoxelClustering::FindAllClusters(world);

    ASSERT_EQ(clusters.size(), 3);

    // Check that each cluster has a unique ID
    std::set<size_t> ids;
    for (const auto& cluster : clusters) {
        ids.insert(cluster.cluster_id);
    }
    EXPECT_EQ(ids.size(), 3);  // All unique
}
