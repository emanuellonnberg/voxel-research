#include <gtest/gtest.h>
#include "VoxelStability.h"
#include "VoxelWorld.h"
#include "VoxelCluster.h"
#include "Material.h"

// Default voxel size constant for tests
static constexpr float VOXEL_SIZE = 0.05f;

class VoxelStabilityTest : public ::testing::Test {
protected:
    void SetUp() override {
        world.Clear();
        wood_id = MaterialDatabase::WOOD;
    }

    VoxelWorld world;
    uint8_t wood_id;
};

// ===== Ground Detection Tests =====

TEST_F(VoxelStabilityTest, SingleGroundVoxel) {
    // Single voxel on ground
    world.SetVoxel(Vector3(0, 0, 0), Voxel(wood_id));

    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 1);

    auto ground_voxels = VoxelStability::FindGroundSupportVoxels(world, clusters[0]);
    EXPECT_EQ(ground_voxels.size(), 1);
    EXPECT_TRUE(ground_voxels.count(Vector3(0, 0, 0)) > 0);
}

TEST_F(VoxelStabilityTest, FloatingVoxel) {
    // Single voxel floating above ground
    world.SetVoxel(Vector3(0, 1, 0), Voxel(wood_id));

    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 1);

    auto ground_voxels = VoxelStability::FindGroundSupportVoxels(world, clusters[0]);
    EXPECT_EQ(ground_voxels.size(), 0);
}

TEST_F(VoxelStabilityTest, TowerWithOneGroundVoxel) {
    // Vertical tower: ground at y=0, others floating
    // Use VOXEL_SIZE spacing to ensure voxels are adjacent
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * VOXEL_SIZE, 0), Voxel(wood_id));
    }

    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 1);

    auto ground_voxels = VoxelStability::FindGroundSupportVoxels(world, clusters[0]);
    EXPECT_EQ(ground_voxels.size(), 1);
    EXPECT_TRUE(ground_voxels.count(Vector3(0, 0, 0)) > 0);
}

TEST_F(VoxelStabilityTest, FlatStructureOnGround) {
    // 3x3 flat structure on ground
    for (int x = -1; x <= 1; x++) {
        for (int z = -1; z <= 1; z++) {
            world.SetVoxel(Vector3(x * VOXEL_SIZE, 0, z * VOXEL_SIZE), Voxel(wood_id));
        }
    }

    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 1);

    auto ground_voxels = VoxelStability::FindGroundSupportVoxels(world, clusters[0]);
    EXPECT_EQ(ground_voxels.size(), 9);  // All 9 voxels touch ground
}

// ===== Support Chain Tests =====

TEST_F(VoxelStabilityTest, AllVoxelsSupported_Tower) {
    // Vertical tower - all voxels should be supported
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * VOXEL_SIZE, 0), Voxel(wood_id));
    }

    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 1);

    auto supported = VoxelStability::FindSupportedVoxels(world, clusters[0]);
    EXPECT_EQ(supported.size(), 5);  // All 5 voxels supported
}

TEST_F(VoxelStabilityTest, NoVoxelsSupported_Floating) {
    // Floating structure - no voxels supported
    for (int x = 0; x < 3; x++) {
        world.SetVoxel(Vector3(x * VOXEL_SIZE, 5 * VOXEL_SIZE, 0), Voxel(wood_id));
    }

    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 1);

    auto supported = VoxelStability::FindSupportedVoxels(world, clusters[0]);
    EXPECT_EQ(supported.size(), 0);  // Nothing supported
}

TEST_F(VoxelStabilityTest, PartiallySupported_Overhang) {
    // L-shape: vertical pillar + horizontal overhang
    // Pillar (supported)
    for (int y = 0; y < 3; y++) {
        world.SetVoxel(Vector3(0, y * VOXEL_SIZE, 0), Voxel(wood_id));
    }
    // Overhang extending from top (also supported via connection)
    for (int x = 1; x <= 2; x++) {
        world.SetVoxel(Vector3(x * VOXEL_SIZE, 2 * VOXEL_SIZE, 0), Voxel(wood_id));
    }

    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 1);

    auto supported = VoxelStability::FindSupportedVoxels(world, clusters[0]);
    EXPECT_EQ(supported.size(), 5);  // All 5 voxels connected to ground (3 pillar + 2 overhang)
}

TEST_F(VoxelStabilityTest, DisconnectedFloatingSection) {
    // Ground structure + disconnected floating section
    world.SetVoxel(Vector3(0, 0, 0), Voxel(wood_id));
    world.SetVoxel(Vector3(0, 1, 0), Voxel(wood_id));

    // Disconnected floating voxel
    world.SetVoxel(Vector3(5, 5, 5), Voxel(wood_id));

    // Should be 2 clusters
    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 2);

    // Find grounded cluster
    VoxelCluster* grounded = nullptr;
    VoxelCluster* floating = nullptr;
    for (auto& cluster : clusters) {
        auto ground_voxels = VoxelStability::FindGroundSupportVoxels(world, cluster);
        if (!ground_voxels.empty()) {
            grounded = &cluster;
        } else {
            floating = &cluster;
        }
    }

    ASSERT_NE(grounded, nullptr);
    ASSERT_NE(floating, nullptr);

    // Grounded cluster: all supported
    auto supported1 = VoxelStability::FindSupportedVoxels(world, *grounded);
    EXPECT_EQ(supported1.size(), 2);

    // Floating cluster: none supported
    auto supported2 = VoxelStability::FindSupportedVoxels(world, *floating);
    EXPECT_EQ(supported2.size(), 0);
}

// ===== Support Classification Tests =====

TEST_F(VoxelStabilityTest, ClassifySupport_SimpleTower) {
    // 3-high tower
    world.SetVoxel(Vector3(0, 0, 0), Voxel(wood_id));  // Ground
    world.SetVoxel(Vector3(0, VOXEL_SIZE, 0), Voxel(wood_id));  // Supported
    world.SetVoxel(Vector3(0, 2 * VOXEL_SIZE, 0), Voxel(wood_id));  // Supported

    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 1);

    auto classification = VoxelStability::ClassifySupport(world, clusters[0]);

    EXPECT_EQ(classification[Vector3(0, 0, 0)], SupportType::GROUNDED);
    EXPECT_EQ(classification[Vector3(0, VOXEL_SIZE, 0)], SupportType::SUPPORTED);
    EXPECT_EQ(classification[Vector3(0, 2 * VOXEL_SIZE, 0)], SupportType::SUPPORTED);
}

TEST_F(VoxelStabilityTest, ClassifySupport_FloatingStructure) {
    // Floating 2x2 platform
    for (int x = 0; x < 2; x++) {
        for (int z = 0; z < 2; z++) {
            world.SetVoxel(Vector3(x * VOXEL_SIZE, 5 * VOXEL_SIZE, z * VOXEL_SIZE), Voxel(wood_id));
        }
    }

    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 1);

    auto classification = VoxelStability::ClassifySupport(world, clusters[0]);

    // All should be unsupported
    for (const auto& [pos, type] : classification) {
        EXPECT_EQ(type, SupportType::UNSUPPORTED);
    }
}

TEST_F(VoxelStabilityTest, HasSupportPath_Connected) {
    // Tower
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * VOXEL_SIZE, 0), Voxel(wood_id));
    }

    // Top voxel should have support path to ground
    EXPECT_TRUE(VoxelStability::HasSupportPath(world, Vector3(0, 4 * VOXEL_SIZE, 0)));
}

TEST_F(VoxelStabilityTest, HasSupportPath_Disconnected) {
    // Just a floating voxel
    world.SetVoxel(Vector3(0, 5, 0), Voxel(wood_id));

    // Should NOT have support path
    EXPECT_FALSE(VoxelStability::HasSupportPath(world, Vector3(0, 5, 0)));
}

// ===== Stability Analysis Tests =====

TEST_F(VoxelStabilityTest, StableStructure_CenteredCube) {
    // 3x3x3 cube on ground - very stable
    for (int x = -1; x <= 1; x++) {
        for (int y = 0; y < 3; y++) {
            for (int z = -1; z <= 1; z++) {
                world.SetVoxel(Vector3(x * VOXEL_SIZE, y * VOXEL_SIZE, z * VOXEL_SIZE), Voxel(wood_id));
            }
        }
    }

    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 1);
    VoxelClustering::CalculateClusterProperties(world, clusters[0]);

    auto info = VoxelStability::AnalyzeStability(world, clusters[0]);

    EXPECT_TRUE(info.is_grounded);
    EXPECT_TRUE(info.is_stable);
    EXPECT_GT(info.stability_score, 0.7f);  // High stability
    EXPECT_LT(info.tip_risk, 0.3f);         // Low tip risk
    EXPECT_GT(info.num_support_voxels, 0);
    EXPECT_EQ(info.num_unsupported_voxels, 0);
}

TEST_F(VoxelStabilityTest, UnstableStructure_Floating) {
    // Floating structure
    world.SetVoxel(Vector3(0, 5, 0), Voxel(wood_id));

    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 1);
    VoxelClustering::CalculateClusterProperties(world, clusters[0]);

    auto info = VoxelStability::AnalyzeStability(world, clusters[0]);

    EXPECT_FALSE(info.is_grounded);
    EXPECT_FALSE(info.is_stable);
    EXPECT_EQ(info.stability_score, 0.0f);
    EXPECT_EQ(info.tip_risk, 1.0f);
    EXPECT_EQ(info.num_support_voxels, 0);
}

TEST_F(VoxelStabilityTest, UnstableStructure_TallThinTower) {
    // Very tall, thin tower - unstable
    for (int y = 0; y < 10; y++) {
        world.SetVoxel(Vector3(0, y * VOXEL_SIZE, 0), Voxel(wood_id));
    }

    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 1);
    VoxelClustering::CalculateClusterProperties(world, clusters[0]);

    auto info = VoxelStability::AnalyzeStability(world, clusters[0]);

    EXPECT_TRUE(info.is_grounded);
    EXPECT_TRUE(info.is_stable);  // COM is still over support (it's vertical)
    EXPECT_EQ(info.num_support_voxels, 1);
    EXPECT_GT(info.tip_risk, 0.5f);  // High tip risk (narrow base)
}

TEST_F(VoxelStabilityTest, TippingStructure_OffCenterMass) {
    // L-shape with heavy overhang
    // Base (1 voxel)
    world.SetVoxel(Vector3(0, 0, 0), Voxel(wood_id));

    // Long horizontal arm extending to one side
    for (int x = 1; x <= 5; x++) {
        world.SetVoxel(Vector3(x * VOXEL_SIZE, 0, 0), Voxel(wood_id));
    }

    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 1);
    VoxelClustering::CalculateClusterProperties(world, clusters[0]);

    auto info = VoxelStability::AnalyzeStability(world, clusters[0]);

    EXPECT_TRUE(info.is_grounded);
    EXPECT_FALSE(info.is_stable);  // COM not over support
    EXPECT_LT(info.stability_score, 0.5f);  // Low stability
}

// ===== Center of Mass Support Tests =====

TEST_F(VoxelStabilityTest, COMSupported_SingleVoxel) {
    world.SetVoxel(Vector3(0, 0, 0), Voxel(wood_id));

    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 1);
    VoxelClustering::CalculateClusterProperties(world, clusters[0]);

    auto ground_voxels = VoxelStability::FindGroundSupportVoxels(world, clusters[0]);
    bool supported = VoxelStability::IsCenterOfMassSupported(clusters[0], ground_voxels);

    EXPECT_TRUE(supported);
}

TEST_F(VoxelStabilityTest, COMSupported_VerticalTower) {
    // Vertical tower - COM should be over base
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * VOXEL_SIZE, 0), Voxel(wood_id));
    }

    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 1);
    VoxelClustering::CalculateClusterProperties(world, clusters[0]);

    auto ground_voxels = VoxelStability::FindGroundSupportVoxels(world, clusters[0]);
    bool supported = VoxelStability::IsCenterOfMassSupported(clusters[0], ground_voxels);

    EXPECT_TRUE(supported);
}

TEST_F(VoxelStabilityTest, COMNotSupported_Cantilever) {
    // Cantilever beam
    world.SetVoxel(Vector3(0, 0, 0), Voxel(wood_id));  // Support
    for (int x = 1; x <= 5; x++) {
        world.SetVoxel(Vector3(x * VOXEL_SIZE, 0, 0), Voxel(wood_id));  // Overhang
    }

    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 1);
    VoxelClustering::CalculateClusterProperties(world, clusters[0]);

    auto ground_voxels = VoxelStability::FindGroundSupportVoxels(world, clusters[0]);
    bool supported = VoxelStability::IsCenterOfMassSupported(clusters[0], ground_voxels);

    EXPECT_FALSE(supported);  // COM is way off to the side
}

// ===== Support Metrics Tests =====

TEST_F(VoxelStabilityTest, SupportCenter_SingleVoxel) {
    world.SetVoxel(Vector3(1 * VOXEL_SIZE, 0, 2 * VOXEL_SIZE), Voxel(wood_id));

    auto clusters = VoxelClustering::FindAllClusters(world);
    auto ground_voxels = VoxelStability::FindGroundSupportVoxels(world, clusters[0]);

    Vector3 center = VoxelStability::CalculateSupportCenter(ground_voxels);
    EXPECT_EQ(center, Vector3(1 * VOXEL_SIZE, 0, 2 * VOXEL_SIZE));
}

TEST_F(VoxelStabilityTest, SupportCenter_MultipleVoxels) {
    // 2x2 support base
    world.SetVoxel(Vector3(0, 0, 0), Voxel(wood_id));
    world.SetVoxel(Vector3(VOXEL_SIZE, 0, 0), Voxel(wood_id));
    world.SetVoxel(Vector3(0, 0, VOXEL_SIZE), Voxel(wood_id));
    world.SetVoxel(Vector3(VOXEL_SIZE, 0, VOXEL_SIZE), Voxel(wood_id));

    auto clusters = VoxelClustering::FindAllClusters(world);
    auto ground_voxels = VoxelStability::FindGroundSupportVoxels(world, clusters[0]);

    Vector3 center = VoxelStability::CalculateSupportCenter(ground_voxels);
    EXPECT_NEAR(center.x, VOXEL_SIZE * 0.5f, 0.001f);
    EXPECT_NEAR(center.z, VOXEL_SIZE * 0.5f, 0.001f);
}

TEST_F(VoxelStabilityTest, SupportArea_SingleVoxel) {
    world.SetVoxel(Vector3(0, 0, 0), Voxel(wood_id));

    auto clusters = VoxelClustering::FindAllClusters(world);
    auto ground_voxels = VoxelStability::FindGroundSupportVoxels(world, clusters[0]);

    float area = VoxelStability::CalculateSupportArea(ground_voxels);
    EXPECT_FLOAT_EQ(area, VOXEL_SIZE * VOXEL_SIZE);
}

TEST_F(VoxelStabilityTest, SupportArea_Square) {
    // 3x3 square
    for (int x = 0; x < 3; x++) {
        for (int z = 0; z < 3; z++) {
            world.SetVoxel(Vector3(x * VOXEL_SIZE, 0, z * VOXEL_SIZE), Voxel(wood_id));
        }
    }

    auto clusters = VoxelClustering::FindAllClusters(world);
    auto ground_voxels = VoxelStability::FindGroundSupportVoxels(world, clusters[0]);

    float area = VoxelStability::CalculateSupportArea(ground_voxels);
    // Should be roughly (3 * VOXEL_SIZE)^2
    float expected = 3.0f * VOXEL_SIZE * 3.0f * VOXEL_SIZE;
    EXPECT_NEAR(area, expected, 0.001f);
}

// ===== Edge Cases =====

TEST_F(VoxelStabilityTest, EmptyCluster) {
    VoxelCluster empty_cluster;
    empty_cluster.voxel_positions.clear();

    auto ground_voxels = VoxelStability::FindGroundSupportVoxels(world, empty_cluster);
    EXPECT_EQ(ground_voxels.size(), 0);

    auto supported = VoxelStability::FindSupportedVoxels(world, empty_cluster);
    EXPECT_EQ(supported.size(), 0);
}

TEST_F(VoxelStabilityTest, NegativeGroundLevel) {
    // Set ground level at y = -VOXEL_SIZE
    world.SetVoxel(Vector3(0, -VOXEL_SIZE, 0), Voxel(wood_id));
    world.SetVoxel(Vector3(0, 0, 0), Voxel(wood_id));

    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 1);

    auto ground_voxels = VoxelStability::FindGroundSupportVoxels(world, clusters[0], -VOXEL_SIZE);
    EXPECT_EQ(ground_voxels.size(), 1);
    EXPECT_TRUE(ground_voxels.count(Vector3(0, -VOXEL_SIZE, 0)) > 0);

    auto info = VoxelStability::AnalyzeStability(world, clusters[0], -VOXEL_SIZE);
    EXPECT_TRUE(info.is_grounded);
}

TEST_F(VoxelStabilityTest, ComplexStructure_Bridge) {
    // Bridge with two pillars and deck
    // Left pillar
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * VOXEL_SIZE, 0), Voxel(wood_id));
    }
    // Right pillar
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(10 * VOXEL_SIZE, y * VOXEL_SIZE, 0), Voxel(wood_id));
    }
    // Deck connecting them
    for (int x = 0; x <= 10; x++) {
        world.SetVoxel(Vector3(x * VOXEL_SIZE, 4 * VOXEL_SIZE, 0), Voxel(wood_id));
    }

    auto clusters = VoxelClustering::FindAllClusters(world);
    ASSERT_EQ(clusters.size(), 1);
    VoxelClustering::CalculateClusterProperties(world, clusters[0]);

    auto info = VoxelStability::AnalyzeStability(world, clusters[0]);

    EXPECT_TRUE(info.is_grounded);
    EXPECT_TRUE(info.is_stable);
    EXPECT_EQ(info.num_support_voxels, 2);  // Two ground voxels
    EXPECT_EQ(info.num_unsupported_voxels, 0);  // All connected
    EXPECT_GT(info.stability_score, 0.3f);  // Reasonably stable
}
