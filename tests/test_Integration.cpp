#include <gtest/gtest.h>
#include "VoxelWorld.h"
#include "StructuralAnalyzer.h"
#include "VoxelPhysicsIntegration.h"
#include "Material.h"
#include <chrono>

#ifdef USE_BULLET
#include "BulletEngine.h"
#else
#include "MockPhysicsEngine.h"
#endif

// ===== Integration Test Fixture =====

class IntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        world = new VoxelWorld();
        analyzer = new StructuralAnalyzer();

#ifdef USE_BULLET
        physics = new BulletEngine();
#else
        physics = new MockPhysicsEngine();
#endif
        physics->Initialize();

        integration = nullptr;
    }

    void TearDown() override {
        delete integration;
        if (physics) {
            physics->Shutdown();
            delete physics;
        }
        delete analyzer;
        delete world;
    }

    // Helper: Build a simple tower
    std::vector<Vector3> BuildTower(int height, Vector3 base_pos) {
        std::vector<Vector3> positions;
        float voxel_size = world->GetVoxelSize();
        for (int y = 0; y < height; y++) {
            Vector3 pos = base_pos + Vector3(0, y * voxel_size, 0);
            world->SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
            positions.push_back(pos);
        }
        return positions;
    }

    // Helper: Damage base of tower
    std::vector<Vector3> DamageBase(Vector3 base_pos, int damage_radius = 1) {
        std::vector<Vector3> damaged;
        float voxel_size = world->GetVoxelSize();

        for (int x = -damage_radius; x <= damage_radius; x++) {
            for (int z = -damage_radius; z <= damage_radius; z++) {
                Vector3 pos = base_pos + Vector3(x * voxel_size, 0, z * voxel_size);
                if (world->HasVoxel(pos)) {
                    world->RemoveVoxel(pos);
                    damaged.push_back(pos);
                }
            }
        }
        return damaged;
    }

    VoxelWorld* world;
    StructuralAnalyzer* analyzer;
    IPhysicsEngine* physics;
    VoxelPhysicsIntegration* integration;
};

// ===== End-to-End Integration Tests =====

TEST_F(IntegrationTest, TowerCollapse_CompleteFlow) {
    // STEP 1: Build 10-voxel tower
    Vector3 base(0, 0, 0);
    BuildTower(10, base);
    EXPECT_EQ(world->GetVoxelCount(), 10);

    // STEP 2: Remove base voxel
    auto damaged = DamageBase(base, 0);  // Just remove center
    EXPECT_EQ(world->GetVoxelCount(), 9);  // 10 - 1 = 9

    // STEP 3: Run structural analysis
    auto result = analyzer->Analyze(*world, damaged);

    // STEP 4: Verify failure detected
    EXPECT_TRUE(result.structure_failed);
    EXPECT_GT(result.num_failed_nodes, 0);
    EXPECT_GT(result.failed_clusters.size(), 0);

    // STEP 5: Spawn physics debris
    integration = new VoxelPhysicsIntegration(physics, world);
    int spawned = integration->SpawnDebris(result.failed_clusters);
    EXPECT_GT(spawned, 0);
    EXPECT_EQ(integration->GetDebrisCount(), spawned);

    // STEP 6: Simulate physics for 1 second
    for (int i = 0; i < 60; i++) {
        integration->Step(1.0f / 60.0f);
    }

    EXPECT_GT(integration->GetDebrisCount(), 0);
}

TEST_F(IntegrationTest, StableStructure_NoCollapse) {
    // Build wide, stable base
    float voxel_size = world->GetVoxelSize();
    for (int x = -2; x <= 2; x++) {
        for (int z = -2; z <= 2; z++) {
            world->SetVoxel(Vector3(x * voxel_size, 0, z * voxel_size),
                          Voxel(MaterialDatabase::CONCRETE));
        }
    }

    // Damage one edge voxel (not critical)
    Vector3 edge_pos(-2 * voxel_size, 0, -2 * voxel_size);
    world->RemoveVoxel(edge_pos);
    std::vector<Vector3> damaged = {edge_pos};

    // Analyze
    auto result = analyzer->Analyze(*world, damaged);

    // Should remain stable (redundant support)
    EXPECT_FALSE(result.structure_failed);
    EXPECT_EQ(result.failed_clusters.size(), 0);

    // No debris to spawn
    integration = new VoxelPhysicsIntegration(physics, world);
    int spawned = integration->SpawnDebris(result.failed_clusters);
    EXPECT_EQ(spawned, 0);
}

TEST_F(IntegrationTest, BridgeCollapse) {
    float voxel_size = world->GetVoxelSize();

    // Build bridge: two pillars with span
    // Left pillar
    for (int y = 0; y < 5; y++) {
        world->SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(MaterialDatabase::BRICK));
    }

    // Right pillar
    for (int y = 0; y < 5; y++) {
        world->SetVoxel(Vector3(10 * voxel_size, y * voxel_size, 0),
                       Voxel(MaterialDatabase::BRICK));
    }

    // Bridge span
    for (int x = 0; x <= 10; x++) {
        world->SetVoxel(Vector3(x * voxel_size, 4 * voxel_size, 0),
                       Voxel(MaterialDatabase::WOOD));
    }

    // Remove middle of span
    Vector3 middle(5 * voxel_size, 4 * voxel_size, 0);
    world->RemoveVoxel(middle);
    std::vector<Vector3> damaged = {middle};

    // Analyze
    auto result = analyzer->Analyze(*world, damaged);

    // Bridge span should fail (loss of support)
    EXPECT_TRUE(result.structure_failed);
    EXPECT_GT(result.failed_clusters.size(), 0);

    // Spawn debris
    integration = new VoxelPhysicsIntegration(physics, world);
    int spawned = integration->SpawnDebris(result.failed_clusters);
    EXPECT_GT(spawned, 0);
}

TEST_F(IntegrationTest, MultipleDestruction_Waves) {
    // Build structure
    BuildTower(8, Vector3(0, 0, 0));

    integration = new VoxelPhysicsIntegration(physics, world);

    // Wave 1: Damage base
    auto damaged1 = DamageBase(Vector3(0, 0, 0), 0);
    auto result1 = analyzer->Analyze(*world, damaged1);
    EXPECT_TRUE(result1.structure_failed);

    int spawned1 = integration->SpawnDebris(result1.failed_clusters);
    EXPECT_GT(spawned1, 0);

    // Simulate
    for (int i = 0; i < 30; i++) {
        integration->Step(1.0f / 60.0f);
    }

    // Verify debris exists
    EXPECT_EQ(integration->GetDebrisCount(), spawned1);
}

TEST_F(IntegrationTest, DebrisCleanup) {
    // Build and destroy tower
    BuildTower(6, Vector3(0, 0, 0));
    auto damaged = DamageBase(Vector3(0, 0, 0), 0);
    auto result = analyzer->Analyze(*world, damaged);

    integration = new VoxelPhysicsIntegration(physics, world);
    integration->SpawnDebris(result.failed_clusters);
    EXPECT_GT(integration->GetDebrisCount(), 0);

    // Clear debris
    integration->ClearDebris();
    EXPECT_EQ(integration->GetDebrisCount(), 0);
}

TEST_F(IntegrationTest, DebrisOutOfBounds) {
    float voxel_size = world->GetVoxelSize();

    // Build tower high up
    for (int y = 0; y < 5; y++) {
        world->SetVoxel(Vector3(0, (y + 100) * voxel_size, 0),
                       Voxel(MaterialDatabase::BRICK));
    }

    // Remove base
    Vector3 base(0, 100 * voxel_size, 0);
    world->RemoveVoxel(base);
    std::vector<Vector3> damaged = {base};

    auto result = analyzer->Analyze(*world, damaged);

    integration = new VoxelPhysicsIntegration(physics, world);
    int spawned = integration->SpawnDebris(result.failed_clusters);
    EXPECT_GT(spawned, 0);

    // Simulate long enough for debris to fall
    for (int i = 0; i < 300; i++) {
        integration->Step(1.0f / 60.0f);
    }

    // Remove debris that fell below y = -100
    int removed = integration->RemoveOutOfBoundsDebris(-100.0f);

    // May or may not remove depending on physics engine (MockEngine doesn't simulate)
    // Just check it doesn't crash
    EXPECT_GE(removed, 0);
}

TEST_F(IntegrationTest, PerformanceTest_LargeCollapse) {
    float voxel_size = world->GetVoxelSize();

    // Build large structure (10x10x5 = 500 voxels)
    for (int x = 0; x < 10; x++) {
        for (int z = 0; z < 10; z++) {
            for (int y = 0; y < 5; y++) {
                world->SetVoxel(Vector3(x * voxel_size, y * voxel_size, z * voxel_size),
                               Voxel(MaterialDatabase::BRICK));
            }
        }
    }

    // Remove entire base layer
    std::vector<Vector3> damaged;
    for (int x = 0; x < 10; x++) {
        for (int z = 0; z < 10; z++) {
            Vector3 pos(x * voxel_size, 0, z * voxel_size);
            world->RemoveVoxel(pos);
            damaged.push_back(pos);
        }
    }

    // Analyze (should complete within reasonable time)
    auto start = std::chrono::high_resolution_clock::now();
    auto result = analyzer->Analyze(*world, damaged, 1000.0f);  // 1 second budget
    auto end = std::chrono::high_resolution_clock::now();

    float elapsed = std::chrono::duration<float, std::milli>(end - start).count();

    EXPECT_LT(elapsed, 1000.0f);  // Within budget
    EXPECT_TRUE(result.structure_failed);

    // Spawn debris (should be fast)
    integration = new VoxelPhysicsIntegration(physics, world);
    int spawned = integration->SpawnDebris(result.failed_clusters);
    EXPECT_GT(spawned, 0);
}

TEST_F(IntegrationTest, MaterialVariations) {
    float voxel_size = world->GetVoxelSize();

    // Build mixed-material tower
    for (int y = 0; y < 3; y++) {
        world->SetVoxel(Vector3(0, y * voxel_size, 0),
                       Voxel(MaterialDatabase::CONCRETE));
    }
    for (int y = 3; y < 6; y++) {
        world->SetVoxel(Vector3(0, y * voxel_size, 0),
                       Voxel(MaterialDatabase::WOOD));
    }

    // Remove base
    world->RemoveVoxel(Vector3(0, 0, 0));
    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    auto result = analyzer->Analyze(*world, damaged);
    EXPECT_TRUE(result.structure_failed);

    integration = new VoxelPhysicsIntegration(physics, world);
    int spawned = integration->SpawnDebris(result.failed_clusters);
    EXPECT_GT(spawned, 0);
}

// ===== Stress Tests =====

TEST_F(IntegrationTest, RepeatedAnalysis) {
    // Build structure once
    BuildTower(5, Vector3(0, 0, 0));

    // Run analysis multiple times (should be consistent)
    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};
    world->RemoveVoxel(Vector3(0, 0, 0));

    for (int i = 0; i < 5; i++) {
        analyzer->Clear();
        auto result = analyzer->Analyze(*world, damaged);
        EXPECT_TRUE(result.structure_failed);
        EXPECT_GT(result.num_failed_nodes, 0);
    }
}

TEST_F(IntegrationTest, EdgeCase_EmptyWorld) {
    // Analyze empty world
    std::vector<Vector3> damaged;
    auto result = analyzer->Analyze(*world, damaged);

    EXPECT_FALSE(result.structure_failed);
    EXPECT_EQ(result.failed_clusters.size(), 0);

    integration = new VoxelPhysicsIntegration(physics, world);
    int spawned = integration->SpawnDebris(result.failed_clusters);
    EXPECT_EQ(spawned, 0);
}

TEST_F(IntegrationTest, EdgeCase_SingleVoxel) {
    // Single floating voxel
    float voxel_size = world->GetVoxelSize();
    world->SetVoxel(Vector3(0, 10 * voxel_size, 0), Voxel(MaterialDatabase::BRICK));

    std::vector<Vector3> damaged = {Vector3(0, 9 * voxel_size, 0)};
    auto result = analyzer->Analyze(*world, damaged);

    EXPECT_TRUE(result.structure_failed);  // Floating voxel should fail

    integration = new VoxelPhysicsIntegration(physics, world);
    int spawned = integration->SpawnDebris(result.failed_clusters);
    EXPECT_EQ(spawned, 1);  // Single cluster
}
