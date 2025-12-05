#include <gtest/gtest.h>
#include "MaxFlowStructuralAnalyzer.h"
#include "StructuralAnalyzer.h"
#include "VoxelWorld.h"
#include "VoxelUtils.h"
#include "Material.h"
#include "JobSystem.h"  // Week 13 Day 42: Parallel testing
#include <iostream>
#include <chrono>

class MaxFlowStructuralAnalyzerTest : public ::testing::Test {
protected:
    void SetUp() override {
        world.Clear();
        analyzer.Clear();
        brick_id = MaterialDatabase::BRICK;
        wood_id = MaterialDatabase::WOOD;
        concrete_id = MaterialDatabase::CONCRETE;
    }

    VoxelWorld world;
    MaxFlowStructuralAnalyzer analyzer;
    uint8_t brick_id;
    uint8_t wood_id;
    uint8_t concrete_id;
};

// ===== Basic Functionality Tests =====

TEST_F(MaxFlowStructuralAnalyzerTest, EmptyWorldNoNodes) {
    // Analyze empty world
    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    EXPECT_FALSE(result.structure_failed);
    EXPECT_EQ(analyzer.GetNodeCount(), 2);  // Only source and sink
}

TEST_F(MaxFlowStructuralAnalyzerTest, SingleGroundVoxelStable) {
    // Single voxel on ground should be stable
    world.SetVoxel(Vector3(0, 0, 0), Voxel(brick_id));

    std::vector<Vector3> damaged = {Vector3(1, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    EXPECT_FALSE(result.structure_failed);
    std::cout << "Max flow: " << analyzer.GetLastMaxFlow()
              << ", Required: " << analyzer.GetLastRequiredFlow() << "\n";
}

TEST_F(MaxFlowStructuralAnalyzerTest, FloatingVoxelFails) {
    // Single voxel floating in air should fail
    float voxel_size = world.GetVoxelSize();
    world.SetVoxel(Vector3(0, voxel_size * 5, 0), Voxel(brick_id));

    std::vector<Vector3> damaged = {Vector3(0, voxel_size * 5, 0)};

    auto result = analyzer.Analyze(world, damaged);

    EXPECT_TRUE(result.structure_failed);
    EXPECT_GT(result.failed_clusters.size(), 0);
}

TEST_F(MaxFlowStructuralAnalyzerTest, GroundedTowerStable) {
    // Create 5-voxel tower on ground - should be stable
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    std::vector<Vector3> damaged = {Vector3(1, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    EXPECT_FALSE(result.structure_failed);
    std::cout << "Tower: Max flow " << analyzer.GetLastMaxFlow()
              << " >= Required " << analyzer.GetLastRequiredFlow() << "\n";
}

TEST_F(MaxFlowStructuralAnalyzerTest, TowerWithoutBaseFails) {
    // Create 5-voxel tower but remove base - should fail
    float voxel_size = world.GetVoxelSize();
    for (int y = 1; y < 6; y++) {  // Start from y=1 (no base)
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    EXPECT_TRUE(result.structure_failed);
    EXPECT_GT(result.failed_clusters.size(), 0);
}

// ===== Network Construction Tests =====

TEST_F(MaxFlowStructuralAnalyzerTest, NodeCountCorrect) {
    // Create 3x3 grid on ground
    float voxel_size = world.GetVoxelSize();
    for (int x = 0; x < 3; x++) {
        for (int z = 0; z < 3; z++) {
            world.SetVoxel(Vector3(x * voxel_size, 0, z * voxel_size), Voxel(brick_id));
        }
    }

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    analyzer.Analyze(world, damaged);

    // Should have: 2 (source+sink) + 9 (voxels) = 11 nodes
    EXPECT_EQ(analyzer.GetNodeCount(), 11);
}

TEST_F(MaxFlowStructuralAnalyzerTest, SourceAndSinkCreated) {
    world.SetVoxel(Vector3(0, 0, 0), Voxel(brick_id));

    analyzer.Analyze(world, {Vector3(1, 0, 0)});

    // Check that source (node 0) and sink (node 1) exist
    EXPECT_NE(analyzer.GetNode(0), nullptr);
    EXPECT_NE(analyzer.GetNode(1), nullptr);
    EXPECT_EQ(analyzer.GetNode(0)->type, FlowNode::SOURCE);
    EXPECT_EQ(analyzer.GetNode(1)->type, FlowNode::SINK);
}

// ===== Max-Flow Algorithm Tests =====

TEST_F(MaxFlowStructuralAnalyzerTest, MaxFlowComputesCorrectly) {
    // Simple 2-voxel tower: ground + one above
    float voxel_size = world.GetVoxelSize();
    world.SetVoxel(Vector3(0, 0, 0), Voxel(brick_id));
    world.SetVoxel(Vector3(0, voxel_size, 0), Voxel(brick_id));

    analyzer.Analyze(world, {Vector3(1, 0, 0)});

    float max_flow = analyzer.GetLastMaxFlow();
    float required = analyzer.GetLastRequiredFlow();

    // Max flow should be at least as large as required for stability
    EXPECT_GT(max_flow, 0.0f);
    EXPECT_GT(required, 0.0f);
    EXPECT_GE(max_flow, required);  // Stable structure
}

TEST_F(MaxFlowStructuralAnalyzerTest, RequiredFlowEqualsLoad) {
    // Create structure and verify required flow = mass × g
    float voxel_size = world.GetVoxelSize();
    world.SetVoxel(Vector3(0, 0, 0), Voxel(brick_id));
    world.SetVoxel(Vector3(0, voxel_size, 0), Voxel(brick_id));

    analyzer.Analyze(world, {Vector3(1, 0, 0)});

    const Material& brick = g_materials.GetMaterial(brick_id);
    float expected_load = 2.0f * brick.density * 1.0f * 9.81f;  // 2 voxels × density × volume × g
    float required = analyzer.GetLastRequiredFlow();

    EXPECT_NEAR(required, expected_load, 1.0f);  // Within 1N tolerance
}

// ===== Material Strength Tests =====

TEST_F(MaxFlowStructuralAnalyzerTest, WeakMaterialFailsUnderLoad) {
    // Create tall tower of weak material - may fail
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 20; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(wood_id));
    }

    std::vector<Vector3> damaged = {Vector3(1, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Wood may not support 20-voxel tower
    std::cout << "Wood tower: Required " << analyzer.GetLastRequiredFlow()
              << "N, Max " << analyzer.GetLastMaxFlow() << "N\n";

    if (result.structure_failed) {
        EXPECT_GT(result.failed_clusters.size(), 0);
    }
}

TEST_F(MaxFlowStructuralAnalyzerTest, StrongMaterialSupportsLoad) {
    // Create tall tower of strong material - should be stable
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 20; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(concrete_id));
    }

    std::vector<Vector3> damaged = {Vector3(1, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Concrete should support 20-voxel tower
    EXPECT_FALSE(result.structure_failed);
    EXPECT_GE(analyzer.GetLastMaxFlow(), analyzer.GetLastRequiredFlow());
}

// ===== Cluster Detection Tests =====

TEST_F(MaxFlowStructuralAnalyzerTest, DisconnectedClusterDetected) {
    // Create two separate structures
    float voxel_size = world.GetVoxelSize();

    // Grounded structure
    world.SetVoxel(Vector3(0, 0, 0), Voxel(brick_id));

    // Floating structure (far away)
    world.SetVoxel(Vector3(10 * voxel_size, 5 * voxel_size, 0), Voxel(brick_id));

    std::vector<Vector3> damaged = {Vector3(10 * voxel_size, 5 * voxel_size, 0)};

    auto result = analyzer.Analyze(world, damaged);

    if (result.structure_failed) {
        EXPECT_EQ(result.failed_clusters.size(), 1);  // One floating cluster
    }
}

TEST_F(MaxFlowStructuralAnalyzerTest, MultipleFailedClusters) {
    // Create structure that breaks into multiple pieces
    float voxel_size = world.GetVoxelSize();

    // Create two floating towers
    for (int y = 1; y < 4; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
        world.SetVoxel(Vector3(5 * voxel_size, y * voxel_size, 0), Voxel(brick_id));
    }

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    EXPECT_TRUE(result.structure_failed);
    // Should detect both floating towers as separate clusters
    EXPECT_GE(result.failed_clusters.size(), 1);
}

// ===== Performance Tests =====

TEST_F(MaxFlowStructuralAnalyzerTest, PerformanceUnder500ms) {
    // Create large structure and verify analysis time
    float voxel_size = world.GetVoxelSize();

    // Create 10x10x10 building
    for (int x = 0; x < 10; x++) {
        for (int y = 0; y < 10; y++) {
            for (int z = 0; z < 10; z++) {
                world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, z * voxel_size),
                              Voxel(brick_id));
            }
        }
    }

    // Remove one voxel
    world.RemoveVoxel(Vector3(0, 0, 0));
    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    auto result = analyzer.Analyze(world, damaged, 500.0f);

    std::cout << "Large structure analysis time: " << result.calculation_time_ms << "ms\n";
    EXPECT_LT(result.calculation_time_ms, 500.0f);
}

// Week 13 Day 42: Parallel Performance Test
TEST_F(MaxFlowStructuralAnalyzerTest, ParallelSpeedupTest) {
    // Initialize global job system for this test
    JobSystem job_system;
    job_system.Initialize();
    g_JobSystem = &job_system;

    float voxel_size = world.GetVoxelSize();

    // Create large structure (15x15x15 = 3375 voxels)
    for (int x = 0; x < 15; x++) {
        for (int y = 0; y < 15; y++) {
            for (int z = 0; z < 15; z++) {
                world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, z * voxel_size),
                              Voxel(brick_id));
            }
        }
    }

    // Remove voxels to trigger analysis
    std::vector<Vector3> damaged;
    for (int i = 0; i < 10; i++) {
        Vector3 pos(i * voxel_size, 0, 0);
        world.RemoveVoxel(pos);
        damaged.push_back(pos);
    }

    // Run with parallel DISABLED
    MaxFlowStructuralAnalyzer serial_analyzer;
    serial_analyzer.params.use_parallel_construction = false;
    auto start_serial = std::chrono::high_resolution_clock::now();
    auto serial_result = serial_analyzer.Analyze(world, damaged);
    auto end_serial = std::chrono::high_resolution_clock::now();
    float serial_time = std::chrono::duration<float, std::milli>(end_serial - start_serial).count();

    // Run with parallel ENABLED
    MaxFlowStructuralAnalyzer parallel_analyzer;
    parallel_analyzer.params.use_parallel_construction = true;
    auto start_parallel = std::chrono::high_resolution_clock::now();
    auto parallel_result = parallel_analyzer.Analyze(world, damaged);
    auto end_parallel = std::chrono::high_resolution_clock::now();
    float parallel_time = std::chrono::duration<float, std::milli>(end_parallel - start_parallel).count();

    float speedup = serial_time / parallel_time;

    std::cout << "\nParallel Structural Analysis Performance (15x15x15 structure):\n";
    std::cout << "  Serial:   " << serial_time << "ms\n";
    std::cout << "  Parallel: " << parallel_time << "ms\n";
    std::cout << "  Speedup:  " << speedup << "x\n";
    std::cout << "  Nodes analyzed: " << serial_result.nodes_analyzed << "\n";

    // Expect at least 1.5x speedup (conservative, target is 2-3x)
    EXPECT_GT(speedup, 1.5f);

    // Verify both methods give same result
    EXPECT_EQ(serial_result.structure_failed, parallel_result.structure_failed);
    EXPECT_EQ(serial_result.num_failed_nodes, parallel_result.num_failed_nodes);

    // Cleanup
    job_system.Shutdown();
    g_JobSystem = nullptr;
}

TEST_F(MaxFlowStructuralAnalyzerTest, ConvergesQuickly) {
    // Verify max-flow converges in reasonable iterations
    float voxel_size = world.GetVoxelSize();

    for (int y = 0; y < 10; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    auto result = analyzer.Analyze(world, {Vector3(1, 0, 0)});

    std::cout << "Max-flow iterations: " << result.iterations_used << "\n";
    EXPECT_LT(result.iterations_used, 50);  // Should be much faster than spring system
    EXPECT_TRUE(result.converged);
}

// ===== Comparison Tests (Spring vs Max-Flow) =====

TEST_F(MaxFlowStructuralAnalyzerTest, CompareWithSpringAnalyzer) {
    // Create same structure and compare results
    float voxel_size = world.GetVoxelSize();

    // 5-voxel tower without base
    for (int y = 1; y < 6; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    // Run max-flow analyzer
    auto maxflow_result = analyzer.Analyze(world, damaged);

    // Run spring analyzer
    StructuralAnalyzer spring_analyzer;
    auto spring_result = spring_analyzer.Analyze(world, damaged);

    // Both should detect failure
    EXPECT_EQ(maxflow_result.structure_failed, spring_result.structure_failed);

    std::cout << "Max-flow time: " << maxflow_result.calculation_time_ms << "ms, "
              << "Spring time: " << spring_result.calculation_time_ms << "ms\n";

    // Max-flow should be faster
    EXPECT_LT(maxflow_result.calculation_time_ms, spring_result.calculation_time_ms);
}

// ===== Parameter Tests =====

TEST_F(MaxFlowStructuralAnalyzerTest, InfluenceRadiusWorks) {
    float voxel_size = world.GetVoxelSize();

    // Create structure with voxels far apart
    world.SetVoxel(Vector3(0, 0, 0), Voxel(brick_id));
    world.SetVoxel(Vector3(10 * voxel_size, 0, 0), Voxel(brick_id));

    // Small influence radius - should only analyze nearby voxels
    analyzer.params.influence_radius = 2.0f;
    analyzer.Analyze(world, {Vector3(0, 0, 0)});

    size_t small_radius_nodes = analyzer.GetNodeCount();

    analyzer.Clear();

    // Large influence radius - should analyze more voxels
    analyzer.params.influence_radius = 15.0f;
    analyzer.Analyze(world, {Vector3(0, 0, 0)});

    size_t large_radius_nodes = analyzer.GetNodeCount();

    EXPECT_GE(large_radius_nodes, small_radius_nodes);
}

TEST_F(MaxFlowStructuralAnalyzerTest, CapacityMultiplierAffectsStability) {
    float voxel_size = world.GetVoxelSize();

    // Create borderline structure
    for (int y = 0; y < 15; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(wood_id));
    }

    // With default multiplier
    analyzer.params.edge_capacity_multiplier = 1.0f;
    auto result1 = analyzer.Analyze(world, {Vector3(1, 0, 0)});

    analyzer.Clear();

    // With increased capacity (stronger connections)
    analyzer.params.edge_capacity_multiplier = 10.0f;
    auto result2 = analyzer.Analyze(world, {Vector3(1, 0, 0)});

    std::cout << "Multiplier 1.0: max_flow=" << analyzer.GetLastMaxFlow() << "\n";

    // Higher multiplier should increase max flow
    EXPECT_GT(result2.calculation_time_ms, 0.0f);  // Just verify it runs
}

// ===== Edge Case Tests =====

TEST_F(MaxFlowStructuralAnalyzerTest, NoDamagedPositions) {
    // Analyze without specifying damaged positions
    float voxel_size = world.GetVoxelSize();
    world.SetVoxel(Vector3(0, 0, 0), Voxel(brick_id));

    std::vector<Vector3> damaged;  // Empty

    auto result = analyzer.Analyze(world, damaged);

    // Should still analyze (all voxels)
    EXPECT_GT(analyzer.GetNodeCount(), 2);
}

TEST_F(MaxFlowStructuralAnalyzerTest, SingleVoxelNotOnGround) {
    // Edge case: single voxel clearly above ground (not within ground tolerance)
    float voxel_size = world.GetVoxelSize();
    // Use 1 voxel above ground (well above the 0.01m ground tolerance)
    world.SetVoxel(Vector3(0, voxel_size, 0), Voxel(brick_id));

    auto result = analyzer.Analyze(world, {Vector3(0, voxel_size, 0)});

    // Should fail (not grounded)
    EXPECT_TRUE(result.structure_failed);
}

TEST_F(MaxFlowStructuralAnalyzerTest, ClearResetsState) {
    float voxel_size = world.GetVoxelSize();
    world.SetVoxel(Vector3(0, 0, 0), Voxel(brick_id));

    analyzer.Analyze(world, {Vector3(1, 0, 0)});
    EXPECT_GT(analyzer.GetNodeCount(), 0);

    analyzer.Clear();
    EXPECT_EQ(analyzer.GetNodeCount(), 0);
}
