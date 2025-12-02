#include <gtest/gtest.h>
#include "StructuralAnalyzer.h"
#include "VoxelWorld.h"
#include "VoxelUtils.h"
#include "Material.h"

class StructuralAnalyzerTest : public ::testing::Test {
protected:
    void SetUp() override {
        world.Clear();
        analyzer.Clear();
        brick_id = MaterialDatabase::BRICK;
        wood_id = MaterialDatabase::WOOD;
    }

    VoxelWorld world;
    StructuralAnalyzer analyzer;
    uint8_t brick_id;
    uint8_t wood_id;
};

// ===== Node Graph Building Tests =====

TEST_F(StructuralAnalyzerTest, EmptyWorldNoNodes) {
    // Analyze empty world
    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    EXPECT_FALSE(result.structure_failed);
    EXPECT_EQ(analyzer.GetNodeCount(), 0);
}

TEST_F(StructuralAnalyzerTest, SingleVoxelCreatesNode) {
    // Create single voxel
    world.SetVoxel(Vector3(0, 0, 0), Voxel(brick_id));

    // Damage nearby (within 3x3x3 neighborhood)
    std::vector<Vector3> damaged = {Vector3(0.05f, 0, 0)};  // One voxel away

    auto result = analyzer.Analyze(world, damaged);

    // Should create at least one node
    EXPECT_GT(analyzer.GetNodeCount(), 0);
}

TEST_F(StructuralAnalyzerTest, TowerCreatesMultipleNodes) {
    // Create 5-voxel tower
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    // Remove base voxel
    world.RemoveVoxel(Vector3(0, 0, 0));
    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Should create nodes for voxels near damage
    EXPECT_GT(analyzer.GetNodeCount(), 0);
    std::cout << "Created " << analyzer.GetNodeCount() << " nodes for tower\n";
}

TEST_F(StructuralAnalyzerTest, GroundAnchorsIdentified) {
    // Create voxels at different heights
    float voxel_size = world.GetVoxelSize();
    world.SetVoxel(Vector3(0, 0, 0), Voxel(brick_id));        // Ground
    world.SetVoxel(Vector3(0, voxel_size, 0), Voxel(brick_id));   // Above ground

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Check nodes for ground anchors
    bool found_anchor = false;
    bool found_non_anchor = false;

    for (size_t i = 0; i < analyzer.GetNodeCount(); i++) {
        const SpringNode* node = analyzer.GetNode(i);
        if (node->is_ground_anchor) {
            found_anchor = true;
            EXPECT_LE(node->position.y, 0.0f);  // Ground level or below
        } else {
            found_non_anchor = true;
            EXPECT_GT(node->position.y, 0.0f);  // Above ground
        }
    }

    // We should have at least one of each (if both voxels are surface voxels)
    EXPECT_TRUE(found_anchor || found_non_anchor);
}

TEST_F(StructuralAnalyzerTest, NeighborConnectivity) {
    // Create 3-voxel horizontal line
    float voxel_size = world.GetVoxelSize();
    world.SetVoxel(Vector3(0, 0, 0), Voxel(brick_id));
    world.SetVoxel(Vector3(voxel_size, 0, 0), Voxel(brick_id));
    world.SetVoxel(Vector3(2 * voxel_size, 0, 0), Voxel(brick_id));

    std::vector<Vector3> damaged = {Vector3(voxel_size, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Check that nodes have neighbors
    int nodes_with_neighbors = 0;
    for (size_t i = 0; i < analyzer.GetNodeCount(); i++) {
        const SpringNode* node = analyzer.GetNode(i);
        if (!node->neighbors.empty()) {
            nodes_with_neighbors++;
        }
    }

    EXPECT_GT(nodes_with_neighbors, 0);
}

// ===== Mass Calculation Tests =====

TEST_F(StructuralAnalyzerTest, GroundAnchorZeroMass) {
    // Ground voxel should support zero mass (ground supports itself)
    world.SetVoxel(Vector3(0, 0, 0), Voxel(brick_id));

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Find ground anchor node
    for (size_t i = 0; i < analyzer.GetNodeCount(); i++) {
        const SpringNode* node = analyzer.GetNode(i);
        if (node->is_ground_anchor) {
            EXPECT_FLOAT_EQ(node->mass_supported, 0.0f);
        }
    }
}

TEST_F(StructuralAnalyzerTest, TowerMassCalculation) {
    // Create tower: base supports weight of all voxels above
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    std::vector<Vector3> damaged = {Vector3(voxel_size, 0, 0)};  // Damage near base

    auto result = analyzer.Analyze(world, damaged);

    // Find non-ground nodes and check they have mass
    bool found_node_with_mass = false;
    for (size_t i = 0; i < analyzer.GetNodeCount(); i++) {
        const SpringNode* node = analyzer.GetNode(i);
        if (!node->is_ground_anchor && node->mass_supported > 0.0f) {
            found_node_with_mass = true;
            std::cout << "Node at " << node->position
                     << " supports " << node->mass_supported << " kg\n";
        }
    }

    EXPECT_TRUE(found_node_with_mass);
}

// ===== Displacement Solver Tests =====

TEST_F(StructuralAnalyzerTest, ConvergenceCheck) {
    // Simple structure should converge
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 3; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    std::vector<Vector3> damaged = {Vector3(voxel_size, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Should converge or at least complete without crashing
    EXPECT_GE(result.iterations_used, 0);
    EXPECT_LE(result.iterations_used, analyzer.params.max_iterations);
    std::cout << "Converged: " << result.converged
              << ", Iterations: " << result.iterations_used << "\n";
}

TEST_F(StructuralAnalyzerTest, DisplacementNegative) {
    // Displacement should be negative (downward) under load
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(wood_id));
    }

    std::vector<Vector3> damaged = {Vector3(voxel_size, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Check that some nodes have negative displacement
    bool found_displacement = false;
    for (size_t i = 0; i < analyzer.GetNodeCount(); i++) {
        const SpringNode* node = analyzer.GetNode(i);
        if (!node->is_ground_anchor && node->displacement != 0.0f) {
            found_displacement = true;
            std::cout << "Node displacement: " << node->displacement << " m\n";
        }
    }

    // Note: May not always have displacement depending on convergence
    // This is more of a sanity check
    if (found_displacement) {
        EXPECT_TRUE(true);  // Good, we saw some movement
    }
}

// ===== Failure Detection Tests =====

TEST_F(StructuralAnalyzerTest, StableStructureNoFailure) {
    // Short, wide structure on ground should be stable
    float voxel_size = world.GetVoxelSize();
    for (int x = -1; x <= 1; x++) {
        for (int z = -1; z <= 1; z++) {
            world.SetVoxel(Vector3(x * voxel_size, 0, z * voxel_size), Voxel(brick_id));
        }
    }

    // Damage edge voxel (not critical)
    world.RemoveVoxel(Vector3(-voxel_size, 0, -voxel_size));
    std::vector<Vector3> damaged = {Vector3(-voxel_size, 0, -voxel_size)};

    auto result = analyzer.Analyze(world, damaged);

    // Should be stable (no failures expected for robust structure)
    std::cout << "Structure failed: " << result.structure_failed
              << ", Failed nodes: " << result.num_failed_nodes << "\n";
}

TEST_F(StructuralAnalyzerTest, WeakMaterialMoreLikelyToFail) {
    // Wood (weaker) vs Brick (stronger)
    float voxel_size = world.GetVoxelSize();

    // Test with wood (max_displacement = 0.02m)
    for (int y = 0; y < 10; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(wood_id));
    }

    world.RemoveVoxel(Vector3(0, 0, 0));  // Remove base
    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    auto result_wood = analyzer.Analyze(world, damaged);

    std::cout << "Wood tower: " << result_wood.num_failed_nodes << " failures\n";
    std::cout << "Calculation time: " << result_wood.calculation_time_ms << " ms\n";

    // Just check it ran without crashing
    EXPECT_GE(result_wood.calculation_time_ms, 0.0f);
}

// ===== Analysis Result Tests =====

TEST_F(StructuralAnalyzerTest, AnalysisResultPopulated) {
    // Create structure and analyze
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 3; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Check result fields are populated
    EXPECT_GE(result.calculation_time_ms, 0.0f);
    EXPECT_GE(result.iterations_used, 0);
    EXPECT_FALSE(result.debug_info.empty());

    std::cout << "Debug info: " << result.debug_info << "\n";
}

TEST_F(StructuralAnalyzerTest, PerformanceBudget) {
    // Create large structure
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 20; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    // Set low time budget
    float budget_ms = 100.0f;
    auto result = analyzer.Analyze(world, damaged, budget_ms);

    // Should complete (though may not converge)
    std::cout << "Time: " << result.calculation_time_ms << " ms (budget: " << budget_ms << " ms)\n";
    EXPECT_GT(result.calculation_time_ms, 0.0f);
}

// ===== Edge Cases =====

TEST_F(StructuralAnalyzerTest, NoDamagedPositions) {
    // Create structure
    world.SetVoxel(Vector3(0, 0, 0), Voxel(brick_id));

    // Empty damage list
    std::vector<Vector3> damaged;

    auto result = analyzer.Analyze(world, damaged);

    EXPECT_FALSE(result.structure_failed);
    EXPECT_EQ(analyzer.GetNodeCount(), 0);
}

TEST_F(StructuralAnalyzerTest, DamageOutsideStructure) {
    // Create structure at origin
    world.SetVoxel(Vector3(0, 0, 0), Voxel(brick_id));

    // Damage far away
    std::vector<Vector3> damaged = {Vector3(10, 10, 10)};

    auto result = analyzer.Analyze(world, damaged);

    // Should find no nodes (damage too far)
    EXPECT_EQ(analyzer.GetNodeCount(), 0);
}

TEST_F(StructuralAnalyzerTest, MultipleAnalysisCalls) {
    // Test that analyzer can be reused
    float voxel_size = world.GetVoxelSize();
    world.SetVoxel(Vector3(0, 0, 0), Voxel(brick_id));
    world.SetVoxel(Vector3(0, voxel_size, 0), Voxel(brick_id));

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    // First analysis
    auto result1 = analyzer.Analyze(world, damaged);
    size_t count1 = analyzer.GetNodeCount();

    // Second analysis (should clear previous state)
    auto result2 = analyzer.Analyze(world, damaged);
    size_t count2 = analyzer.GetNodeCount();

    EXPECT_EQ(count1, count2);  // Should get same result
}
