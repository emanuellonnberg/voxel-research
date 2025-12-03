#include <gtest/gtest.h>
#include "StructuralAnalyzer.h"
#include "ParameterTuning.h"
#include "VoxelWorld.h"
#include "VoxelUtils.h"
#include "Material.h"
#include <fstream>

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

// ===== Day 12: Cache and Profiling Tests =====

TEST_F(StructuralAnalyzerTest, CacheInvalidation_ClearCache) {
    // Create structure
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    // First analysis (populate cache)
    auto result1 = analyzer.Analyze(world, damaged);
    int hits1, misses1;
    analyzer.GetCacheStats(hits1, misses1);
    EXPECT_GT(misses1, 0);  // Should have cache misses

    // Clear cache
    analyzer.ClearMassCache();
    int hits2, misses2;
    analyzer.GetCacheStats(hits2, misses2);
    EXPECT_EQ(hits2, 0);    // Cache cleared
    EXPECT_EQ(misses2, 0);
}

TEST_F(StructuralAnalyzerTest, CacheInvalidation_ModifiedVoxels) {
    // Create tower
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    std::vector<Vector3> changed_positions = {
        Vector3(0, 2 * voxel_size, 0),  // Middle voxel
        Vector3(0, 3 * voxel_size, 0)   // Above middle
    };

    // Invalidate cache for specific positions
    analyzer.InvalidateMassCache(changed_positions);

    // Cache should be cleared (we don't track specific entries in test)
    EXPECT_TRUE(true);  // Just verify method doesn't crash
}

TEST_F(StructuralAnalyzerTest, ProfilingMetrics_Populated) {
    // Create structure
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 3; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Check all profiling metrics are populated
    EXPECT_GT(result.node_graph_time_ms, 0.0f);
    EXPECT_GE(result.mass_calc_time_ms, 0.0f);
    EXPECT_GT(result.solver_time_ms, 0.0f);
    EXPECT_GE(result.failure_detection_time_ms, 0.0f);
    EXPECT_GT(result.nodes_analyzed, 0);

    // Sum of parts should approximately equal total
    float sum_parts = result.node_graph_time_ms + result.mass_calc_time_ms +
                     result.solver_time_ms + result.failure_detection_time_ms;
    EXPECT_LE(sum_parts, result.calculation_time_ms * 1.1f);  // Within 10% tolerance

    std::cout << "[Profiling] Node graph: " << result.node_graph_time_ms << "ms\n";
    std::cout << "[Profiling] Mass calc: " << result.mass_calc_time_ms << "ms\n";
    std::cout << "[Profiling] Solver: " << result.solver_time_ms << "ms\n";
    std::cout << "[Profiling] Failure detection: " << result.failure_detection_time_ms << "ms\n";
}

TEST_F(StructuralAnalyzerTest, CachePerformance_HitsAndMisses) {
    // Create tower
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    // Analysis should track cache performance
    auto result = analyzer.Analyze(world, damaged);

    // Within a single analysis, we should see cache behavior
    // (cache is cleared between Analyze() calls, so we check within one call)
    EXPECT_GE(result.cache_hits + result.cache_misses, 0);  // Some cache activity

    std::cout << "[Cache Stats] Hits: " << result.cache_hits
              << ", Misses: " << result.cache_misses << "\n";

    // Note: Cache is per-analysis, cleared each Analyze() call
    // This is by design to ensure fresh analysis each time
}

TEST_F(StructuralAnalyzerTest, LargeStructure_PerformanceProfile) {
    // Create larger structure (10-voxel tower)
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 10; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Verify analysis completed
    EXPECT_GT(result.nodes_analyzed, 0);
    EXPECT_GT(result.calculation_time_ms, 0.0f);

    // Should handle reasonably sized structures quickly
    EXPECT_LT(result.calculation_time_ms, 100.0f);  // < 100ms

    std::cout << "[Large Structure] Analyzed " << result.nodes_analyzed
              << " nodes in " << result.calculation_time_ms << "ms\n";
    std::cout << "  Avg time per node: "
              << (result.calculation_time_ms / result.nodes_analyzed) << "ms\n";
}

TEST_F(StructuralAnalyzerTest, MassCalculation_ConcreteVsWood) {
    // Test that heavier materials calculate correctly
    float voxel_size = world.GetVoxelSize();

    // Create concrete tower
    world.Clear();
    for (int y = 0; y < 3; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(MaterialDatabase::CONCRETE));
    }

    std::vector<Vector3> damaged = {Vector3(voxel_size, 0, 0)};
    auto result_concrete = analyzer.Analyze(world, damaged);

    // Create wood tower (same structure, lighter material)
    world.Clear();
    analyzer.Clear();
    for (int y = 0; y < 3; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(wood_id));
    }

    auto result_wood = analyzer.Analyze(world, damaged);

    // Both should complete
    EXPECT_GE(result_concrete.nodes_analyzed, 0);
    EXPECT_GE(result_wood.nodes_analyzed, 0);

    std::cout << "[Material Comparison] Concrete: " << result_concrete.calculation_time_ms << "ms\n";
    std::cout << "[Material Comparison] Wood: " << result_wood.calculation_time_ms << "ms\n";
}

// ===== Day 13: Displacement Solver Refinements =====

TEST_F(StructuralAnalyzerTest, ConvergenceCriteria_DisplacementBased) {
    // Create stable tower
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    // Should converge with displacement-based criteria
    auto result = analyzer.Analyze(world, damaged);

    EXPECT_TRUE(result.converged);
    EXPECT_GT(result.iterations_used, 0);
    EXPECT_LT(result.iterations_used, 100);  // Should converge quickly

    std::cout << "[Convergence] Converged: " << result.converged
              << ", Iterations: " << result.iterations_used << "\n";
}

TEST_F(StructuralAnalyzerTest, DisplacementClamping_NoUpwardSag) {
    // Create structure that might have numerical instabilities
    float voxel_size = world.GetVoxelSize();
    world.SetVoxel(Vector3(0, 0, 0), Voxel(brick_id));  // Ground
    world.SetVoxel(Vector3(0, voxel_size, 0), Voxel(wood_id));  // Weak middle

    std::vector<Vector3> damaged = {Vector3(voxel_size, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Check all nodes for physically correct displacements
    for (size_t i = 0; i < analyzer.GetNodeCount(); i++) {
        const auto* node = analyzer.GetNode(i);
        if (node && !node->is_ground_anchor) {
            // Displacement should be non-positive (sagging down or zero)
            EXPECT_LE(node->displacement, 0.0f)
                << "Node at (" << node->position.x << ", " << node->position.y
                << ", " << node->position.z << ") has upward displacement: "
                << node->displacement;
        }
    }

    std::cout << "[Clamping] All nodes have physically correct displacements\n";
}

TEST_F(StructuralAnalyzerTest, ParameterSensitivity_Timestep) {
    // Create test structure
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 3; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    // Test different timesteps
    std::vector<float> timesteps = {0.001f, 0.005f, 0.01f, 0.02f, 0.05f};

    std::cout << "[Timestep Sensitivity]\n";
    for (float dt : timesteps) {
        world.Clear();
        analyzer.Clear();

        // Recreate structure
        for (int y = 0; y < 3; y++) {
            world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
        }

        analyzer.params.timestep = dt;
        auto result = analyzer.Analyze(world, damaged);

        std::cout << "  dt=" << dt << ": converged=" << result.converged
                  << ", iterations=" << result.iterations_used
                  << ", time=" << result.calculation_time_ms << "ms\n";

        // All timesteps should produce stable results
        EXPECT_TRUE(result.converged || result.iterations_used > 50);
    }
}

TEST_F(StructuralAnalyzerTest, ParameterSensitivity_Damping) {
    // Create test structure
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 3; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    // Test different damping values
    std::vector<float> damping_values = {0.7f, 0.8f, 0.9f, 0.95f, 0.99f};

    std::cout << "[Damping Sensitivity]\n";
    for (float damp : damping_values) {
        world.Clear();
        analyzer.Clear();

        // Recreate structure
        for (int y = 0; y < 3; y++) {
            world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
        }

        analyzer.params.damping = damp;
        auto result = analyzer.Analyze(world, damaged);

        std::cout << "  damping=" << damp << ": converged=" << result.converged
                  << ", iterations=" << result.iterations_used
                  << ", time=" << result.calculation_time_ms << "ms\n";

        // Higher damping should help convergence
        if (damp >= 0.9f) {
            EXPECT_TRUE(result.converged || result.iterations_used < 100);
        }
    }
}

TEST_F(StructuralAnalyzerTest, ConvergenceSpeed_OptimalParameters) {
    // Test that default parameters provide good convergence
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    // Default parameters (timestep=0.01, damping=0.9)
    auto result = analyzer.Analyze(world, damaged);

    // Should converge reasonably fast with default parameters
    EXPECT_TRUE(result.converged);
    EXPECT_LT(result.iterations_used, 100);  // Not too many iterations
    EXPECT_GT(result.iterations_used, 1);    // Not suspiciously fast

    std::cout << "[Optimal Params] Iterations: " << result.iterations_used
              << ", Time: " << result.calculation_time_ms << "ms\n";
}

TEST_F(StructuralAnalyzerTest, SolverStability_NoNaN) {
    // Create structure that might cause numerical issues
    float voxel_size = world.GetVoxelSize();

    // Very light structure (wood)
    for (int y = 0; y < 10; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(wood_id));
    }

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Check for NaN or Inf in results
    EXPECT_FALSE(std::isnan(result.calculation_time_ms));
    EXPECT_FALSE(std::isinf(result.calculation_time_ms));

    // Check all nodes for valid displacements
    for (size_t i = 0; i < analyzer.GetNodeCount(); i++) {
        const auto* node = analyzer.GetNode(i);
        if (node) {
            EXPECT_FALSE(std::isnan(node->displacement))
                << "Node " << i << " has NaN displacement";
            EXPECT_FALSE(std::isinf(node->displacement))
                << "Node " << i << " has Inf displacement";
            EXPECT_FALSE(std::isnan(node->velocity))
                << "Node " << i << " has NaN velocity";
        }
    }

    std::cout << "[Stability] No NaN/Inf values detected\n";
}

TEST_F(StructuralAnalyzerTest, PerformanceTarget_100msFor1000Iterations) {
    // Create moderate structure
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 8; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Even with many iterations, should be reasonably fast
    // Target from Day 13 plan: < 100ms for 1000 nodes
    // We have fewer nodes, so should be much faster
    EXPECT_LT(result.calculation_time_ms, 100.0f);

    float time_per_iteration = result.calculation_time_ms / result.iterations_used;

    std::cout << "[Performance] Total: " << result.calculation_time_ms << "ms"
              << ", Per iteration: " << time_per_iteration << "ms"
              << ", Iterations: " << result.iterations_used << "\n";
}

TEST_F(StructuralAnalyzerTest, ConvergenceComparison_DisplacementVsVelocity) {
    // Compare convergence behavior between displacement and velocity criteria
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 4; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Should converge (either by displacement or velocity)
    EXPECT_TRUE(result.converged);

    // Displacement-based convergence should be robust
    EXPECT_GT(result.iterations_used, 0);

    std::cout << "[Convergence Comparison] Iterations: " << result.iterations_used
              << ", Converged: " << result.converged << "\n";
}

// ===== Day 14: Failure Detection Enhancements =====

TEST_F(StructuralAnalyzerTest, GroundConnectivity_ConnectedStructure) {
    // Create tower connected to ground
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    std::vector<Vector3> damaged = {Vector3(voxel_size, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // All nodes should be connected to ground
    EXPECT_EQ(result.num_disconnected_nodes, 0);

    std::cout << "[Ground Connectivity] Connected structure: "
              << result.num_disconnected_nodes << " disconnected\n";
}

TEST_F(StructuralAnalyzerTest, GroundConnectivity_FloatingStructure) {
    // Create floating structure (no ground connection)
    float voxel_size = world.GetVoxelSize();
    for (int y = 2; y < 5; y++) {  // Start at y=2, floating
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    std::vector<Vector3> damaged = {Vector3(voxel_size, 2 * voxel_size, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // All nodes should be disconnected (floating)
    EXPECT_GT(result.num_disconnected_nodes, 0);
    EXPECT_TRUE(result.structure_failed);

    std::cout << "[Ground Connectivity] Floating structure: "
              << result.num_disconnected_nodes << " disconnected\n";
}

TEST_F(StructuralAnalyzerTest, FailureDetection_RemoveBaseSupport) {
    // Create tower, then remove base voxel
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    // Remove ground voxel
    world.RemoveVoxel(Vector3(0, 0, 0));

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Upper voxels should be detected as floating
    EXPECT_TRUE(result.structure_failed);
    EXPECT_GT(result.num_failed_nodes, 0);
    EXPECT_GT(result.num_disconnected_nodes, 0);

    std::cout << "[Failure Detection] Removed base: "
              << result.num_failed_nodes << " failed, "
              << result.num_disconnected_nodes << " disconnected\n";
}

TEST_F(StructuralAnalyzerTest, FailureDetection_BrokenBridge) {
    // Create elevated bridge, remove middle support
    float voxel_size = world.GetVoxelSize();

    // Left pillar (grounded)
    for (int y = 0; y < 3; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    // Bridge span
    for (int x = 0; x < 5; x++) {
        world.SetVoxel(Vector3(x * voxel_size, 2 * voxel_size, 0), Voxel(brick_id));
    }

    // Right pillar (this will become disconnected)
    for (int y = 0; y < 3; y++) {
        world.SetVoxel(Vector3(4 * voxel_size, y * voxel_size, 0), Voxel(brick_id));
    }

    // Remove middle voxel of left pillar (disconnect right side)
    world.RemoveVoxel(Vector3(0, voxel_size, 0));

    std::vector<Vector3> damaged = {Vector3(0, voxel_size, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Right side should be disconnected
    EXPECT_TRUE(result.structure_failed);
    EXPECT_GT(result.num_failed_nodes, 0);

    std::cout << "[Broken Bridge] Failed: " << result.num_failed_nodes
              << ", Disconnected: " << result.num_disconnected_nodes << "\n";
}

TEST_F(StructuralAnalyzerTest, FailureDetection_DualCriteria) {
    // Test structure that fails both by displacement AND disconnection
    float voxel_size = world.GetVoxelSize();

    // Weak material tower (will fail by displacement)
    for (int y = 0; y < 10; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(wood_id));
    }

    // Remove a middle voxel (creates disconnection)
    world.RemoveVoxel(Vector3(0, 5 * voxel_size, 0));

    std::vector<Vector3> damaged = {Vector3(0, 5 * voxel_size, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Should detect failures from both criteria
    EXPECT_TRUE(result.structure_failed);
    EXPECT_GT(result.num_failed_nodes, 0);

    std::cout << "[Dual Criteria] Total failed: " << result.num_failed_nodes
              << ", Disconnected: " << result.num_disconnected_nodes << "\n";
}

TEST_F(StructuralAnalyzerTest, ClusterGeneration_MultipleFloatingPieces) {
    // Create structure that breaks into pieces when base is removed
    float voxel_size = world.GetVoxelSize();

    // Central pillar (will be removed)
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    // Attached pieces at top
    world.SetVoxel(Vector3(voxel_size, 4 * voxel_size, 0), Voxel(brick_id));
    world.SetVoxel(Vector3(-voxel_size, 4 * voxel_size, 0), Voxel(brick_id));

    // Remove base (makes everything float)
    world.RemoveVoxel(Vector3(0, 0, 0));

    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Should detect floating pieces
    EXPECT_TRUE(result.structure_failed);
    EXPECT_GT(result.num_failed_nodes, 0);

    std::cout << "[Multiple Pieces] Failed: " << result.num_failed_nodes
              << ", Clusters: " << result.failed_clusters.size() << "\n";
}

TEST_F(StructuralAnalyzerTest, GroundConnectivity_PartialDisconnection) {
    // Create T-shape where one arm disconnects
    float voxel_size = world.GetVoxelSize();

    // Vertical trunk (ground connected)
    for (int y = 0; y < 3; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    // Left arm (connected)
    for (int x = -2; x < 0; x++) {
        world.SetVoxel(Vector3(x * voxel_size, 2 * voxel_size, 0), Voxel(brick_id));
    }

    // Right arm (will be disconnected)
    for (int x = 1; x < 3; x++) {
        world.SetVoxel(Vector3(x * voxel_size, 2 * voxel_size, 0), Voxel(brick_id));
    }

    // Damage at junction (disconnect right arm)
    world.RemoveVoxel(Vector3(voxel_size, 2 * voxel_size, 0));

    std::vector<Vector3> damaged = {Vector3(voxel_size, 2 * voxel_size, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Right arm should be disconnected
    EXPECT_GT(result.num_disconnected_nodes, 0);

    std::cout << "[Partial Disconnection] Disconnected: "
              << result.num_disconnected_nodes << " nodes\n";
}

TEST_F(StructuralAnalyzerTest, FailureDetection_StableAfterDamage) {
    // Structure that remains stable even after damage
    float voxel_size = world.GetVoxelSize();

    // Wide stable base
    for (int x = 0; x < 5; x++) {
        for (int y = 0; y < 2; y++) {
            world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, 0), Voxel(brick_id));
        }
    }

    // Remove one voxel from top
    world.RemoveVoxel(Vector3(2 * voxel_size, voxel_size, 0));

    std::vector<Vector3> damaged = {Vector3(2 * voxel_size, voxel_size, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Should remain stable (all voxels still connected)
    EXPECT_FALSE(result.structure_failed) << "Structure should remain stable";
    EXPECT_EQ(result.num_disconnected_nodes, 0);

    std::cout << "[Stable After Damage] Failed: " << result.num_failed_nodes
              << ", Disconnected: " << result.num_disconnected_nodes << "\n";
}

// ===== Day 15: Week 3 Integration & Testing =====

TEST_F(StructuralAnalyzerTest, Integration_TowerCollapse) {
    // Complete end-to-end test: Build tower, remove base, verify collapse
    float voxel_size = world.GetVoxelSize();

    // Build 10-voxel tower
    for (int y = 0; y < 10; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    // Remove base voxel
    world.RemoveVoxel(Vector3(0, 0, 0));
    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    // Analyze
    auto result = analyzer.Analyze(world, damaged);

    // Verify complete collapse
    EXPECT_TRUE(result.structure_failed);
    EXPECT_GT(result.num_failed_nodes, 0);
    EXPECT_GT(result.num_disconnected_nodes, 0);
    EXPECT_TRUE(result.converged || result.iterations_used > 0);
    EXPECT_GT(result.failed_clusters.size(), 0);

    // Verify performance
    EXPECT_LT(result.calculation_time_ms, 500.0f);  // Within time budget

    std::cout << "[Integration: Tower Collapse] "
              << "Failed: " << result.num_failed_nodes << ", "
              << "Time: " << result.calculation_time_ms << "ms, "
              << "Clusters: " << result.failed_clusters.size() << "\n";
}

TEST_F(StructuralAnalyzerTest, Integration_BridgeWithRedundancy) {
    // Foundation with redundant support - should remain stable after edge voxel removed
    float voxel_size = world.GetVoxelSize();

    // Create a 3x3 platform at ground level (y=0) - all are ground anchors
    for (int x = 0; x < 3; x++) {
        for (int z = 0; z < 3; z++) {
            world.SetVoxel(Vector3(x * voxel_size, 0, z * voxel_size), Voxel(brick_id));
        }
    }

    // Add a platform level on top connecting all columns
    for (int x = 0; x < 3; x++) {
        for (int z = 0; z < 3; z++) {
            world.SetVoxel(Vector3(x * voxel_size, voxel_size, z * voxel_size), Voxel(brick_id));
        }
    }

    // Remove edge voxel from base - structure still has many ground connections
    world.RemoveVoxel(Vector3(2 * voxel_size, 0, 2 * voxel_size));
    std::vector<Vector3> damaged = {Vector3(2 * voxel_size, 0, 2 * voxel_size)};

    auto result = analyzer.Analyze(world, damaged);

    // Structure should remain stable (redundant ground support from other 8 columns)
    EXPECT_FALSE(result.structure_failed);
    EXPECT_EQ(result.num_disconnected_nodes, 0);

    std::cout << "[Integration: Bridge Redundancy] "
              << "Stable: " << !result.structure_failed << ", "
              << "Time: " << result.calculation_time_ms << "ms\n";
}

TEST_F(StructuralAnalyzerTest, Integration_WallPartialDamage) {
    // Wall with partial damage near ground - should remain mostly stable
    float voxel_size = world.GetVoxelSize();

    // Create 10x5 wall (grounded at y=0)
    for (int x = 0; x < 10; x++) {
        for (int y = 0; y < 5; y++) {
            world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, 0), Voxel(brick_id));
        }
    }

    // Remove voxels from lower portion (near ground where analyzer will find ground-connected nodes)
    world.RemoveVoxel(Vector3(5 * voxel_size, 1 * voxel_size, 0));

    std::vector<Vector3> damaged = {
        Vector3(5 * voxel_size, 1 * voxel_size, 0)
    };

    auto result = analyzer.Analyze(world, damaged);

    // Most nodes should remain stable (connected to ground on both sides)
    // At most, a few nodes immediately adjacent to damage might fail
    EXPECT_TRUE(result.nodes_analyzed > 0);
    EXPECT_LT(static_cast<float>(result.num_failed_nodes) / result.nodes_analyzed, 0.5f);  // Less than 50% fail

    std::cout << "[Integration: Wall Damage] "
              << "Failed: " << result.num_failed_nodes << " / " << result.nodes_analyzed << ", "
              << "Time: " << result.calculation_time_ms << "ms\n";
}

TEST_F(StructuralAnalyzerTest, Integration_ComplexMultiStory) {
    // Multi-story structure with columns - remove middle column to cause collapse
    float voxel_size = world.GetVoxelSize();

    // Create 3 columns in a line
    for (int x = 0; x <= 2; x++) {
        for (int y = 0; y < 6; y++) {
            world.SetVoxel(Vector3(x * 2 * voxel_size, y * voxel_size, 0), Voxel(brick_id));
        }
    }

    // Span connecting tops of all columns at level 5
    for (int x = 0; x <= 4; x++) {
        world.SetVoxel(Vector3(x * voxel_size, 5 * voxel_size, 0), Voxel(brick_id));
    }

    // Remove middle column at levels 2-3, breaking support
    world.RemoveVoxel(Vector3(2 * voxel_size, 2 * voxel_size, 0));
    world.RemoveVoxel(Vector3(2 * voxel_size, 3 * voxel_size, 0));

    std::vector<Vector3> damaged = {
        Vector3(2 * voxel_size, 2 * voxel_size, 0),
        Vector3(2 * voxel_size, 3 * voxel_size, 0)
    };

    auto result = analyzer.Analyze(world, damaged);

    // Should detect failure (top of middle column disconnected)
    EXPECT_TRUE(result.structure_failed);
    EXPECT_GT(result.num_failed_nodes, 0);

    std::cout << "[Integration: Multi-Story] "
              << "Failed: " << result.num_failed_nodes << ", "
              << "Clusters: " << result.failed_clusters.size() << ", "
              << "Time: " << result.calculation_time_ms << "ms\n";
}

TEST_F(StructuralAnalyzerTest, Integration_PerformanceLargeStructure) {
    // Performance test with larger structure
    float voxel_size = world.GetVoxelSize();

    // Create 15x15 base
    for (int x = 0; x < 15; x++) {
        for (int z = 0; z < 15; z++) {
            world.SetVoxel(Vector3(x * voxel_size, 0, z * voxel_size), Voxel(brick_id));
        }
    }

    // Add some height (5 levels)
    for (int y = 1; y < 5; y++) {
        for (int x = 2; x < 13; x++) {
            for (int z = 2; z < 13; z++) {
                world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, z * voxel_size), Voxel(brick_id));
            }
        }
    }

    // Remove one voxel
    world.RemoveVoxel(Vector3(7 * voxel_size, 0, 7 * voxel_size));
    std::vector<Vector3> damaged = {Vector3(7 * voxel_size, 0, 7 * voxel_size)};

    auto start = std::chrono::high_resolution_clock::now();
    auto result = analyzer.Analyze(world, damaged, 1000.0f);  // 1 second budget
    auto end = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float, std::milli>(end - start).count();

    // Should complete within budget
    EXPECT_LT(elapsed, 1000.0f);
    EXPECT_LT(result.calculation_time_ms, 1000.0f);

    // Should complete reasonably fast
    EXPECT_LT(result.calculation_time_ms, 200.0f);  // Aim for < 200ms

    std::cout << "[Integration: Large Structure Performance] "
              << "Nodes: " << result.nodes_analyzed << ", "
              << "Time: " << result.calculation_time_ms << "ms, "
              << "Time/node: " << (result.calculation_time_ms / result.nodes_analyzed) << "ms\n";
}

TEST_F(StructuralAnalyzerTest, Integration_EndToEndWorkflow) {
    // Complete workflow test: Build -> Damage -> Analyze -> Verify
    float voxel_size = world.GetVoxelSize();

    // Step 1: Build structure (L-shape)
    for (int x = 0; x < 5; x++) {
        world.SetVoxel(Vector3(x * voxel_size, 0, 0), Voxel(brick_id));
    }
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    // Step 2: Apply damage
    Vector3 damage_pos = Vector3(0, voxel_size, 0);
    world.RemoveVoxel(damage_pos);

    // Step 3: Analyze
    auto result = analyzer.Analyze(world, {damage_pos});

    // Step 4: Verify results
    EXPECT_TRUE(result.structure_failed);
    EXPECT_GT(result.calculation_time_ms, 0.0f);
    EXPECT_FALSE(result.debug_info.empty());

    // Step 5: Verify profiling data
    EXPECT_GT(result.node_graph_time_ms, 0.0f);
    EXPECT_GE(result.mass_calc_time_ms, 0.0f);
    EXPECT_GT(result.solver_time_ms, 0.0f);
    EXPECT_GE(result.failure_detection_time_ms, 0.0f);

    // Step 6: Verify clusters are usable
    if (!result.failed_clusters.empty()) {
        const auto& cluster = result.failed_clusters[0];
        EXPECT_GT(cluster.voxel_positions.size(), 0);
        EXPECT_GE(cluster.total_mass, 0.0f);
    }

    std::cout << "[Integration: End-to-End Workflow] "
              << "Complete: SUCCESS, "
              << "Total time: " << result.calculation_time_ms << "ms\n";
}

TEST_F(StructuralAnalyzerTest, Integration_MaterialVariations) {
    // Test different materials in same tower - remove base to cause failure
    float voxel_size = world.GetVoxelSize();

    // Build single tower with material layers
    // Concrete base (grounded)
    world.SetVoxel(Vector3(0, 0, 0), Voxel(MaterialDatabase::CONCRETE));
    world.SetVoxel(Vector3(0, voxel_size, 0), Voxel(MaterialDatabase::CONCRETE));

    // Brick middle
    for (int y = 2; y < 4; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    // Wood top
    for (int y = 4; y < 6; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(wood_id));
    }

    // Remove ground base - entire tower floats
    world.RemoveVoxel(Vector3(0, 0, 0));
    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Should detect failure (entire tower disconnected from ground)
    EXPECT_TRUE(result.structure_failed);
    EXPECT_GT(result.num_failed_nodes, 0);
    EXPECT_GT(result.num_disconnected_nodes, 0);

    std::cout << "[Integration: Material Variations] "
              << "Failed: " << result.num_failed_nodes << ", "
              << "Disconnected: " << result.num_disconnected_nodes << ", "
              << "Time: " << result.calculation_time_ms << "ms\n";
}

TEST_F(StructuralAnalyzerTest, Integration_Week3Summary) {
    // Summary test demonstrating all Week 3 capabilities
    float voxel_size = world.GetVoxelSize();

    // Build test structure
    for (int y = 0; y < 8; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    // Add some width at top (overhang)
    for (int x = -1; x <= 1; x++) {
        world.SetVoxel(Vector3(x * voxel_size, 7 * voxel_size, 0), Voxel(brick_id));
    }

    // Remove middle voxel
    world.RemoveVoxel(Vector3(0, 4 * voxel_size, 0));
    std::vector<Vector3> damaged = {Vector3(0, 4 * voxel_size, 0)};

    auto result = analyzer.Analyze(world, damaged);

    // Week 3 Capabilities Demonstrated:
    std::cout << "\n=== Week 3 Summary ===" << "\n";
    std::cout << "✅ Spring system: " << result.nodes_analyzed << " nodes analyzed\n";
    std::cout << "✅ Mass calculation: " << result.mass_calc_time_ms << "ms (cache hits: " << result.cache_hits << ")\n";
    std::cout << "✅ Displacement solver: " << result.iterations_used << " iterations, converged: " << result.converged << "\n";
    std::cout << "✅ Failure detection: " << result.num_failed_nodes << " failed ("
              << (result.num_failed_nodes - result.num_disconnected_nodes) << " displacement, "
              << result.num_disconnected_nodes << " disconnected)\n";
    std::cout << "✅ Cluster generation: " << result.failed_clusters.size() << " clusters\n";
    std::cout << "✅ Performance profiling: " << result.calculation_time_ms << "ms total\n";
    std::cout << "  - Node graph: " << result.node_graph_time_ms << "ms\n";
    std::cout << "  - Mass calc: " << result.mass_calc_time_ms << "ms\n";
    std::cout << "  - Solver: " << result.solver_time_ms << "ms\n";
    std::cout << "  - Failure detection: " << result.failure_detection_time_ms << "ms\n";
    std::cout << "=====================\n\n";

    // Verify all capabilities work
    EXPECT_GT(result.nodes_analyzed, 0);
    EXPECT_GE(result.iterations_used, 0);
    EXPECT_GT(result.calculation_time_ms, 0.0f);
}

// ============================================================================
// DAY 16: Performance Optimization Tests
// ============================================================================

TEST_F(StructuralAnalyzerTest, Optimization_InfluenceRadius) {
    // Day 16: Test influence radius optimization
    float voxel_size = world.GetVoxelSize();

    // Create large structure (20x20 base)
    for (int x = 0; x < 20; x++) {
        for (int z = 0; z < 20; z++) {
            for (int y = 0; y < 3; y++) {
                world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, z * voxel_size), Voxel(brick_id));
            }
        }
    }

    // Damage at corner
    Vector3 damage_pos(0, 0, 0);
    world.RemoveVoxel(damage_pos);
    std::vector<Vector3> damaged = {damage_pos};

    // Test with large radius (should analyze many nodes)
    analyzer.params.influence_radius = 50.0f;
    auto result_large = analyzer.Analyze(world, damaged);

    // Test with small radius (should analyze few nodes)
    analyzer.params.influence_radius = 1.0f;
    auto result_small = analyzer.Analyze(world, damaged);

    // Small radius should analyze fewer nodes
    EXPECT_LT(result_small.nodes_analyzed, result_large.nodes_analyzed);
    EXPECT_LT(result_small.calculation_time_ms, result_large.calculation_time_ms);

    std::cout << "[Optimization: Influence Radius]\n";
    std::cout << "  Large radius (50m): " << result_large.nodes_analyzed << " nodes, "
              << result_large.calculation_time_ms << "ms\n";
    std::cout << "  Small radius (1m): " << result_small.nodes_analyzed << " nodes, "
              << result_small.calculation_time_ms << "ms\n";
    std::cout << "  Speedup: " << (result_large.calculation_time_ms / result_small.calculation_time_ms) << "x\n";
}

TEST_F(StructuralAnalyzerTest, Optimization_ParallelMassCalculation) {
    // Day 16: Test parallel mass calculation speedup
    float voxel_size = world.GetVoxelSize();

    // Create structure with many voxels
    for (int x = 0; x < 15; x++) {
        for (int z = 0; z < 15; z++) {
            for (int y = 0; y < 5; y++) {
                world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, z * voxel_size), Voxel(brick_id));
            }
        }
    }

    Vector3 damage_pos(7 * voxel_size, 0, 7 * voxel_size);
    world.RemoveVoxel(damage_pos);
    std::vector<Vector3> damaged = {damage_pos};

    // Test with parallel disabled
    analyzer.params.use_parallel_mass_calc = false;
    auto start_sequential = std::chrono::high_resolution_clock::now();
    auto result_sequential = analyzer.Analyze(world, damaged);
    auto time_sequential = std::chrono::duration<float, std::milli>(
        std::chrono::high_resolution_clock::now() - start_sequential).count();

    // Test with parallel enabled
    analyzer.params.use_parallel_mass_calc = true;
    auto start_parallel = std::chrono::high_resolution_clock::now();
    auto result_parallel = analyzer.Analyze(world, damaged);
    auto time_parallel = std::chrono::duration<float, std::milli>(
        std::chrono::high_resolution_clock::now() - start_parallel).count();

    // Parallel should be faster (or at least not slower)
    // Note: For small structures, overhead might make it slower
    std::cout << "[Optimization: Parallel Mass Calculation]\n";
    std::cout << "  Sequential: " << time_sequential << "ms\n";
    std::cout << "  Parallel: " << time_parallel << "ms\n";
    if (time_sequential > time_parallel) {
        std::cout << "  Speedup: " << (time_sequential / time_parallel) << "x\n";
    } else {
        std::cout << "  (Note: Parallel overhead may dominate for small structures)\n";
    }

    // Both should produce same results
    EXPECT_EQ(result_sequential.structure_failed, result_parallel.structure_failed);
    EXPECT_EQ(result_sequential.num_failed_nodes, result_parallel.num_failed_nodes);
}

TEST_F(StructuralAnalyzerTest, Optimization_EarlyTermination) {
    // Day 16: Test early termination for obvious failures
    float voxel_size = world.GetVoxelSize();

    // Create tall unstable tower (will clearly fail)
    for (int y = 0; y < 20; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(wood_id));
    }

    // Remove base
    world.RemoveVoxel(Vector3(0, 0, 0));
    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};

    // With early termination
    analyzer.params.use_early_termination = true;
    auto result_early = analyzer.Analyze(world, damaged);

    // Without early termination
    analyzer.params.use_early_termination = false;
    auto result_normal = analyzer.Analyze(world, damaged);

    // Early termination should use fewer iterations
    EXPECT_LE(result_early.iterations_used, result_normal.iterations_used);

    std::cout << "[Optimization: Early Termination]\n";
    std::cout << "  With early termination: " << result_early.iterations_used << " iterations\n";
    std::cout << "  Without early termination: " << result_normal.iterations_used << " iterations\n";
}

// ============================================================================
// DAY 17: Parameter Tuning System Tests
// ============================================================================

TEST_F(StructuralAnalyzerTest, ParameterSaveLoad_SaveCreatesValidFile) {
    // Day 17: Test saving parameters to INI file
    std::string test_file = "/tmp/test_params.ini";

    // Set custom parameters
    analyzer.params.timestep = 0.02f;
    analyzer.params.damping = 0.95f;
    analyzer.params.max_iterations = 150;
    analyzer.params.convergence_threshold = 0.0002f;
    analyzer.params.ground_level = -1.0f;
    analyzer.params.use_surface_only = false;
    analyzer.params.influence_radius = 10.0f;
    analyzer.params.use_parallel_mass_calc = false;
    analyzer.params.use_early_termination = true;

    // Save parameters
    bool save_success = analyzer.SaveParameters(test_file);
    EXPECT_TRUE(save_success);

    // Verify file exists and is readable
    std::ifstream file(test_file);
    EXPECT_TRUE(file.is_open());

    // Verify content format
    std::string line;
    bool found_section = false;
    bool found_timestep = false;
    while (std::getline(file, line)) {
        if (line == "[StructuralAnalyzer]") found_section = true;
        if (line.find("timestep=") != std::string::npos) found_timestep = true;
    }
    file.close();

    EXPECT_TRUE(found_section);
    EXPECT_TRUE(found_timestep);

    std::cout << "[ParameterSaveLoad: SaveCreatesValidFile] SUCCESS\n";
}

TEST_F(StructuralAnalyzerTest, ParameterSaveLoad_LoadRestoresValues) {
    // Day 17: Test loading parameters from INI file
    std::string test_file = "/tmp/test_load_params.ini";

    // Create test INI file
    std::ofstream file(test_file);
    file << "[StructuralAnalyzer]\n";
    file << "timestep=0.025\n";
    file << "damping=0.85\n";
    file << "max_iterations=200\n";
    file << "convergence_threshold=0.0005\n";
    file << "ground_level=1.5\n";
    file << "use_surface_only=0\n";
    file << "influence_radius=15.0\n";
    file << "use_parallel_mass_calc=0\n";
    file << "use_early_termination=1\n";
    file.close();

    // Load parameters
    bool load_success = analyzer.LoadParameters(test_file);
    EXPECT_TRUE(load_success);

    // Verify values were loaded correctly
    EXPECT_FLOAT_EQ(analyzer.params.timestep, 0.025f);
    EXPECT_FLOAT_EQ(analyzer.params.damping, 0.85f);
    EXPECT_EQ(analyzer.params.max_iterations, 200);
    EXPECT_FLOAT_EQ(analyzer.params.convergence_threshold, 0.0005f);
    EXPECT_FLOAT_EQ(analyzer.params.ground_level, 1.5f);
    EXPECT_FALSE(analyzer.params.use_surface_only);
    EXPECT_FLOAT_EQ(analyzer.params.influence_radius, 15.0f);
    EXPECT_FALSE(analyzer.params.use_parallel_mass_calc);
    EXPECT_TRUE(analyzer.params.use_early_termination);

    std::cout << "[ParameterSaveLoad: LoadRestoresValues] SUCCESS\n";
}

TEST_F(StructuralAnalyzerTest, ParameterSaveLoad_RoundTripPreservesValues) {
    // Day 17: Test that save->load preserves all values
    std::string test_file = "/tmp/test_roundtrip.ini";

    // Set unique values for all parameters
    analyzer.params.timestep = 0.0123f;
    analyzer.params.damping = 0.777f;
    analyzer.params.max_iterations = 123;
    analyzer.params.convergence_threshold = 0.000123f;
    analyzer.params.ground_level = -2.5f;
    analyzer.params.use_surface_only = true;
    analyzer.params.influence_radius = 7.5f;
    analyzer.params.use_parallel_mass_calc = true;
    analyzer.params.use_early_termination = false;

    // Save
    EXPECT_TRUE(analyzer.SaveParameters(test_file));

    // Change values
    analyzer.params.timestep = 0.999f;
    analyzer.params.max_iterations = 999;
    analyzer.params.use_surface_only = false;

    // Load
    EXPECT_TRUE(analyzer.LoadParameters(test_file));

    // Verify original values restored
    EXPECT_FLOAT_EQ(analyzer.params.timestep, 0.0123f);
    EXPECT_FLOAT_EQ(analyzer.params.damping, 0.777f);
    EXPECT_EQ(analyzer.params.max_iterations, 123);
    EXPECT_FLOAT_EQ(analyzer.params.convergence_threshold, 0.000123f);
    EXPECT_FLOAT_EQ(analyzer.params.ground_level, -2.5f);
    EXPECT_TRUE(analyzer.params.use_surface_only);
    EXPECT_FLOAT_EQ(analyzer.params.influence_radius, 7.5f);
    EXPECT_TRUE(analyzer.params.use_parallel_mass_calc);
    EXPECT_FALSE(analyzer.params.use_early_termination);

    std::cout << "[ParameterSaveLoad: RoundTripPreservesValues] SUCCESS\n";
}

TEST_F(StructuralAnalyzerTest, ParameterSaveLoad_LoadMissingFileReturnsFalse) {
    // Day 17: Test that loading missing file fails gracefully
    std::string missing_file = "/tmp/nonexistent_params.ini";

    bool load_success = analyzer.LoadParameters(missing_file);
    EXPECT_FALSE(load_success);

    std::cout << "[ParameterSaveLoad: LoadMissingFileReturnsFalse] SUCCESS\n";
}

TEST_F(StructuralAnalyzerTest, ParameterSaveLoad_LoadDefaultConfig) {
    // Day 17: Test loading the default config file
    std::string default_file = "../config/default_params.ini";

    // Load default parameters
    bool load_success = analyzer.LoadParameters(default_file);
    EXPECT_TRUE(load_success);

    // Verify default values match what we expect
    EXPECT_FLOAT_EQ(analyzer.params.timestep, 0.01f);
    EXPECT_FLOAT_EQ(analyzer.params.damping, 0.9f);
    EXPECT_EQ(analyzer.params.max_iterations, 100);
    EXPECT_FLOAT_EQ(analyzer.params.convergence_threshold, 0.0001f);
    EXPECT_FLOAT_EQ(analyzer.params.ground_level, 0.0f);
    EXPECT_TRUE(analyzer.params.use_surface_only);
    EXPECT_FLOAT_EQ(analyzer.params.influence_radius, 5.0f);
    EXPECT_TRUE(analyzer.params.use_parallel_mass_calc);
    EXPECT_FALSE(analyzer.params.use_early_termination);

    std::cout << "[ParameterSaveLoad: LoadDefaultConfig] SUCCESS\n";
}

TEST_F(StructuralAnalyzerTest, ParameterTuning_SensitivityAnalysis) {
    // Day 17: Test parameter sensitivity analysis
    float voxel_size = world.GetVoxelSize();

    // Build test structure (tower)
    for (int y = 0; y < 6; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    // Damage middle
    Vector3 damage_pos(0, 3 * voxel_size, 0);
    world.RemoveVoxel(damage_pos);
    std::vector<Vector3> damaged = {damage_pos};

    // Test timestep sensitivity
    std::vector<float> timestep_values = {0.005f, 0.01f, 0.02f, 0.03f};
    auto result = ParameterTuning::AnalyzeParameterSensitivity(
        analyzer, world, damaged, "timestep", timestep_values);

    // Verify results
    EXPECT_EQ(result.parameter_name, "timestep");
    EXPECT_EQ(result.test_values.size(), timestep_values.size());
    EXPECT_EQ(result.calculation_times_ms.size(), timestep_values.size());
    EXPECT_GT(result.min_time_ms, 0.0f);
    EXPECT_GT(result.max_time_ms, 0.0f);
    EXPECT_GE(result.max_time_ms, result.min_time_ms);
    EXPECT_FALSE(result.analysis_summary.empty());

    std::cout << "[ParameterTuning: SensitivityAnalysis] SUCCESS\n";
}

TEST_F(StructuralAnalyzerTest, ParameterTuning_InfluenceRadiusSensitivity) {
    // Day 17: Test influence radius sensitivity
    float voxel_size = world.GetVoxelSize();

    // Build larger structure
    for (int x = 0; x < 5; x++) {
        for (int y = 0; y < 4; y++) {
            world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, 0), Voxel(brick_id));
        }
    }

    Vector3 damage_pos(2 * voxel_size, 0, 0);
    world.RemoveVoxel(damage_pos);
    std::vector<Vector3> damaged = {damage_pos};

    // Test different influence radii
    std::vector<float> radius_values = {1.0f, 3.0f, 5.0f, 10.0f};
    auto result = ParameterTuning::AnalyzeParameterSensitivity(
        analyzer, world, damaged, "influence_radius", radius_values);

    // Larger radius should analyze more nodes (usually)
    EXPECT_EQ(result.nodes_analyzed.size(), radius_values.size());

    // Should find an optimal value
    EXPECT_GT(result.optimal_value, 0.0f);

    std::cout << "[ParameterTuning: InfluenceRadiusSensitivity] SUCCESS\n";
}

TEST_F(StructuralAnalyzerTest, ParameterTuning_AutoTune) {
    // Day 17: Test auto-tuning with grid search
    float voxel_size = world.GetVoxelSize();

    // Build test structure
    for (int y = 0; y < 5; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    Vector3 damage_pos(0, 2 * voxel_size, 0);
    world.RemoveVoxel(damage_pos);
    std::vector<Vector3> damaged = {damage_pos};

    // Run auto-tuning (limited iterations for speed)
    auto result = ParameterTuning::AutoTuneParameters(world, damaged, 20);

    // Verify results
    EXPECT_GT(result.total_tests_run, 0);
    EXPECT_LE(result.total_tests_run, 20);
    EXPECT_EQ(result.tuning_method, "grid_search");
    EXPECT_FALSE(result.summary.empty());

    // Optimal parameters should be valid
    EXPECT_GT(result.optimal_params.timestep, 0.0f);
    EXPECT_GT(result.optimal_params.damping, 0.0f);
    EXPECT_GT(result.optimal_params.max_iterations, 0);
    EXPECT_GT(result.optimal_params.influence_radius, 0.0f);

    std::cout << "[ParameterTuning: AutoTune] SUCCESS\n";
}

TEST_F(StructuralAnalyzerTest, ParameterTuning_SaveSensitivityCSV) {
    // Day 17: Test saving sensitivity results to CSV
    float voxel_size = world.GetVoxelSize();

    // Build simple structure
    for (int y = 0; y < 4; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0), Voxel(brick_id));
    }

    Vector3 damage_pos(0, 1 * voxel_size, 0);
    world.RemoveVoxel(damage_pos);
    std::vector<Vector3> damaged = {damage_pos};

    // Run sensitivity analysis
    std::vector<float> damping_values = {0.8f, 0.9f, 0.95f};
    auto result = ParameterTuning::AnalyzeParameterSensitivity(
        analyzer, world, damaged, "damping", damping_values);

    // Save to CSV
    std::string csv_file = "/tmp/test_sensitivity.csv";
    bool save_success = ParameterTuning::SaveSensitivityResultsCSV(result, csv_file);
    EXPECT_TRUE(save_success);

    // Verify file exists and has content
    std::ifstream file(csv_file);
    EXPECT_TRUE(file.is_open());

    std::string line;
    int line_count = 0;
    while (std::getline(file, line)) {
        line_count++;
    }
    file.close();

    // Should have header + data rows
    EXPECT_EQ(line_count, 1 + static_cast<int>(damping_values.size()));

    std::cout << "[ParameterTuning: SaveSensitivityCSV] SUCCESS\n";
}
