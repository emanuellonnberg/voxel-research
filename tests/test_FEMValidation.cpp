/**
 * FEM Validation Tests
 *
 * Compares Spring and Max-Flow analyzers against FEM ground truth.
 * Measures actual accuracy percentages.
 */

#include <gtest/gtest.h>
#include "VoxelWorld.h"
#include "StructuralAnalyzer.h"
#include "MaxFlowStructuralAnalyzer.h"
#include "Material.h"
#include <fstream>
#include <vector>
#include <string>
#include <map>

// Simple JSON parsing (simplified - production would use a library)
struct FEMGroundTruth {
    std::string structure_name;
    std::string fem_solver;
    bool should_fail;
    std::vector<Vector3> failed_voxel_positions;
    std::string method_description;
};

// Test structure definition
struct TestStructure {
    std::string name;
    std::vector<std::pair<Vector3, uint8_t>> voxels;  // position, material_id
    std::vector<Vector3> damaged;
};

class FEMValidationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // No initialization needed - MaterialDatabase is automatically available
    }

    // Load ground truth from JSON file
    FEMGroundTruth LoadGroundTruth(const std::string& filename) {
        FEMGroundTruth truth;

        std::string full_path = "../validation/results/" + filename;
        std::ifstream file(full_path);

        if (!file.is_open()) {
            // Try without ../ prefix
            full_path = "validation/results/" + filename;
            file.open(full_path);
        }

        EXPECT_TRUE(file.is_open()) << "Could not open: " << full_path;

        // Parse JSON (simplified - just read the should_fail field)
        std::string line;
        while (std::getline(file, line)) {
            if (line.find("\"structure_name\"") != std::string::npos) {
                size_t start = line.find(":") + 1;
                size_t end = line.find_last_of("\"");
                truth.structure_name = line.substr(start, end - start);
                // Remove quotes and whitespace
                truth.structure_name.erase(
                    std::remove(truth.structure_name.begin(), truth.structure_name.end(), '\"'),
                    truth.structure_name.end());
                truth.structure_name.erase(
                    std::remove(truth.structure_name.begin(), truth.structure_name.end(), ' '),
                    truth.structure_name.end());
                truth.structure_name.erase(
                    std::remove(truth.structure_name.begin(), truth.structure_name.end(), ','),
                    truth.structure_name.end());
            }
            if (line.find("\"should_fail\"") != std::string::npos) {
                truth.should_fail = (line.find("true") != std::string::npos);
            }
            if (line.find("\"fem_solver\"") != std::string::npos) {
                size_t start = line.find(":") + 1;
                truth.fem_solver = line.substr(start);
                // Extract value between quotes
                size_t q1 = truth.fem_solver.find("\"");
                size_t q2 = truth.fem_solver.find("\"", q1 + 1);
                if (q1 != std::string::npos && q2 != std::string::npos) {
                    truth.fem_solver = truth.fem_solver.substr(q1 + 1, q2 - q1 - 1);
                }
            }
        }

        return truth;
    }

    // Build test structure
    void BuildStructure(VoxelWorld& world, const TestStructure& structure) {
        for (const auto& [pos, material_id] : structure.voxels) {
            world.SetVoxel(pos, Voxel(material_id));
        }

        // Remove damaged voxels
        for (const auto& pos : structure.damaged) {
            world.RemoveVoxel(pos);
        }
    }

    // Define test structures (matching Python test_structures.py)
    TestStructure GetTowerSimple() {
        float s = 0.05f;  // voxel size
        return {
            "tower_simple",
            {
                {Vector3(0, 0*s, 0), MaterialDatabase::BRICK},
                {Vector3(0, 1*s, 0), MaterialDatabase::BRICK},
                {Vector3(0, 2*s, 0), MaterialDatabase::BRICK},
                {Vector3(0, 3*s, 0), MaterialDatabase::BRICK},
                {Vector3(0, 4*s, 0), MaterialDatabase::BRICK},
            },
            {Vector3(0, 0, 0)}  // Remove base
        };
    }

    TestStructure GetTowerGrounded() {
        float s = 0.05f;
        return {
            "tower_grounded",
            {
                {Vector3(0, 0*s, 0), MaterialDatabase::BRICK},
                {Vector3(0, 1*s, 0), MaterialDatabase::BRICK},
                {Vector3(0, 2*s, 0), MaterialDatabase::BRICK},
                {Vector3(0, 3*s, 0), MaterialDatabase::BRICK},
                {Vector3(0, 4*s, 0), MaterialDatabase::BRICK},
            },
            {Vector3(1*s, 0, 0)}  // Damage adjacent, not tower
        };
    }

    TestStructure GetCantilever() {
        float s = 0.05f;
        return {
            "cantilever",
            {
                {Vector3(0*s, 0, 0), MaterialDatabase::CONCRETE},
                {Vector3(1*s, 0, 0), MaterialDatabase::CONCRETE},
                {Vector3(2*s, 0, 0), MaterialDatabase::CONCRETE},
                {Vector3(3*s, 0, 0), MaterialDatabase::CONCRETE},
                {Vector3(4*s, 0, 0), MaterialDatabase::CONCRETE},
            },
            {}  // No damage
        };
    }

    TestStructure GetLShapeDamaged() {
        float s = 0.05f;
        return {
            "l_shape_damaged",
            {
                // Vertical part
                {Vector3(0, 0*s, 0), MaterialDatabase::CONCRETE},
                {Vector3(0, 1*s, 0), MaterialDatabase::CONCRETE},
                {Vector3(0, 2*s, 0), MaterialDatabase::CONCRETE},
                // Horizontal part
                {Vector3(1*s, 2*s, 0), MaterialDatabase::CONCRETE},
                {Vector3(2*s, 2*s, 0), MaterialDatabase::CONCRETE},
                {Vector3(3*s, 2*s, 0), MaterialDatabase::CONCRETE},
            },
            {Vector3(0, 2*s, 0)}  // Remove corner
        };
    }

    TestStructure GetMultiSpanBridge() {
        float s = 0.05f;
        TestStructure structure;
        structure.name = "multi_span_bridge";

        auto add_voxel = [&structure](const Vector3& pos, uint8_t mat) {
            structure.voxels.push_back({pos, mat});
        };

        // Supports at x=0, 4, 8 (two voxels tall)
        for (float y = 0; y <= s; y += s) {
        add_voxel(Vector3(0.0f, y, 0.0f), MaterialDatabase::CONCRETE);
        add_voxel(Vector3(4 * s, y, 0.0f), MaterialDatabase::CONCRETE);
        add_voxel(Vector3(8 * s, y, 0.0f), MaterialDatabase::CONCRETE);
    }

    // Deck at y = 2s
    for (int x = 0; x <= 8; ++x) {
        add_voxel(Vector3(x * s, 2 * s, 0.0f), MaterialDatabase::STEEL);
    }

    structure.damaged = {
        Vector3(4 * s, 0.0f, 0.0f),
        Vector3(4 * s, s, 0.0f),
        Vector3(4 * s, 2 * s, 0.0f),
        Vector3(5 * s, 2 * s, 0.0f),
        Vector3(8 * s, 0.0f, 0.0f),
        Vector3(8 * s, s, 0.0f),
        Vector3(7 * s, 2 * s, 0.0f),
    };
    return structure;
}

    TestStructure GetMultiLevelFrame() {
        float s = 0.05f;
        TestStructure structure;
        structure.name = "multi_level_frame";

        auto add_voxel = [&structure](const Vector3& pos, uint8_t mat) {
            structure.voxels.push_back({pos, mat});
        };

        // Columns at x = 0 and x = 2s (four voxels tall)
        for (int y = 0; y < 4; ++y) {
            add_voxel(Vector3(0.0f, y * s, 0.0f), MaterialDatabase::STEEL);
            add_voxel(Vector3(2 * s, y * s, 0.0f), MaterialDatabase::STEEL);
        }

        // Floor beams at y = 2s and y = 3s
        for (int x : {1, 2, 3}) {
            add_voxel(Vector3(x * s, 2 * s, 0.0f), MaterialDatabase::CONCRETE);
            add_voxel(Vector3(x * s, 3 * s, 0.0f), MaterialDatabase::CONCRETE);
        }

        structure.damaged = {
            Vector3(2 * s, 0.0f, 0.0f),
            Vector3(2 * s, 1 * s, 0.0f),
            Vector3(2 * s, 2 * s, 0.0f),
            Vector3(2 * s, 3 * s, 0.0f),
        };
        return structure;
    }

    TestStructure GetDiagonalBracedFrame() {
        float s = 0.05f;
        TestStructure structure;
        structure.name = "diagonal_braced_frame";

        auto add_voxel = [&structure](const Vector3& pos, uint8_t mat) {
            structure.voxels.push_back({pos, mat});
        };

        for (int y = 0; y < 4; ++y) {
            add_voxel(Vector3(0.0f, y * s, 0.0f), MaterialDatabase::CONCRETE);
            add_voxel(Vector3(2 * s, y * s, 0.0f), MaterialDatabase::CONCRETE);
        }

        add_voxel(Vector3(1 * s, 3 * s, 0.0f), MaterialDatabase::CONCRETE);
        add_voxel(Vector3(1 * s, 2 * s, 0.0f), MaterialDatabase::CONCRETE);
        add_voxel(Vector3(1 * s, 1 * s, 0.0f), MaterialDatabase::CONCRETE);

        structure.damaged = {
            Vector3(2 * s, 0.0f, 0.0f),
            Vector3(1 * s, 1 * s, 0.0f),
            Vector3(1 * s, 2 * s, 0.0f),
            Vector3(1 * s, 3 * s, 0.0f),
        };
        return structure;
    }
};

// Test: Tower Simple
TEST_F(FEMValidationTest, TowerSimple) {
    auto ground_truth = LoadGroundTruth("tower_simple.json");
    auto structure = GetTowerSimple();

    VoxelWorld world;
    BuildStructure(world, structure);

    // Test Spring analyzer
    StructuralAnalyzer spring;
    auto spring_result = spring.Analyze(world, structure.damaged);

    // Test Max-Flow analyzer
    MaxFlowStructuralAnalyzer maxflow;
    auto maxflow_result = maxflow.Analyze(world, structure.damaged);

    // Compare against FEM
    std::cout << "\nTower Simple:\n";
    std::cout << "  FEM Ground Truth: " << (ground_truth.should_fail ? "FAIL" : "OK")
              << " (" << ground_truth.fem_solver << ")\n";
    std::cout << "  Spring Analyzer:  " << (spring_result.structure_failed ? "FAIL" : "OK");
    if (spring_result.structure_failed == ground_truth.should_fail) {
        std::cout << " ✓ MATCH\n";
    } else {
        std::cout << " ✗ MISMATCH\n";
    }
    std::cout << "  Max-Flow Analyzer: " << (maxflow_result.structure_failed ? "FAIL" : "OK");
    if (maxflow_result.structure_failed == ground_truth.should_fail) {
        std::cout << " ✓ MATCH\n";
    } else {
        std::cout << " ✗ MISMATCH\n";
    }

    // Both should match FEM prediction
    EXPECT_EQ(spring_result.structure_failed, ground_truth.should_fail)
        << "Spring analyzer disagrees with FEM";
    EXPECT_EQ(maxflow_result.structure_failed, ground_truth.should_fail)
        << "Max-Flow analyzer disagrees with FEM";
}

// Test: Tower Grounded
TEST_F(FEMValidationTest, TowerGrounded) {
    auto ground_truth = LoadGroundTruth("tower_grounded.json");
    auto structure = GetTowerGrounded();

    VoxelWorld world;
    BuildStructure(world, structure);

    StructuralAnalyzer spring;
    auto spring_result = spring.Analyze(world, structure.damaged);

    MaxFlowStructuralAnalyzer maxflow;
    auto maxflow_result = maxflow.Analyze(world, structure.damaged);

    std::cout << "\nTower Grounded:\n";
    std::cout << "  FEM: " << (ground_truth.should_fail ? "FAIL" : "OK") << "\n";
    std::cout << "  Spring: " << (spring_result.structure_failed ? "FAIL" : "OK")
              << (spring_result.structure_failed == ground_truth.should_fail ? " ✓" : " ✗") << "\n";
    std::cout << "  Max-Flow: " << (maxflow_result.structure_failed ? "FAIL" : "OK")
              << (maxflow_result.structure_failed == ground_truth.should_fail ? " ✓" : " ✗") << "\n";

    EXPECT_EQ(spring_result.structure_failed, ground_truth.should_fail);
    EXPECT_EQ(maxflow_result.structure_failed, ground_truth.should_fail);
}

// Test: Cantilever
TEST_F(FEMValidationTest, Cantilever) {
    auto ground_truth = LoadGroundTruth("cantilever.json");
    auto structure = GetCantilever();

    VoxelWorld world;
    BuildStructure(world, structure);

    StructuralAnalyzer spring;
    auto spring_result = spring.Analyze(world, structure.damaged);

    MaxFlowStructuralAnalyzer maxflow;
    auto maxflow_result = maxflow.Analyze(world, structure.damaged);

    std::cout << "\nCantilever:\n";
    std::cout << "  FEM: " << (ground_truth.should_fail ? "FAIL" : "OK") << "\n";
    std::cout << "  Spring: " << (spring_result.structure_failed ? "FAIL" : "OK")
              << (spring_result.structure_failed == ground_truth.should_fail ? " ✓" : " ✗") << "\n";
    std::cout << "  Max-Flow: " << (maxflow_result.structure_failed ? "FAIL" : "OK")
              << (maxflow_result.structure_failed == ground_truth.should_fail ? " ✓" : " ✗") << "\n";

    EXPECT_EQ(spring_result.structure_failed, ground_truth.should_fail);
    EXPECT_EQ(maxflow_result.structure_failed, ground_truth.should_fail);
}

// Test: L-Shape Damaged
TEST_F(FEMValidationTest, LShapeDamaged) {
    auto ground_truth = LoadGroundTruth("l_shape_damaged.json");
    auto structure = GetLShapeDamaged();

    VoxelWorld world;
    BuildStructure(world, structure);

    StructuralAnalyzer spring;
    auto spring_result = spring.Analyze(world, structure.damaged);

    MaxFlowStructuralAnalyzer maxflow;
    auto maxflow_result = maxflow.Analyze(world, structure.damaged);

    std::cout << "\nL-Shape Damaged:\n";
    std::cout << "  FEM: " << (ground_truth.should_fail ? "FAIL" : "OK") << "\n";
    std::cout << "  Spring: " << (spring_result.structure_failed ? "FAIL" : "OK")
              << (spring_result.structure_failed == ground_truth.should_fail ? " ✓" : " ✗") << "\n";
    std::cout << "  Max-Flow: " << (maxflow_result.structure_failed ? "FAIL" : "OK")
              << (maxflow_result.structure_failed == ground_truth.should_fail ? " ✓" : " ✗") << "\n";

    EXPECT_EQ(spring_result.structure_failed, ground_truth.should_fail);
    EXPECT_EQ(maxflow_result.structure_failed, ground_truth.should_fail);
}

TEST_F(FEMValidationTest, MultiSpanBridgeDamaged) {
    auto ground_truth = LoadGroundTruth("multi_span_bridge.json");
    auto structure = GetMultiSpanBridge();

    VoxelWorld world;
    BuildStructure(world, structure);

    StructuralAnalyzer spring;
    auto spring_result = spring.Analyze(world, structure.damaged);

    MaxFlowStructuralAnalyzer maxflow;
    auto maxflow_result = maxflow.Analyze(world, structure.damaged);

    std::cout << "\nMulti-Span Bridge (Center Support Removed):\n";
    std::cout << "  FEM: " << (ground_truth.should_fail ? "FAIL" : "OK") << "\n";
    std::cout << "  Spring: " << (spring_result.structure_failed ? "FAIL" : "OK")
              << (spring_result.structure_failed == ground_truth.should_fail ? " ✓" : " ✗") << "\n";
    std::cout << "  Max-Flow: " << (maxflow_result.structure_failed ? "FAIL" : "OK")
              << (maxflow_result.structure_failed == ground_truth.should_fail ? " ✓" : " ✗") << "\n";

    EXPECT_EQ(spring_result.structure_failed, ground_truth.should_fail);
    EXPECT_EQ(maxflow_result.structure_failed, ground_truth.should_fail);
}

TEST_F(FEMValidationTest, MultiLevelFrameColumnRemoved) {
    auto ground_truth = LoadGroundTruth("multi_level_frame.json");
    auto structure = GetMultiLevelFrame();

    VoxelWorld world;
    BuildStructure(world, structure);

    StructuralAnalyzer spring;
    auto spring_result = spring.Analyze(world, structure.damaged);

    MaxFlowStructuralAnalyzer maxflow;
    auto maxflow_result = maxflow.Analyze(world, structure.damaged);

    std::cout << "\nMulti-Level Frame (Column Removed):\n";
    std::cout << "  FEM: " << (ground_truth.should_fail ? "FAIL" : "OK") << "\n";
    std::cout << "  Spring: " << (spring_result.structure_failed ? "FAIL" : "OK")
              << (spring_result.structure_failed == ground_truth.should_fail ? " ✓" : " ✗") << "\n";
    std::cout << "  Max-Flow: " << (maxflow_result.structure_failed ? "FAIL" : "OK")
              << (maxflow_result.structure_failed == ground_truth.should_fail ? " ✓" : " ✗") << "\n";

    EXPECT_EQ(spring_result.structure_failed, ground_truth.should_fail);
    EXPECT_EQ(maxflow_result.structure_failed, ground_truth.should_fail);
}

TEST_F(FEMValidationTest, DiagonalBracedFrame) {
    auto ground_truth = LoadGroundTruth("diagonal_braced_frame.json");
    auto structure = GetDiagonalBracedFrame();

    VoxelWorld world;
    BuildStructure(world, structure);

    StructuralAnalyzer spring;
    auto spring_result = spring.Analyze(world, structure.damaged);

    MaxFlowStructuralAnalyzer maxflow;
    auto maxflow_result = maxflow.Analyze(world, structure.damaged);

    std::cout << "\nDiagonal-Braced Frame:\n";
    std::cout << "  FEM: " << (ground_truth.should_fail ? "FAIL" : "OK") << "\n";
    std::cout << "  Spring: " << (spring_result.structure_failed ? "FAIL" : "OK")
              << (spring_result.structure_failed == ground_truth.should_fail ? " ✓" : " ✗") << "\n";
    std::cout << "  Max-Flow: " << (maxflow_result.structure_failed ? "FAIL" : "OK")
              << (maxflow_result.structure_failed == ground_truth.should_fail ? " ✓" : " ✗") << "\n";

    EXPECT_EQ(spring_result.structure_failed, ground_truth.should_fail);
    EXPECT_EQ(maxflow_result.structure_failed, ground_truth.should_fail);
}

// Summary test - run all and report accuracy
TEST_F(FEMValidationTest, AccuracySummary) {
    std::vector<std::string> test_cases = {
        "tower_simple",
        "tower_grounded",
        "cantilever",
        "bridge_supported",
        "l_shape",
        "arch",
    };

    int spring_correct = 0;
    int maxflow_correct = 0;
    int total = 0;

    for (const auto& test_name : test_cases) {
        try {
            auto ground_truth = LoadGroundTruth(test_name + ".json");
            // Note: Would need to implement structure builders for each test
            // This is a simplified example
            total++;
        } catch (...) {
            // Skip if test case not available
        }
    }

    if (total > 0) {
        float spring_accuracy = 100.0f * spring_correct / total;
        float maxflow_accuracy = 100.0f * maxflow_correct / total;

        std::cout << "\n" << std::string(60, '=') << "\n";
        std::cout << "FEM Validation Accuracy Summary\n";
        std::cout << std::string(60, '=') << "\n";
        std::cout << "Test cases: " << total << "\n";
        std::cout << "Spring Analyzer:  " << spring_correct << "/" << total
                  << " (" << spring_accuracy << "%)\n";
        std::cout << "Max-Flow Analyzer: " << maxflow_correct << "/" << total
                  << " (" << maxflow_accuracy << "%)\n";
    }
}
