#pragma once

#include "Vector3.h"
#include "Material.h"
#include "VoxelWorld.h"
#include "VoxelCluster.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <string>

/**
 * SpringNode - Represents a voxel in the structural spring system
 *
 * Each node is a mass-spring system connected to neighbors.
 * Displacement under load determines structural failure.
 */
struct SpringNode {
    Vector3 position;           // World position
    uint8_t material_id;        // Material type

    // Spring physics state
    float displacement;         // Vertical displacement (meters, negative = sag down)
    float velocity;             // Rate of displacement change
    float mass_supported;       // Total mass this node supports (kg)

    // Connectivity
    std::vector<SpringNode*> neighbors;
    bool is_ground_anchor;      // Touching ground (infinite support)

    // Failure tracking
    bool has_failed;            // Exceeded material limits
    bool is_disconnected;       // No path to ground

    // For debugging
    int id;

    SpringNode()
        : position(Vector3::Zero())
        , material_id(0)
        , displacement(0.0f)
        , velocity(0.0f)
        , mass_supported(0.0f)
        , is_ground_anchor(false)
        , has_failed(false)
        , is_disconnected(false)
        , id(-1)
    {}

    SpringNode(const Vector3& pos, uint8_t mat_id, bool is_anchor, int node_id)
        : position(pos)
        , material_id(mat_id)
        , displacement(0.0f)
        , velocity(0.0f)
        , mass_supported(0.0f)
        , is_ground_anchor(is_anchor)
        , has_failed(false)
        , is_disconnected(false)
        , id(node_id)
    {}
};

/**
 * AnalysisResult - Output from structural analysis
 *
 * Contains information about structural failures and which
 * voxels should be removed or fall.
 */
struct AnalysisResult {
    bool structure_failed;                      // Did any nodes fail?
    std::vector<VoxelCluster> failed_clusters;  // Clusters to remove/simulate
    int num_failed_nodes;                       // Total nodes that failed
    int num_disconnected_nodes;                 // Nodes with no ground path

    // Performance metrics
    float calculation_time_ms;
    int iterations_used;
    bool converged;

    // Debug info
    std::string debug_info;

    AnalysisResult()
        : structure_failed(false)
        , num_failed_nodes(0)
        , num_disconnected_nodes(0)
        , calculation_time_ms(0.0f)
        , iterations_used(0)
        , converged(false)
    {}
};

/**
 * StructuralAnalyzer - Analyzes voxel structures for stability
 *
 * Uses a displacement-based spring system to simulate structural loads
 * and detect failures. This determines which voxels should collapse
 * after damage is applied.
 *
 * Algorithm:
 * 1. Build graph of surface voxels near damage
 * 2. Calculate mass each voxel supports (raycast upward)
 * 3. Solve spring displacements iteratively
 * 4. Detect failures (displacement > material limit)
 * 5. Find disconnected clusters (no ground support)
 * 6. Return clusters ready for physics simulation
 */
class StructuralAnalyzer {
public:
    StructuralAnalyzer();
    ~StructuralAnalyzer();

    /**
     * Analyze structure after damage
     *
     * @param voxel_world The voxel world to analyze
     * @param damaged_positions Positions where voxels were removed
     * @param time_budget_ms Maximum time to spend (default 500ms)
     * @return Analysis results with failed clusters
     */
    AnalysisResult Analyze(
        VoxelWorld& voxel_world,
        const std::vector<Vector3>& damaged_positions,
        float time_budget_ms = 500.0f
    );

    /**
     * Get current node count (for testing/debugging)
     */
    size_t GetNodeCount() const { return nodes.size(); }

    /**
     * Get specific node (for testing/debugging)
     */
    const SpringNode* GetNode(size_t index) const {
        if (index < nodes.size()) return nodes[index];
        return nullptr;
    }

    /**
     * Clear all nodes and state
     */
    void Clear();

    // Tunable parameters
    struct Parameters {
        float timestep;                 // Simulation timestep (default 0.01s)
        float damping;                  // Velocity damping (default 0.9)
        int max_iterations;             // Max solver iterations (default 100)
        float convergence_threshold;    // Convergence check (default 0.0001m)
        float ground_level;             // Ground plane Y (default 0.0)
        bool use_surface_only;          // Analyze only surface voxels (default true)

        Parameters()
            : timestep(0.01f)
            , damping(0.9f)
            , max_iterations(100)
            , convergence_threshold(0.0001f)
            , ground_level(0.0f)
            , use_surface_only(true)
        {}
    };

    Parameters params;

private:
    // Analysis pipeline
    void BuildNodeGraph(const std::vector<Vector3>& damaged_positions);
    void CalculateMassSupported();
    bool SolveDisplacements();
    std::vector<SpringNode*> DetectFailures();
    std::vector<VoxelCluster> FindFailedClusters(const std::vector<SpringNode*>& failed_nodes);

    // Helper methods
    bool IsGroundAnchor(const Vector3& position) const;
    float CalculateSpringForce(const SpringNode* node) const;
    void UpdateDisplacement(SpringNode* node, float dt);
    bool CheckConvergence() const;

    // Data
    VoxelWorld* world;
    std::vector<SpringNode*> nodes;
    std::unordered_map<Vector3, SpringNode*, Vector3::Hash> node_map;

    // Cache for mass calculation
    std::unordered_map<Vector3, float, Vector3::Hash> mass_cache;

    // Performance tracking
    int iteration_count;
};
