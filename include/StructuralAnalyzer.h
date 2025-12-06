#pragma once

#include "Vector3.h"
#include "Material.h"
#include "VoxelWorld.h"
#include "VoxelCluster.h"
#include "PhysicsProxySimulator.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <limits>

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
    float redistributed_load;   // Mass after redistribution (kg)
    float load_capacity;        // Maximum allowable supported mass

    // Connectivity
    std::vector<SpringNode*> neighbors;
    bool is_ground_anchor;      // Touching ground (infinite support)

    // Failure tracking
    bool has_failed;            // Exceeded material limits
    bool is_disconnected;       // No path to ground
    float ground_path_length;   // Shortest path distance to ground anchors
    float vertical_stack_depth; // Continuous voxel height directly beneath

    // For debugging
    int id;

    SpringNode()
        : position(Vector3::Zero())
        , material_id(0)
        , displacement(0.0f)
        , velocity(0.0f)
        , mass_supported(0.0f)
        , redistributed_load(0.0f)
        , load_capacity(0.0f)
        , is_ground_anchor(false)
        , has_failed(false)
        , is_disconnected(false)
        , ground_path_length(std::numeric_limits<float>::infinity())
        , vertical_stack_depth(0.0f)
        , id(-1)
    {}

    SpringNode(const Vector3& pos, uint8_t mat_id, bool is_anchor, int node_id)
        : position(pos)
        , material_id(mat_id)
        , displacement(0.0f)
        , velocity(0.0f)
        , mass_supported(0.0f)
        , redistributed_load(0.0f)
        , load_capacity(0.0f)
        , is_ground_anchor(is_anchor)
        , has_failed(false)
        , is_disconnected(false)
        , ground_path_length(std::numeric_limits<float>::infinity())
        , vertical_stack_depth(0.0f)
        , id(node_id)
    {}
};

/**
 * LoadRedistributionStats - Telemetry for redistribution pass
 */
struct LoadRedistributionStats {
    bool enabled;
    int iterations;
    int nodes_over_capacity;
    int transfers;
    float total_excess_load;
    float load_dumped_to_ground;

    LoadRedistributionStats()
        : enabled(false)
        , iterations(0)
        , nodes_over_capacity(0)
        , transfers(0)
        , total_excess_load(0.0f)
        , load_dumped_to_ground(0.0f)
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
    bool proxy_mode_used;                       // True when physics proxy handled failures
    PhysicsProxyResult proxy_result;            // Telemetry from proxy simulator

    // Performance metrics
    float calculation_time_ms;
    int iterations_used;
    bool converged;

    // Detailed profiling (Day 12)
    float node_graph_time_ms;
    float mass_calc_time_ms;
    float solver_time_ms;
    float failure_detection_time_ms;
    int nodes_analyzed;
    int cache_hits;
    int cache_misses;

    // Debug info
    std::string debug_info;

    // Load redistribution telemetry (only populated for that mode)
    LoadRedistributionStats load_stats;

    AnalysisResult()
        : structure_failed(false)
        , num_failed_nodes(0)
        , num_disconnected_nodes(0)
        , proxy_mode_used(false)
        , proxy_result()
        , calculation_time_ms(0.0f)
        , iterations_used(0)
        , converged(false)
        , node_graph_time_ms(0.0f)
        , mass_calc_time_ms(0.0f)
        , solver_time_ms(0.0f)
        , failure_detection_time_ms(0.0f)
        , nodes_analyzed(0)
        , cache_hits(0)
        , cache_misses(0)
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

    /**
     * Invalidate mass cache for positions (Day 12)
     * Called when voxels are added/removed to clear affected cache entries
     */
    void InvalidateMassCache(const std::vector<Vector3>& changed_positions);

    /**
     * Clear entire mass cache (Day 12)
     */
    void ClearMassCache();

    /**
     * Get cache statistics (Day 12)
     */
    void GetCacheStats(int& hits, int& misses) const {
        hits = cache_hits;
        misses = cache_misses;
    }

    /**
     * Save parameters to INI file (Day 17)
     * @param filename Path to INI file to write
     * @return true if successful
     */
    bool SaveParameters(const std::string& filename) const;

    /**
     * Load parameters from INI file (Day 17)
     * @param filename Path to INI file to read
     * @return true if successful
     */
    bool LoadParameters(const std::string& filename);

    /**
     * Validate parameters (Day 18)
     * Clamps parameters to safe ranges and warns about invalid values
     * @return true if parameters are valid/corrected
     */
    bool ValidateParameters();

    /**
     * Check for numerical stability issues (Day 18)
     * Detects NaN, Inf, and extreme values in node states
     * @return true if numerically stable
     */
    bool CheckNumericalStability() const;

    // Analysis strategy selection
    enum class StructuralMode {
        HeuristicSupport,     // Current displacement/ground-connectivity heuristics
        StackDepth,           // Future: require per-material support stacks
        LoadRedistribution,   // Future: iterative load rebalancing
        PhysicsProxy          // Future: Bullet-based proxy simulation
    };

    // Tunable parameters
    struct Parameters {
        float timestep;                 // Simulation timestep (default 0.01s)
        float damping;                  // Velocity damping (default 0.9)
        int max_iterations;             // Max solver iterations (default 100)
        float convergence_threshold;    // Convergence check (default 0.0001m)
        float ground_level;             // Ground plane Y (default 0.0)
        bool use_surface_only;          // Analyze only surface voxels (default true)
        float influence_radius;         // Radius around damage to analyze (Day 16, default 5.0m)
        bool use_parallel_mass_calc;    // Use multi-threading for mass calculation (Day 16)
        bool use_early_termination;     // Enable early termination optimizations (Day 16)
        StructuralMode analysis_mode;   // Selects structural failure strategy
        int load_redistribution_iterations;   // Max redistribution passes
        float load_redistribution_tolerance;  // Redistribution epsilon
        float load_capacity_scale;            // Global multiplier on per-material capacity
        bool allow_ground_absorption;         // Allow nodes to dump load directly to ground
        PhysicsProxySettings proxy_settings;  // Settings for physics proxy mode

        Parameters()
            : timestep(0.01f)
            , damping(0.9f)
            , max_iterations(100)
            , convergence_threshold(0.0001f)
            , ground_level(0.0f)
            , use_surface_only(true)
            , influence_radius(5.0f)
            , use_parallel_mass_calc(true)
            , use_early_termination(false)  // Day 16: Disabled by default (too aggressive)
            , analysis_mode(StructuralMode::HeuristicSupport)
            , load_redistribution_iterations(8)
            , load_redistribution_tolerance(1e-4f)
            , load_capacity_scale(1.0f)
            , allow_ground_absorption(true)
            , proxy_settings()
        {}
    };

    Parameters params;

private:
    // Analysis pipeline
    void BuildNodeGraph(const std::vector<Vector3>& damaged_positions);
    void CalculateMassSupported();
    void RedistributeLoads();
    void CalculateStackDepth();
    bool SolveDisplacements();
    std::vector<SpringNode*> DetectFailures();
    std::vector<VoxelCluster> FindFailedClusters(const std::vector<SpringNode*>& failed_nodes);

    // Helper methods
    bool IsGroundAnchor(const Vector3& position) const;
    float CalculateSpringForce(const SpringNode* node) const;
    void UpdateDisplacement(SpringNode* node, float dt);
    bool CheckConvergence() const;
    float CalculateStableTimestep() const;
    const char* ModeToString(StructuralMode mode) const;
    StructuralMode StringToMode(const std::string& mode) const;

    // Day 14: Ground connectivity (flood-fill from ground anchors)
    std::vector<SpringNode*> FindFloatingNodes();

    // Data
    VoxelWorld* world;
    std::vector<SpringNode*> nodes;
    std::unordered_map<Vector3, SpringNode*, Vector3::Hash> node_map;
    LoadRedistributionStats load_stats;

    // Cache for mass calculation (Day 11)
    std::unordered_map<Vector3, float, Vector3::Hash> mass_cache;

    // Performance tracking (Day 11 + Day 12)
    int iteration_count;
    mutable int cache_hits;    // Day 12: Track cache performance
    mutable int cache_misses;  // Day 12: Track cache performance
};
