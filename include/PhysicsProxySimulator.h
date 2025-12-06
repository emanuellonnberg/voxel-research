#pragma once

#include "Vector3.h"

#include <vector>

class VoxelWorld;
struct SpringNode;

/**
 * Settings for the physics proxy burst simulation.
 *
 * These values are derived from StructuralAnalyzer parameters so that
 * designers can tune behaviour without touching the simulator directly.
 */
struct PhysicsProxySettings {
    float selection_radius;       // world-space radius for selecting nodes
    int max_bodies;               // max rigid bodies spawned in proxy
    float simulation_time;        // total simulated time (seconds)
    float time_step;              // integration timestep (seconds)
    float drop_threshold;         // meters of downward displacement before flagging failure
    float velocity_threshold;     // linear velocity magnitude before flagging failure
    float ground_level;           // ground plane height copied from analyzer parameters
    float selection_grid_size;    // quantization grid for spreading selections
    float borderline_load_ratio;  // load ratio that forces inclusion even if far
    float overload_priority_weight; // weight for overload vs. distance
    float support_priority_weight;  // weight for support-chain proximity
    int support_chain_depth;      // extra nodes to pull along the ground path
    bool record_impulse_history;  // enable detailed impulse samples
    int max_impulse_samples;      // max samples per body

    PhysicsProxySettings()
        : selection_radius(5.0f)
        , max_bodies(256)
        , simulation_time(0.35f)
        , time_step(1.0f / 120.0f)
        , drop_threshold(0.15f)
        , velocity_threshold(2.0f)
        , ground_level(0.0f)
        , selection_grid_size(0.25f)
        , borderline_load_ratio(0.85f)
        , overload_priority_weight(2.0f)
        , support_priority_weight(0.5f)
        , support_chain_depth(3)
        , record_impulse_history(false)
        , max_impulse_samples(8)
    {}
};

struct PhysicsProxyResult {
    bool ran_simulation;
    int bodies_simulated;
    float build_time_ms;
    float simulate_time_ms;
    std::vector<int> failed_node_ids;
    int bodies_reused;
    int bodies_created;
    struct BodyTelemetry {
        int node_id;
        float peak_drop;
        float peak_speed;
        float peak_impulse;
        std::vector<float> impulse_samples;

        BodyTelemetry()
            : node_id(-1)
            , peak_drop(0.0f)
            , peak_speed(0.0f)
            , peak_impulse(0.0f)
        {}
    };
    std::vector<BodyTelemetry> body_stats;

    PhysicsProxyResult()
        : ran_simulation(false)
        , bodies_simulated(0)
        , build_time_ms(0.0f)
        , simulate_time_ms(0.0f)
        , bodies_reused(0)
        , bodies_created(0)
    {}
};

/**
 * PhysicsProxySimulator
 *
 * Builds a lightweight Bullet world that mirrors a subset of StructuralAnalyzer
 * nodes and steps it for a short burst to detect overloads/instability.
 *
 * When Bullet is not available the simulator simply reports that it did not run.
 */
class PhysicsProxySimulator {
public:
    PhysicsProxySimulator() = default;

    /**
     * SelectCandidateNodes
     *
     * Public for testing so selection heuristics can be validated without Bullet.
     */
    std::vector<SpringNode*> SelectCandidateNodes(
        const PhysicsProxySettings& settings,
        VoxelWorld& world,
        const std::vector<SpringNode*>& nodes,
        const std::vector<Vector3>& damaged_positions) const;

    PhysicsProxyResult Run(
        const PhysicsProxySettings& settings,
        VoxelWorld& world,
        const std::vector<SpringNode*>& nodes,
        const std::vector<Vector3>& damaged_positions);
};
