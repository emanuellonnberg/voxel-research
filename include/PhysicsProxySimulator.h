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

    PhysicsProxySettings()
        : selection_radius(5.0f)
        , max_bodies(256)
        , simulation_time(0.35f)
        , time_step(1.0f / 120.0f)
        , drop_threshold(0.15f)
        , velocity_threshold(2.0f)
        , ground_level(0.0f)
    {}
};

struct PhysicsProxyResult {
    bool ran_simulation;
    int bodies_simulated;
    float build_time_ms;
    float simulate_time_ms;
    std::vector<int> failed_node_ids;

    PhysicsProxyResult()
        : ran_simulation(false)
        , bodies_simulated(0)
        , build_time_ms(0.0f)
        , simulate_time_ms(0.0f)
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
    PhysicsProxyResult Run(
        const PhysicsProxySettings& settings,
        VoxelWorld& world,
        const std::vector<SpringNode*>& nodes,
        const std::vector<Vector3>& damaged_positions);
};
