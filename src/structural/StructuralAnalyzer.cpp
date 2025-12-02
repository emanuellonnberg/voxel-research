#include "StructuralAnalyzer.h"
#include "VoxelUtils.h"
#include <chrono>
#include <iostream>
#include <queue>
#include <algorithm>

using Clock = std::chrono::high_resolution_clock;

StructuralAnalyzer::StructuralAnalyzer()
    : world(nullptr)
    , iteration_count(0)
    , cache_hits(0)
    , cache_misses(0)
{
}

StructuralAnalyzer::~StructuralAnalyzer() {
    Clear();
}

void StructuralAnalyzer::Clear() {
    for (auto* node : nodes) {
        delete node;
    }
    nodes.clear();
    node_map.clear();
    mass_cache.clear();
    iteration_count = 0;
    cache_hits = 0;
    cache_misses = 0;
}

void StructuralAnalyzer::ClearMassCache() {
    mass_cache.clear();
    cache_hits = 0;
    cache_misses = 0;
}

void StructuralAnalyzer::InvalidateMassCache(const std::vector<Vector3>& changed_positions) {
    if (changed_positions.empty()) {
        return;
    }

    float voxel_size = world ? world->GetVoxelSize() : 0.05f;

    for (const auto& pos : changed_positions) {
        // Remove the position itself
        mass_cache.erase(pos);

        // Invalidate everything below (their supported mass changed)
        // Go down up to 100m
        for (float y = pos.y - 100.0f; y < pos.y; y += voxel_size) {
            Vector3 below_pos(pos.x, y, pos.z);
            mass_cache.erase(below_pos);
        }
    }
}

bool StructuralAnalyzer::IsGroundAnchor(const Vector3& position) const {
    return position.y <= params.ground_level;
}

void StructuralAnalyzer::BuildNodeGraph(const std::vector<Vector3>& damaged_positions) {
    // Clear existing graph
    Clear();

    if (damaged_positions.empty()) {
        return;
    }

    // Collect surface voxels near damage
    std::unordered_set<Vector3, Vector3::Hash> surface_voxels;

    // For each damaged position, check neighborhood
    for (const auto& pos : damaged_positions) {
        // Check 3x3x3 neighborhood around damage
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                for (int dz = -1; dz <= 1; dz++) {
                    Vector3 check_pos = pos + Vector3(
                        dx * world->GetVoxelSize(),
                        dy * world->GetVoxelSize(),
                        dz * world->GetVoxelSize()
                    );

                    // If voxel exists and is surface voxel, add it
                    if (world->HasVoxel(check_pos)) {
                        if (params.use_surface_only) {
                            if (world->IsSurfaceVoxel(check_pos)) {
                                surface_voxels.insert(check_pos);
                            }
                        } else {
                            // Include all voxels if not using surface-only mode
                            surface_voxels.insert(check_pos);
                        }
                    }
                }
            }
        }
    }

    // Create nodes for each surface voxel
    int id = 0;
    for (const auto& pos : surface_voxels) {
        Voxel voxel = world->GetVoxel(pos);
        bool is_anchor = IsGroundAnchor(pos);

        SpringNode* node = new SpringNode(pos, voxel.material_id, is_anchor, id++);
        nodes.push_back(node);
        node_map[pos] = node;
    }

    // Connect neighbors (6-connectivity)
    for (auto* node : nodes) {
        auto neighbor_positions = world->GetNeighbors(node->position, Connectivity::SIX);

        for (const auto& npos : neighbor_positions) {
            auto it = node_map.find(npos);
            if (it != node_map.end()) {
                node->neighbors.push_back(it->second);
            }
        }
    }

    std::cout << "[StructuralAnalyzer] Built node graph: "
              << nodes.size() << " nodes, "
              << surface_voxels.size() << " surface voxels near damage\n";
}

void StructuralAnalyzer::CalculateMassSupported() {
    // For each non-ground node, raycast upward to find supported mass
    for (auto* node : nodes) {
        if (node->is_ground_anchor) {
            node->mass_supported = 0.0f;  // Ground doesn't support weight
            continue;
        }

        // Check cache first (Day 12: Track cache performance)
        auto cache_it = mass_cache.find(node->position);
        if (cache_it != mass_cache.end()) {
            node->mass_supported = cache_it->second;
            cache_hits++;
            continue;
        }

        // Cache miss
        cache_misses++;

        // Raycast upward
        Vector3 ray_origin = node->position;
        Vector3 ray_direction(0, 1, 0);  // Up
        float max_distance = 100.0f;

        float total_mass = 0.0f;
        float voxel_size = world->GetVoxelSize();
        float voxel_volume = voxel_size * voxel_size * voxel_size;

        // Simple approach: check voxels directly above
        for (float dist = voxel_size; dist < max_distance; dist += voxel_size) {
            Vector3 check_pos = ray_origin + ray_direction * dist;

            if (world->HasVoxel(check_pos)) {
                Voxel voxel = world->GetVoxel(check_pos);
                const Material& mat = g_materials.GetMaterial(voxel.material_id);

                float voxel_mass = mat.density * voxel_volume;
                total_mass += voxel_mass;
            }
        }

        node->mass_supported = total_mass;
        mass_cache[node->position] = total_mass;
    }
}

float StructuralAnalyzer::CalculateSpringForce(const SpringNode* node) const {
    if (node->is_ground_anchor) {
        return 0.0f;  // Ground anchors don't move
    }

    const Material& mat = g_materials.GetMaterial(node->material_id);

    // Force from weight (downward)
    float weight_force = node->mass_supported * 9.81f;  // F = mg

    // Spring force from neighbors (Hooke's law)
    float spring_force = 0.0f;
    for (const auto* neighbor : node->neighbors) {
        float displacement_diff = neighbor->displacement - node->displacement;
        spring_force += mat.spring_constant * displacement_diff;
    }

    // Total force (negative = downward)
    float total_force = spring_force - weight_force;

    return total_force;
}

void StructuralAnalyzer::UpdateDisplacement(SpringNode* node, float dt) {
    if (node->is_ground_anchor) {
        return;  // Ground anchors don't move
    }

    const Material& mat = g_materials.GetMaterial(node->material_id);

    // Calculate net force
    float force = CalculateSpringForce(node);

    // F = ma, a = F/m
    // Mass is the node's own mass plus supported mass
    float voxel_volume = world->GetVoxelSize() * world->GetVoxelSize() * world->GetVoxelSize();
    float node_mass = mat.density * voxel_volume;
    float total_mass = node_mass + node->mass_supported;

    if (total_mass < 0.0001f) {
        total_mass = 0.0001f;  // Prevent division by zero
    }

    float acceleration = force / total_mass;

    // Update velocity with damping
    node->velocity += acceleration * dt;
    node->velocity *= params.damping;

    // Update displacement
    node->displacement += node->velocity * dt;
}

bool StructuralAnalyzer::CheckConvergence() const {
    // Check if all velocities are below threshold
    for (const auto* node : nodes) {
        if (std::abs(node->velocity) > params.convergence_threshold) {
            return false;
        }
    }
    return true;
}

bool StructuralAnalyzer::SolveDisplacements() {
    iteration_count = 0;

    for (int i = 0; i < params.max_iterations; i++) {
        iteration_count++;

        // Update all nodes
        for (auto* node : nodes) {
            UpdateDisplacement(node, params.timestep);
        }

        // Check convergence
        if (CheckConvergence()) {
            std::cout << "[StructuralAnalyzer] Converged in " << iteration_count << " iterations\n";
            return true;
        }
    }

    std::cout << "[StructuralAnalyzer] Did not converge (max iterations reached)\n";
    return false;
}

std::vector<SpringNode*> StructuralAnalyzer::DetectFailures() {
    std::vector<SpringNode*> failed_nodes;

    for (auto* node : nodes) {
        if (node->is_ground_anchor) {
            continue;  // Ground anchors can't fail
        }

        const Material& mat = g_materials.GetMaterial(node->material_id);

        // Check if displacement exceeds material limit
        if (std::abs(node->displacement) > mat.max_displacement) {
            node->has_failed = true;
            failed_nodes.push_back(node);
        }
    }

    std::cout << "[StructuralAnalyzer] Detected " << failed_nodes.size() << " failed nodes\n";
    return failed_nodes;
}

std::vector<VoxelCluster> StructuralAnalyzer::FindFailedClusters(
    const std::vector<SpringNode*>& failed_nodes)
{
    std::vector<VoxelCluster> clusters;

    if (failed_nodes.empty()) {
        return clusters;
    }

    // Convert failed nodes to positions
    std::vector<Vector3> failed_positions;
    for (const auto* node : failed_nodes) {
        failed_positions.push_back(node->position);
    }

    // Use existing clustering system
    // We need to create a temporary world with only failed voxels
    // Or we can manually cluster the failed nodes

    // For now, create one cluster with all failed nodes
    VoxelCluster cluster;
    cluster.voxel_positions = failed_positions;
    cluster.cluster_id = 0;

    // Calculate properties (reuse VoxelClustering if needed)
    // For now, just basic properties
    if (!failed_positions.empty()) {
        Vector3 sum = Vector3::Zero();
        for (const auto& pos : failed_positions) {
            sum += pos;
        }
        cluster.center_of_mass = sum / static_cast<float>(failed_positions.size());

        // Calculate bounds
        cluster.bounds.min = failed_positions[0];
        cluster.bounds.max = failed_positions[0];
        for (const auto& pos : failed_positions) {
            cluster.bounds.min = Vector3::Min(cluster.bounds.min, pos);
            cluster.bounds.max = Vector3::Max(cluster.bounds.max, pos);
        }

        // Estimate total mass
        float voxel_volume = world->GetVoxelSize() * world->GetVoxelSize() * world->GetVoxelSize();
        float total_mass = 0.0f;
        for (const auto* node : failed_nodes) {
            const Material& mat = g_materials.GetMaterial(node->material_id);
            total_mass += mat.density * voxel_volume;
        }
        cluster.total_mass = total_mass;
    }

    clusters.push_back(cluster);
    return clusters;
}

AnalysisResult StructuralAnalyzer::Analyze(
    VoxelWorld& voxel_world,
    const std::vector<Vector3>& damaged_positions,
    float time_budget_ms)
{
    auto start_total = Clock::now();

    AnalysisResult result;
    world = &voxel_world;

    // Step 1: Build node graph (Day 12: Profile this step)
    auto start_graph = Clock::now();
    BuildNodeGraph(damaged_positions);
    result.node_graph_time_ms = std::chrono::duration<float, std::milli>(Clock::now() - start_graph).count();

    if (nodes.empty()) {
        result.debug_info = "No nodes to analyze (no surface voxels near damage)";
        return result;
    }

    // Step 2: Calculate mass supported (Day 12: Profile this step)
    auto start_mass = Clock::now();
    CalculateMassSupported();
    result.mass_calc_time_ms = std::chrono::duration<float, std::milli>(Clock::now() - start_mass).count();
    result.cache_hits = cache_hits;
    result.cache_misses = cache_misses;

    // Step 3: Solve for displacements (Day 12: Profile this step)
    auto start_solver = Clock::now();
    result.converged = SolveDisplacements();
    result.iterations_used = iteration_count;
    result.solver_time_ms = std::chrono::duration<float, std::milli>(Clock::now() - start_solver).count();

    // Step 4: Detect failures (Day 12: Profile this step)
    auto start_failure = Clock::now();
    auto failed_nodes = DetectFailures();
    result.num_failed_nodes = static_cast<int>(failed_nodes.size());
    result.failure_detection_time_ms = std::chrono::duration<float, std::milli>(Clock::now() - start_failure).count();

    // Step 5: Find clusters
    if (!failed_nodes.empty()) {
        result.structure_failed = true;
        result.failed_clusters = FindFailedClusters(failed_nodes);
    }

    // Calculate total elapsed time
    result.calculation_time_ms = std::chrono::duration<float, std::milli>(Clock::now() - start_total).count();
    result.nodes_analyzed = static_cast<int>(nodes.size());

    // Debug info
    result.debug_info = "Analyzed " + std::to_string(nodes.size()) + " nodes in " +
                       std::to_string(result.iterations_used) + " iterations. " +
                       std::to_string(result.num_failed_nodes) + " nodes failed.";

    std::cout << "[StructuralAnalyzer] Analysis complete: "
              << result.calculation_time_ms << "ms, "
              << (result.structure_failed ? "FAILED" : "STABLE") << "\n";

    return result;
}
