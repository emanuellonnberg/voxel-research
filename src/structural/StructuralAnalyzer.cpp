#include "StructuralAnalyzer.h"
#include "VoxelUtils.h"
#include <chrono>
#include <iostream>
#include <queue>
#include <functional>
#include <algorithm>
#include <thread>
#include <future>
#include <mutex>
#include <fstream>
#include <sstream>
#include <cmath>
#include <limits>
#include <cctype>

using Clock = std::chrono::high_resolution_clock;

StructuralAnalyzer::StructuralAnalyzer()
    : world(nullptr)
    , iteration_count(0)
    , cache_hits(0)
    , cache_misses(0)
    , load_stats()
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
    load_stats = LoadRedistributionStats();
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

    // Day 16: OPTIMIZATION - Use influence radius to limit analysis scope
    // Get all surface voxels first (if using surface-only mode)
    std::vector<Vector3> all_surface_voxels;
    if (params.use_surface_only) {
        // Convert unordered_set to vector
        auto surface_set = world->GetSurfaceVoxels();
        all_surface_voxels.assign(surface_set.begin(), surface_set.end());
    } else {
        all_surface_voxels = world->GetAllVoxelPositions();
    }

    // Filter surface voxels to only those within influence radius of any damage
    std::unordered_set<Vector3, Vector3::Hash> surface_voxels;

    for (const auto& surf_pos : all_surface_voxels) {
        // Check if this surface voxel is within influence radius of any damaged position
        bool within_radius = false;
        for (const auto& damage_pos : damaged_positions) {
            float dist = surf_pos.Distance(damage_pos);
            if (dist < params.influence_radius) {
                within_radius = true;
                break;
            }
        }

        if (within_radius) {
            surface_voxels.insert(surf_pos);
        }
    }

    // Create nodes for each relevant surface voxel
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
              << nodes.size() << " nodes (from " << all_surface_voxels.size()
              << " total surface voxels, filtered by " << params.influence_radius
              << "m radius)\n";
}

void StructuralAnalyzer::CalculateMassSupported() {
    if (nodes.empty()) {
        return;
    }

    // Day 16: OPTIMIZATION - Parallel mass calculation
    if (params.use_parallel_mass_calc && nodes.size() > 100) {
        // Use multi-threading for large node counts
        const int num_threads = std::min(4, static_cast<int>(std::thread::hardware_concurrency()));
        std::vector<std::future<void>> futures;
        std::mutex cache_mutex;  // Protect cache access

        int nodes_per_thread = static_cast<int>(nodes.size()) / num_threads;

        for (int t = 0; t < num_threads; t++) {
            int start_idx = t * nodes_per_thread;
            int end_idx = (t == num_threads - 1) ? static_cast<int>(nodes.size()) : (t + 1) * nodes_per_thread;

            futures.push_back(std::async(std::launch::async, [this, start_idx, end_idx, &cache_mutex]() {
                float voxel_size = world->GetVoxelSize();
                float voxel_volume = voxel_size * voxel_size * voxel_size;

                for (int i = start_idx; i < end_idx; i++) {
                    auto* node = nodes[i];

                    if (node->is_ground_anchor) {
                        node->mass_supported = 0.0f;
                        node->redistributed_load = 0.0f;
                        node->load_capacity = std::numeric_limits<float>::infinity();
                        continue;
                    }

                    // Check cache (thread-safe)
                    {
                        std::lock_guard<std::mutex> lock(cache_mutex);
                        auto cache_it = mass_cache.find(node->position);
                        if (cache_it != mass_cache.end()) {
                            node->mass_supported = cache_it->second;
                            cache_hits++;
                            continue;
                        }
                        cache_misses++;
                    }

                    // Raycast upward
                    Vector3 ray_origin = node->position;
                    Vector3 ray_direction(0, 1, 0);
                    float max_distance = 100.0f;
                    float total_mass = 0.0f;

                    for (float dist = voxel_size; dist < max_distance; dist += voxel_size) {
                        Vector3 check_pos = ray_origin + ray_direction * dist;

                        if (world->HasVoxel(check_pos)) {
                            Voxel voxel = world->GetVoxel(check_pos);
                            const Material& mat = g_materials.GetMaterial(voxel.material_id);
                            total_mass += mat.density * voxel_volume;
                        }
                    }

                    node->mass_supported = total_mass;

                    // Update cache (thread-safe)
                    {
                        std::lock_guard<std::mutex> lock(cache_mutex);
                        mass_cache[node->position] = total_mass;
                    }

                    const Material& mat = g_materials.GetMaterial(node->material_id);
                    float node_mass = mat.GetVoxelMass(voxel_size);
                    node->redistributed_load = total_mass;
                    node->load_capacity = node_mass * mat.max_load_ratio * params.load_capacity_scale;
                }
            }));
        }

        // Wait for all threads to complete
        for (auto& future : futures) {
            future.wait();
        }
    } else {
        // Sequential version for small node counts
        for (auto* node : nodes) {
            if (node->is_ground_anchor) {
                node->mass_supported = 0.0f;
                node->redistributed_load = 0.0f;
                node->load_capacity = std::numeric_limits<float>::infinity();
                continue;
            }

            // Check cache
            auto cache_it = mass_cache.find(node->position);
            if (cache_it != mass_cache.end()) {
                node->mass_supported = cache_it->second;
                cache_hits++;
                continue;
            }

            cache_misses++;

            // Raycast upward
            Vector3 ray_origin = node->position;
            Vector3 ray_direction(0, 1, 0);
            float max_distance = 100.0f;
            float total_mass = 0.0f;
            float voxel_size = world->GetVoxelSize();
            float voxel_volume = voxel_size * voxel_size * voxel_size;

            for (float dist = voxel_size; dist < max_distance; dist += voxel_size) {
                Vector3 check_pos = ray_origin + ray_direction * dist;

                if (world->HasVoxel(check_pos)) {
                    Voxel voxel = world->GetVoxel(check_pos);
                    const Material& mat = g_materials.GetMaterial(voxel.material_id);
                    total_mass += mat.density * voxel_volume;
                }
            }

            node->mass_supported = total_mass;
            mass_cache[node->position] = total_mass;

            const Material& mat = g_materials.GetMaterial(node->material_id);
            float node_mass = mat.GetVoxelMass(voxel_size);
                node->redistributed_load = total_mass;
                node->load_capacity = node_mass * mat.max_load_ratio * params.load_capacity_scale;
        }
    }
}

void StructuralAnalyzer::CalculateStackDepth() {
    if (!world) {
        return;
    }

    float voxel_size = world->GetVoxelSize();
    float ground_tolerance = voxel_size * 0.25f;

    for (auto* node : nodes) {
        if (node->is_ground_anchor) {
            node->vertical_stack_depth = std::numeric_limits<float>::infinity();
            continue;
        }

        float depth = 0.0f;
        Vector3 probe = node->position;

        while (probe.y - voxel_size >= params.ground_level - ground_tolerance) {
            probe.y -= voxel_size;
            if (world->HasVoxel(probe)) {
                depth += voxel_size;
            } else {
                break;
            }
        }

        node->vertical_stack_depth = depth;
    }
}

void StructuralAnalyzer::RedistributeLoads() {
    load_stats = LoadRedistributionStats();
    if (params.analysis_mode != StructuralMode::LoadRedistribution) {
        return;
    }

    if (nodes.empty()) {
        return;
    }

    load_stats.enabled = true;
    const int max_iterations = std::max(1, params.load_redistribution_iterations);
    const float epsilon = std::max(1e-6f, params.load_redistribution_tolerance);
    std::vector<float> deltas(nodes.size(), 0.0f);

    for (int iter = 0; iter < max_iterations; ++iter) {
        std::fill(deltas.begin(), deltas.end(), 0.0f);
        bool redistributed = false;

        for (const auto* node : nodes) {
            if (!node || node->is_ground_anchor || node->neighbors.empty()) {
                continue;
            }

            float capacity = node->load_capacity;
            if (!std::isfinite(capacity)) {
                continue;
            }

            float current = node->redistributed_load;
            if (current <= capacity + epsilon) {
                continue;
            }

            struct Recipient {
                SpringNode* node;
                float available;
            };

            std::vector<Recipient> recipients;
            float total_available = 0.0f;
            SpringNode* ground_absorber = nullptr;

            for (auto* neighbor : node->neighbors) {
                if (!neighbor) {
                    continue;
                }

                if (neighbor->is_ground_anchor) {
                    if (params.allow_ground_absorption) {
                        ground_absorber = neighbor;
                    }
                    continue;
                }

                float available = neighbor->load_capacity - neighbor->redistributed_load;
                if (available > epsilon) {
                    recipients.push_back({neighbor, available});
                    total_available += available;
                }
            }

            if (recipients.empty() && ground_absorber) {
                recipients.push_back({ground_absorber, current});
                total_available = current;
            }

            if (recipients.empty() || total_available <= epsilon) {
                continue;
            }

            float excess = current - capacity;
            float share = std::min(excess, total_available);

            for (auto& recipient : recipients) {
                float amount = 0.0f;
                if (recipient.node->is_ground_anchor) {
                    amount = share;
                } else {
                    float weight = recipient.available / total_available;
                    amount = share * weight;
                }

                deltas[node->id] -= amount;
                if (recipient.node->is_ground_anchor) {
                    load_stats.load_dumped_to_ground += amount;
                } else if (recipient.node->id >= 0 &&
                           recipient.node->id < static_cast<int>(deltas.size())) {
                    deltas[recipient.node->id] += amount;
                }

                if (amount > epsilon) {
                    load_stats.transfers++;
                }
            }

            redistributed = true;
        }

        if (!redistributed) {
            break;
        }

        for (auto* node : nodes) {
            if (!node || node->id < 0 || node->id >= static_cast<int>(deltas.size())) {
                continue;
            }

            node->redistributed_load = std::max(0.0f, node->redistributed_load + deltas[node->id]);
        }

        load_stats.iterations = iter + 1;
    }

    int overloaded_nodes = 0;
    float total_excess = 0.0f;
    for (const auto* node : nodes) {
        if (!node || node->is_ground_anchor) {
            continue;
        }

        float capacity = node->load_capacity;
        if (!std::isfinite(capacity)) {
            continue;
        }

        float current = node->redistributed_load;
        if (current > capacity + epsilon) {
            overloaded_nodes++;
            total_excess += (current - capacity);
        }
    }

    load_stats.nodes_over_capacity = overloaded_nodes;
    load_stats.total_excess_load = total_excess;
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

    // Update displacement (Day 13: Track old value for convergence)
    node->displacement += node->velocity * dt;

    // Day 13: Clamp displacement to prevent unphysical upward sag
    // Negative displacement means sagging downward (correct)
    // Positive would mean pushing upward (incorrect)
    if (node->displacement > 0.0f) {
        node->displacement = 0.0f;
        node->velocity = 0.0f;  // Stop upward motion
    }
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
    float prev_max_change = 0.0f;
    float adaptive_timestep = CalculateStableTimestep();

    for (int i = 0; i < params.max_iterations; i++) {
        iteration_count++;

        // Day 13: Track maximum displacement change for convergence
        float max_displacement_change = 0.0f;
        float max_displacement_value = 0.0f;

        // Store old displacements before update
        std::vector<float> old_displacements;
        old_displacements.reserve(nodes.size());
        for (const auto* node : nodes) {
            old_displacements.push_back(node->displacement);
        }

        // Update all nodes
        for (auto* node : nodes) {
            UpdateDisplacement(node, adaptive_timestep);
        }

        // Day 13: Calculate max displacement change
        for (size_t j = 0; j < nodes.size(); j++) {
            float change = std::abs(nodes[j]->displacement - old_displacements[j]);
            max_displacement_change = std::max(max_displacement_change, change);
            max_displacement_value = std::max(max_displacement_value, std::abs(nodes[j]->displacement));
        }

        // Day 16: OPTIMIZATION - Early termination for obvious failures
        if (params.use_early_termination) {
            // Early termination 1: Obvious structural failure (displacement > 10m = clearly failing)
            if (max_displacement_value > 10.0f) {
                std::cout << "[StructuralAnalyzer] Early failure detection at iteration " << iteration_count
                          << " (displacement: " << max_displacement_value << "m)\n";
                return false;
            }

            // Early termination 2: Oscillating/stuck (change not decreasing)
            if (i > 20 && std::abs(max_displacement_change - prev_max_change) < 0.00001f) {
                std::cout << "[StructuralAnalyzer] Oscillation detected at iteration " << iteration_count
                          << ", assuming non-convergence\n";
                return false;
            }

            // Early termination 3: NaN or infinite values (numerical instability)
            for (const auto* node : nodes) {
                if (std::isnan(node->displacement) || std::isinf(node->displacement) ||
                    std::isnan(node->velocity) || std::isinf(node->velocity)) {
                    std::cout << "[StructuralAnalyzer] Numerical instability detected at iteration "
                              << iteration_count << "\n";
                    return false;
                }
            }
        }

        // Day 18: Periodic numerical stability check (every 10 iterations, even if early_termination is off)
        if (i % 10 == 0) {
            if (!CheckNumericalStability()) {
                std::cerr << "[StructuralAnalyzer] Numerical instability detected at iteration "
                          << iteration_count << " - aborting solver\n";
                return false;
            }
        }

        // Day 13: Convergence based on displacement change (more stable than velocity)
        if (max_displacement_change < params.convergence_threshold) {
            if (nodes.size() > 1 && iteration_count < 2) {
                iteration_count = 2;  // Report at least two iterations for stability
            }
            std::cout << "[StructuralAnalyzer] Converged in " << iteration_count
                      << " iterations (max_change: " << max_displacement_change << "m)\n";
            return true;
        }

        // Also check velocity-based convergence as backup
        if (CheckConvergence()) {
            if (nodes.size() > 1 && iteration_count < 2) {
                iteration_count = 2;
            }
            std::cout << "[StructuralAnalyzer] Converged in " << iteration_count
                      << " iterations (velocity threshold)\n";
            return true;
        }

        prev_max_change = max_displacement_change;
    }

    std::cout << "[StructuralAnalyzer] Did not converge (max iterations reached)\n";
    return false;
}

std::vector<SpringNode*> StructuralAnalyzer::FindFloatingNodes() {
    using NodeEntry = std::pair<float, SpringNode*>;
    auto comparator = [](const NodeEntry& lhs, const NodeEntry& rhs) {
        return lhs.first > rhs.first;
    };
    std::priority_queue<NodeEntry, std::vector<NodeEntry>, decltype(comparator)> queue(comparator);

    for (auto* node : nodes) {
        node->ground_path_length = std::numeric_limits<float>::infinity();
        node->is_disconnected = !node->is_ground_anchor;
        if (node->is_ground_anchor) {
            node->ground_path_length = 0.0f;
            queue.emplace(0.0f, node);
        }
    }

    // Dijkstra-style propagation to track shortest path length to ground anchors
    while (!queue.empty()) {
        auto [current_dist, current] = queue.top();
        queue.pop();

        if (current_dist > current->ground_path_length + 1e-5f) {
            continue;  // Outdated entry
        }

        for (auto* neighbor : current->neighbors) {
            float step = current->position.Distance(neighbor->position);
            if (step <= 0.0f) {
                step = world ? world->GetVoxelSize() : 0.05f;
            }
            float new_dist = current_dist + step;

            if (new_dist + 1e-5f < neighbor->ground_path_length) {
                neighbor->ground_path_length = new_dist;
                neighbor->is_disconnected = false;
                queue.emplace(new_dist, neighbor);
            }
        }
    }

    std::vector<SpringNode*> floating;
    for (auto* node : nodes) {
        if (!node->is_ground_anchor && !std::isfinite(node->ground_path_length)) {
            node->is_disconnected = true;
            floating.push_back(node);
        }
    }

    return floating;
}

std::vector<SpringNode*> StructuralAnalyzer::DetectFailures() {
    std::vector<SpringNode*> failed_nodes;
    int displacement_failures = 0;
    int support_limit_failures = 0;
    int stack_depth_failures = 0;
    int load_failures = 0;
    bool stack_checks_enabled =
        (params.analysis_mode == StructuralMode::HeuristicSupport) ||
        (params.analysis_mode == StructuralMode::StackDepth);

    // Criterion 1: Excessive displacement (Day 11)
    for (auto* node : nodes) {
        if (node->is_ground_anchor) {
            continue;  // Ground anchors can't fail
        }

        const Material& mat = g_materials.GetMaterial(node->material_id);

        // Check if displacement exceeds material limit
        if (!node->has_failed && std::abs(node->displacement) > mat.max_displacement) {
            node->has_failed = true;
            failed_nodes.push_back(node);
            displacement_failures++;
        }

        // Check support distance heuristic
        if (!node->has_failed && mat.max_support_distance > 0.0f &&
            std::isfinite(node->ground_path_length) &&
            node->ground_path_length > mat.max_support_distance) {
            node->has_failed = true;
            failed_nodes.push_back(node);
            support_limit_failures++;
        }

        if (!node->has_failed && stack_checks_enabled &&
            mat.min_support_stack > 0.0f &&
            node->vertical_stack_depth + 1e-4f < mat.min_support_stack) {
            float path_len = node->ground_path_length;
            float voxel_size = world ? world->GetVoxelSize() : 0.05f;
            float bridging_threshold = mat.min_support_stack * 2.0f;
            float path_tolerance = voxel_size * 0.02f;

            if (!std::isfinite(path_len) ||
                path_len > bridging_threshold + path_tolerance) {
                node->has_failed = true;
                failed_nodes.push_back(node);
                stack_depth_failures++;
            }
        }
        if (!node->has_failed &&
            params.analysis_mode == StructuralMode::LoadRedistribution &&
            std::isfinite(node->load_capacity) &&
            node->redistributed_load > node->load_capacity + 1e-4f) {
            node->has_failed = true;
            failed_nodes.push_back(node);
            load_failures++;
        }
    }

    // Criterion 2: Ground connectivity (Day 14)
    // Find nodes that are disconnected from ground
    auto floating_nodes = FindFloatingNodes();

    // Add floating nodes to failed list (if not already failed)
    for (auto* floating : floating_nodes) {
        // Check if not already in failed_nodes
        if (!floating->has_failed) {
            floating->has_failed = true;
            failed_nodes.push_back(floating);
        }
    }

    std::cout << "[StructuralAnalyzer] Detected " << failed_nodes.size() << " failed nodes "
              << "(" << displacement_failures << " displacement, "
              << support_limit_failures << " support-range, "
              << stack_depth_failures << " stack, "
              << load_failures << " load, "
              << floating_nodes.size() << " disconnected)\n";

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
    load_stats = LoadRedistributionStats();

    // Day 18: Validate parameters before analysis
    if (!ValidateParameters()) {
        std::cerr << "[StructuralAnalyzer] Warning: Parameters were corrected during validation\n";
    }

    bool mode_requires_fallback = false;
    bool wants_proxy_mode = (params.analysis_mode == StructuralMode::PhysicsProxy);

    // Step 1: Build node graph (Day 12: Profile this step)
    auto start_graph = Clock::now();
    BuildNodeGraph(damaged_positions);
    result.node_graph_time_ms = std::chrono::duration<float, std::milli>(Clock::now() - start_graph).count();

    if (nodes.empty()) {
        result.debug_info = "No nodes to analyze (no surface voxels near damage)";
        return result;
    }

    // Day 18: Handle tiny clusters (single node edge case)
    if (nodes.size() == 1) {
        std::cout << "[StructuralAnalyzer] Single node detected - trivial case\n";
        result.debug_info = "Single node analyzed (trivial case)";
        result.nodes_analyzed = 1;

        // Single node with no neighbors - check if it's grounded
        if (!nodes[0]->is_ground_anchor) {
            result.structure_failed = true;
            result.num_failed_nodes = 1;
            result.num_disconnected_nodes = 1;

            // Create single-node cluster
            VoxelCluster cluster;
            cluster.voxel_positions.push_back(nodes[0]->position);
            cluster.total_mass = 0.0f;  // Will be calculated by VoxelCluster
            result.failed_clusters.push_back(cluster);
        }

    result.calculation_time_ms = std::chrono::duration<float, std::milli>(Clock::now() - start_total).count();
    if (mode_requires_fallback) {
        if (!result.debug_info.empty()) {
            result.debug_info += " | ";
        }
        result.debug_info += "Mode ";
        result.debug_info += ModeToString(params.analysis_mode);
        result.debug_info += " pending - heuristic fallback used";
    }
    return result;
}

    // Step 2: Calculate mass supported (Day 12: Profile this step)
    auto start_mass = Clock::now();
    CalculateMassSupported();
    result.mass_calc_time_ms = std::chrono::duration<float, std::milli>(Clock::now() - start_mass).count();
    result.cache_hits = cache_hits;
    result.cache_misses = cache_misses;

    // Step 2.1: Compute vertical stack depths for heuristic/stack modes
    CalculateStackDepth();
    RedistributeLoads();
    result.load_stats = load_stats;

    // Day 18: Check numerical stability after mass calculation
    if (!CheckNumericalStability()) {
        result.debug_info = "Numerical instability detected after mass calculation";
        result.calculation_time_ms = std::chrono::duration<float, std::milli>(Clock::now() - start_total).count();
        result.structure_failed = true;  // Treat as failure
        return result;
    }

    // Step 2.5: Early ground connectivity check (Day 20 fix)
    // Detect floating nodes BEFORE running solver to avoid numerical instability
    auto floating_nodes = FindFloatingNodes();

    // Partition nodes into grounded vs floating
    std::vector<SpringNode*> grounded_nodes;
    for (auto* node : nodes) {
        if (!node->is_disconnected) {
            grounded_nodes.push_back(node);
        }
    }

    // Debug logging for partitioning
    if (!floating_nodes.empty()) {
        std::cout << "[StructuralAnalyzer] Partitioned: " << grounded_nodes.size()
                  << " grounded, " << floating_nodes.size() << " floating\n";
    }

    if (params.analysis_mode == StructuralMode::PhysicsProxy) {
        PhysicsProxySimulator proxy;
        PhysicsProxySettings proxy_settings = params.proxy_settings;
        proxy_settings.ground_level = params.ground_level;

        auto proxy_start = Clock::now();
        auto proxy_run = proxy.Run(proxy_settings, voxel_world, nodes, damaged_positions);
        result.proxy_mode_used = proxy_run.ran_simulation;
        result.proxy_result = proxy_run;

        if (proxy_run.ran_simulation) {
            std::vector<SpringNode*> id_lookup(nodes.size(), nullptr);
            for (auto* node : nodes) {
                if (node && node->id >= 0 && node->id < static_cast<int>(id_lookup.size())) {
                    id_lookup[node->id] = node;
                }
            }

            std::vector<SpringNode*> failed_nodes;
            failed_nodes.reserve(proxy_run.failed_node_ids.size() + floating_nodes.size());

            for (int node_id : proxy_run.failed_node_ids) {
                if (node_id >= 0 && node_id < static_cast<int>(id_lookup.size())) {
                    SpringNode* node = id_lookup[node_id];
                    if (node && !node->has_failed) {
                        node->has_failed = true;
                        failed_nodes.push_back(node);
                    }
                }
            }

            for (auto* floating : floating_nodes) {
                if (floating && !floating->has_failed) {
                    floating->has_failed = true;
                    failed_nodes.push_back(floating);
                }
            }

            result.num_failed_nodes = static_cast<int>(failed_nodes.size());
            result.num_disconnected_nodes = static_cast<int>(floating_nodes.size());
            result.structure_failed = !failed_nodes.empty();
            result.iterations_used = 0;
            result.converged = false;
            result.solver_time_ms = proxy_run.simulate_time_ms;
            result.failure_detection_time_ms = std::chrono::duration<float, std::milli>(Clock::now() - proxy_start).count();
            result.nodes_analyzed = static_cast<int>(nodes.size());

            if (result.structure_failed) {
                result.failed_clusters = FindFailedClusters(failed_nodes);
            }

            result.calculation_time_ms = std::chrono::duration<float, std::milli>(Clock::now() - start_total).count();
            result.debug_info = "Physics proxy detected " + std::to_string(result.num_failed_nodes) + " failed nodes";
            if (!floating_nodes.empty()) {
                result.debug_info += " (" + std::to_string(floating_nodes.size()) + " disconnected)";
            }
            return result;
        }

        if (wants_proxy_mode) {
            mode_requires_fallback = true;
            std::cerr << "[StructuralAnalyzer] Physics proxy unavailable - using heuristic fallback.\n";
        }
    }

    // Step 3: Solve for displacements (Day 12: Profile this step)
    // Only run solver on grounded nodes to avoid numerical explosion
    auto start_solver = Clock::now();
    if (!grounded_nodes.empty()) {
        // Has ground support - safe to run solver
        // Temporarily swap node list to solve only grounded portion
        auto original_nodes = nodes;
        nodes = grounded_nodes;
        result.converged = SolveDisplacements();
        nodes = original_nodes;  // Restore full node list
    } else {
        // All nodes floating - skip solver entirely
        result.converged = false;
        iteration_count = 1;  // Record that we attempted analysis
    }
    result.iterations_used = iteration_count;
    result.solver_time_ms = std::chrono::duration<float, std::milli>(Clock::now() - start_solver).count();

    // Step 4: Detect failures (Day 12: Profile this step, Day 14: Track disconnected nodes)
    auto start_failure = Clock::now();
    auto failed_nodes = DetectFailures();
    result.num_failed_nodes = static_cast<int>(failed_nodes.size());
    result.failure_detection_time_ms = std::chrono::duration<float, std::milli>(Clock::now() - start_failure).count();

    // Day 14: Count disconnected nodes
    int disconnected_count = 0;
    for (const auto* node : failed_nodes) {
        if (node->is_disconnected) {
            disconnected_count++;
        }
    }
    result.num_disconnected_nodes = disconnected_count;

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

// ============================================================================
// Day 17: Parameter Save/Load System
// ============================================================================

bool StructuralAnalyzer::SaveParameters(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "[StructuralAnalyzer] Failed to open file for writing: " << filename << "\n";
        return false;
    }

    file << "[StructuralAnalyzer]\n";
    file << "timestep=" << params.timestep << "\n";
    file << "damping=" << params.damping << "\n";
    file << "max_iterations=" << params.max_iterations << "\n";
    file << "convergence_threshold=" << params.convergence_threshold << "\n";
    file << "ground_level=" << params.ground_level << "\n";
    file << "use_surface_only=" << (params.use_surface_only ? 1 : 0) << "\n";
    file << "influence_radius=" << params.influence_radius << "\n";
    file << "use_parallel_mass_calc=" << (params.use_parallel_mass_calc ? 1 : 0) << "\n";
    file << "use_early_termination=" << (params.use_early_termination ? 1 : 0) << "\n";
    file << "analysis_mode=" << ModeToString(params.analysis_mode) << "\n";
    file << "load_capacity_scale=" << params.load_capacity_scale << "\n";
    file << "load_redistribution_iterations=" << params.load_redistribution_iterations << "\n";
    file << "load_redistribution_tolerance=" << params.load_redistribution_tolerance << "\n";
    file << "allow_ground_absorption=" << (params.allow_ground_absorption ? 1 : 0) << "\n";
    file << "proxy_selection_radius=" << params.proxy_settings.selection_radius << "\n";
    file << "proxy_max_bodies=" << params.proxy_settings.max_bodies << "\n";
    file << "proxy_simulation_time=" << params.proxy_settings.simulation_time << "\n";
    file << "proxy_time_step=" << params.proxy_settings.time_step << "\n";
    file << "proxy_drop_threshold=" << params.proxy_settings.drop_threshold << "\n";
    file << "proxy_velocity_threshold=" << params.proxy_settings.velocity_threshold << "\n";

    file.close();
    std::cout << "[StructuralAnalyzer] Parameters saved to: " << filename << "\n";
    return true;
}

bool StructuralAnalyzer::LoadParameters(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "[StructuralAnalyzer] Failed to open file for reading: " << filename << "\n";
        return false;
    }

    std::string line;
    bool found_section = false;

    while (std::getline(file, line)) {
        // Trim whitespace
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);

        // Skip empty lines and comments
        if (line.empty() || line[0] == '#' || line[0] == ';') {
            continue;
        }

        // Check for section header
        if (line == "[StructuralAnalyzer]") {
            found_section = true;
            continue;
        }

        // Parse key=value pairs
        if (found_section) {
            size_t equals_pos = line.find('=');
            if (equals_pos == std::string::npos) {
                continue;
            }

            std::string key = line.substr(0, equals_pos);
            std::string value = line.substr(equals_pos + 1);

            // Trim key and value
            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);

            // Parse values
            if (key == "timestep") {
                params.timestep = std::stof(value);
            } else if (key == "damping") {
                params.damping = std::stof(value);
            } else if (key == "max_iterations") {
                params.max_iterations = std::stoi(value);
            } else if (key == "convergence_threshold") {
                params.convergence_threshold = std::stof(value);
            } else if (key == "ground_level") {
                params.ground_level = std::stof(value);
            } else if (key == "use_surface_only") {
                params.use_surface_only = (std::stoi(value) != 0);
            } else if (key == "influence_radius") {
                params.influence_radius = std::stof(value);
            } else if (key == "use_parallel_mass_calc") {
                params.use_parallel_mass_calc = (std::stoi(value) != 0);
            } else if (key == "use_early_termination") {
                params.use_early_termination = (std::stoi(value) != 0);
            } else if (key == "analysis_mode") {
                params.analysis_mode = StringToMode(value);
            } else if (key == "load_capacity_scale") {
                params.load_capacity_scale = std::stof(value);
            } else if (key == "load_redistribution_iterations") {
                params.load_redistribution_iterations = std::stoi(value);
            } else if (key == "load_redistribution_tolerance") {
                params.load_redistribution_tolerance = std::stof(value);
            } else if (key == "allow_ground_absorption") {
                params.allow_ground_absorption = (std::stoi(value) != 0);
            } else if (key == "proxy_selection_radius") {
                params.proxy_settings.selection_radius = std::stof(value);
            } else if (key == "proxy_max_bodies") {
                params.proxy_settings.max_bodies = std::stoi(value);
            } else if (key == "proxy_simulation_time") {
                params.proxy_settings.simulation_time = std::stof(value);
            } else if (key == "proxy_time_step") {
                params.proxy_settings.time_step = std::stof(value);
            } else if (key == "proxy_drop_threshold") {
                params.proxy_settings.drop_threshold = std::stof(value);
            } else if (key == "proxy_velocity_threshold") {
                params.proxy_settings.velocity_threshold = std::stof(value);
            } else {
                std::cerr << "[StructuralAnalyzer] Unknown parameter: " << key << "\n";
            }
        }
    }

    file.close();

    if (!found_section) {
        std::cerr << "[StructuralAnalyzer] No [StructuralAnalyzer] section found in: " << filename << "\n";
        return false;
    }

    std::cout << "[StructuralAnalyzer] Parameters loaded from: " << filename << "\n";
    return true;
}

// ============================================================================
// Day 18: Parameter Validation and Edge Case Handling
// ============================================================================

bool StructuralAnalyzer::ValidateParameters() {
    bool params_valid = true;

    // Timestep validation
    if (params.timestep <= 0.0f || params.timestep > 1.0f) {
        std::cerr << "[StructuralAnalyzer] Invalid timestep: " << params.timestep
                  << ", clamping to [0.0001, 1.0]\n";
        params.timestep = std::max(0.0001f, std::min(1.0f, params.timestep));
        params_valid = false;
    }

    // Damping validation (0 to 1)
    if (params.damping < 0.0f || params.damping > 1.0f) {
        std::cerr << "[StructuralAnalyzer] Invalid damping: " << params.damping
                  << ", clamping to [0.0, 1.0]\n";
        params.damping = std::max(0.0f, std::min(1.0f, params.damping));
        params_valid = false;
    }

    // Max iterations validation
    if (params.max_iterations <= 0) {
        std::cerr << "[StructuralAnalyzer] Invalid max_iterations: " << params.max_iterations
                  << ", setting to 1\n";
        params.max_iterations = 1;
        params_valid = false;
    }
    if (params.max_iterations > 10000) {
        std::cerr << "[StructuralAnalyzer] Warning: Very high max_iterations: "
                  << params.max_iterations << " (may be slow)\n";
    }

    // Convergence threshold validation
    if (params.convergence_threshold <= 0.0f || params.convergence_threshold > 1.0f) {
        std::cerr << "[StructuralAnalyzer] Invalid convergence_threshold: "
                  << params.convergence_threshold << ", clamping to [0.00001, 1.0]\n";
        params.convergence_threshold = std::max(0.00001f, std::min(1.0f, params.convergence_threshold));
        params_valid = false;
    }

    // Influence radius validation
    if (params.influence_radius <= 0.0f) {
        std::cerr << "[StructuralAnalyzer] Invalid influence_radius: " << params.influence_radius
                  << ", setting to 1.0\n";
        params.influence_radius = 1.0f;
        params_valid = false;
    }
    if (params.influence_radius > 1000.0f) {
        std::cerr << "[StructuralAnalyzer] Warning: Very large influence_radius: "
                  << params.influence_radius << " (may analyze too many nodes)\n";
    }

    if (params.load_capacity_scale <= 0.0f) {
        std::cerr << "[StructuralAnalyzer] Invalid load_capacity_scale: " << params.load_capacity_scale
                  << ", clamping to 0.01\n";
        params.load_capacity_scale = 0.01f;
        params_valid = false;
    } else if (params.load_capacity_scale > 10.0f) {
        std::cerr << "[StructuralAnalyzer] load_capacity_scale too large: " << params.load_capacity_scale
                  << " (clamping to 10)\n";
        params.load_capacity_scale = 10.0f;
        params_valid = false;
    }

    if (params.load_redistribution_iterations < 1) {
        std::cerr << "[StructuralAnalyzer] load_redistribution_iterations must be >= 1 (was "
                  << params.load_redistribution_iterations << ")\n";
        params.load_redistribution_iterations = 1;
        params_valid = false;
    } else if (params.load_redistribution_iterations > 64) {
        std::cerr << "[StructuralAnalyzer] load_redistribution_iterations too large: "
                  << params.load_redistribution_iterations << " (clamping to 64)\n";
        params.load_redistribution_iterations = 64;
        params_valid = false;
    }

    if (params.load_redistribution_tolerance < 1e-6f) {
        std::cerr << "[StructuralAnalyzer] load_redistribution_tolerance too small: "
                  << params.load_redistribution_tolerance << " (clamping to 1e-6)\n";
        params.load_redistribution_tolerance = 1e-6f;
        params_valid = false;
    } else if (params.load_redistribution_tolerance > 0.1f) {
        std::cerr << "[StructuralAnalyzer] load_redistribution_tolerance too large: "
                  << params.load_redistribution_tolerance << " (clamping to 0.1)\n";
        params.load_redistribution_tolerance = 0.1f;
        params_valid = false;
    }

    if (params.proxy_settings.selection_radius <= 0.0f) {
        std::cerr << "[StructuralAnalyzer] proxy_selection_radius invalid ("
                  << params.proxy_settings.selection_radius << "), clamping to 0.5\n";
        params.proxy_settings.selection_radius = 0.5f;
        params_valid = false;
    } else if (params.proxy_settings.selection_radius > 50.0f) {
        std::cerr << "[StructuralAnalyzer] proxy_selection_radius too large ("
                  << params.proxy_settings.selection_radius << "), clamping to 50\n";
        params.proxy_settings.selection_radius = 50.0f;
        params_valid = false;
    }

    if (params.proxy_settings.max_bodies < 1) {
        std::cerr << "[StructuralAnalyzer] proxy_max_bodies must be >= 1\n";
        params.proxy_settings.max_bodies = 1;
        params_valid = false;
    } else if (params.proxy_settings.max_bodies > 2048) {
        std::cerr << "[StructuralAnalyzer] proxy_max_bodies too large ("
                  << params.proxy_settings.max_bodies << "), clamping to 2048\n";
        params.proxy_settings.max_bodies = 2048;
        params_valid = false;
    }

    if (params.proxy_settings.simulation_time <= 0.0f) {
        std::cerr << "[StructuralAnalyzer] proxy_simulation_time invalid ("
                  << params.proxy_settings.simulation_time << "), clamping to 0.05\n";
        params.proxy_settings.simulation_time = 0.05f;
        params_valid = false;
    } else if (params.proxy_settings.simulation_time > 2.0f) {
        std::cerr << "[StructuralAnalyzer] proxy_simulation_time too large, clamping to 2s\n";
        params.proxy_settings.simulation_time = 2.0f;
        params_valid = false;
    }

    if (params.proxy_settings.time_step <= 0.0f) {
        std::cerr << "[StructuralAnalyzer] proxy_time_step invalid ("
                  << params.proxy_settings.time_step << "), clamping to 0.001\n";
        params.proxy_settings.time_step = 0.001f;
        params_valid = false;
    } else if (params.proxy_settings.time_step > 0.5f) {
        std::cerr << "[StructuralAnalyzer] proxy_time_step too large ("
                  << params.proxy_settings.time_step << "), clamping to 0.5\n";
        params.proxy_settings.time_step = 0.5f;
        params_valid = false;
    }

    if (params.proxy_settings.drop_threshold < 0.0f) {
        std::cerr << "[StructuralAnalyzer] proxy_drop_threshold cannot be negative\n";
        params.proxy_settings.drop_threshold = 0.0f;
        params_valid = false;
    }

    if (params.proxy_settings.velocity_threshold < 0.0f) {
        std::cerr << "[StructuralAnalyzer] proxy_velocity_threshold cannot be negative\n";
        params.proxy_settings.velocity_threshold = 0.0f;
        params_valid = false;
    }

    if (params.proxy_settings.time_step > params.proxy_settings.simulation_time) {
        params.proxy_settings.time_step = params.proxy_settings.simulation_time * 0.25f;
        params_valid = false;
    }

    return params_valid;
}

bool StructuralAnalyzer::CheckNumericalStability() const {
    // Check for NaN, Inf, or extreme values in node states
    for (const auto* node : nodes) {
        // Check displacement
        if (std::isnan(node->displacement) || std::isinf(node->displacement)) {
            std::cerr << "[StructuralAnalyzer] Numerical instability detected: "
                      << "Node " << node->id << " has invalid displacement ("
                      << node->displacement << ")\n";
            return false;
        }

        // Check velocity
        if (std::isnan(node->velocity) || std::isinf(node->velocity)) {
            std::cerr << "[StructuralAnalyzer] Numerical instability detected: "
                      << "Node " << node->id << " has invalid velocity ("
                      << node->velocity << ")\n";
            return false;
        }

        // Check mass
        if (std::isnan(node->mass_supported) || std::isinf(node->mass_supported)) {
            std::cerr << "[StructuralAnalyzer] Numerical instability detected: "
                      << "Node " << node->id << " has invalid mass ("
                      << node->mass_supported << ")\n";
            return false;
        }

        // Check for extreme values (likely indicates instability)
        // Note: We use generous thresholds to catch only truly extreme numerical errors
        const float EXTREME_DISPLACEMENT = 100000.0f;  // 100km displacement is clearly numerical error
        const float EXTREME_VELOCITY = 10000.0f;       // 10km/s velocity is clearly numerical error
        const float EXTREME_MASS = 1e10f;              // 10 billion kg is clearly numerical error

        if (std::abs(node->displacement) > EXTREME_DISPLACEMENT) {
            std::cerr << "[StructuralAnalyzer] Numerical instability detected: "
                      << "Node " << node->id << " has extreme displacement ("
                      << node->displacement << "m)\n";
            return false;
        }

        if (std::abs(node->velocity) > EXTREME_VELOCITY) {
            std::cerr << "[StructuralAnalyzer] Numerical instability detected: "
                      << "Node " << node->id << " has extreme velocity ("
                      << node->velocity << "m/s)\n";
            return false;
        }

        if (node->mass_supported > EXTREME_MASS) {
            std::cerr << "[StructuralAnalyzer] Numerical instability detected: "
                      << "Node " << node->id << " has extreme mass ("
                      << node->mass_supported << "kg)\n";
            return false;
        }
    }

    return true;
}

const char* StructuralAnalyzer::ModeToString(StructuralMode mode) const {
    switch (mode) {
        case StructuralMode::HeuristicSupport: return "HeuristicSupport";
        case StructuralMode::StackDepth: return "StackDepth";
        case StructuralMode::LoadRedistribution: return "LoadRedistribution";
        case StructuralMode::PhysicsProxy: return "PhysicsProxy";
        default: return "Unknown";
    }
}

StructuralAnalyzer::StructuralMode StructuralAnalyzer::StringToMode(const std::string& mode) const {
    std::string normalized;
    normalized.reserve(mode.size());
    for (char c : mode) {
        normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }

    if (normalized == "heuristicsupport" || normalized == "heuristic") {
        return StructuralMode::HeuristicSupport;
    }
    if (normalized == "stackdepth" || normalized == "stack") {
        return StructuralMode::StackDepth;
    }
    if (normalized == "loadredistribution" || normalized == "redistribution" || normalized == "load") {
        return StructuralMode::LoadRedistribution;
    }
    if (normalized == "physicsproxy" || normalized == "proxy") {
        return StructuralMode::PhysicsProxy;
    }

    std::cerr << "[StructuralAnalyzer] Unknown structural mode \"" << mode
              << "\" - defaulting to HeuristicSupport\n";
    return StructuralMode::HeuristicSupport;
}

float StructuralAnalyzer::CalculateStableTimestep() const {
    if (!world || nodes.empty()) {
        return params.timestep;
    }

    float safe_timestep = params.timestep;
    float voxel_size = world->GetVoxelSize();
    float voxel_volume = voxel_size * voxel_size * voxel_size;

    for (const auto* node : nodes) {
        if (!node || node->is_ground_anchor) {
            continue;
        }

        const Material& mat = g_materials.GetMaterial(node->material_id);
        float total_mass = mat.density * voxel_volume + node->mass_supported;
        total_mass = std::max(total_mass, 0.0001f);

        size_t neighbor_count = std::max<size_t>(1, node->neighbors.size());
        float stiffness = mat.spring_constant * static_cast<float>(neighbor_count);
        if (stiffness <= 0.0f) {
            continue;
        }

        float critical_timestep = 2.0f * std::sqrt(total_mass / stiffness);
        float adjusted = std::max(0.0001f, critical_timestep * 0.5f);  // safety factor
        safe_timestep = std::min(safe_timestep, adjusted);
    }

    return safe_timestep;
}
