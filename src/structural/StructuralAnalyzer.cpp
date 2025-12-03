#include "StructuralAnalyzer.h"
#include "VoxelUtils.h"
#include <chrono>
#include <iostream>
#include <queue>
#include <algorithm>
#include <thread>
#include <future>
#include <mutex>
#include <fstream>
#include <sstream>

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
        }
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
            UpdateDisplacement(node, params.timestep);
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
            std::cout << "[StructuralAnalyzer] Converged in " << iteration_count
                      << " iterations (max_change: " << max_displacement_change << "m)\n";
            return true;
        }

        // Also check velocity-based convergence as backup
        if (CheckConvergence()) {
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
    // Day 14: Flood-fill from ground anchors to find connected nodes
    std::unordered_set<SpringNode*> grounded;
    std::queue<SpringNode*> queue;

    // Start from all ground anchors
    for (auto* node : nodes) {
        if (node->is_ground_anchor) {
            queue.push(node);
            grounded.insert(node);
        }
    }

    // Flood-fill to find all nodes connected to ground
    while (!queue.empty()) {
        SpringNode* current = queue.front();
        queue.pop();

        for (auto* neighbor : current->neighbors) {
            // Only traverse through non-failed neighbors
            if (!neighbor->has_failed && grounded.find(neighbor) == grounded.end()) {
                grounded.insert(neighbor);
                queue.push(neighbor);
            }
        }
    }

    // Any node not grounded is floating (disconnected)
    std::vector<SpringNode*> floating;
    for (auto* node : nodes) {
        if (!node->is_ground_anchor && grounded.find(node) == grounded.end()) {
            node->is_disconnected = true;
            floating.push_back(node);
        }
    }

    return floating;
}

std::vector<SpringNode*> StructuralAnalyzer::DetectFailures() {
    std::vector<SpringNode*> failed_nodes;

    // Criterion 1: Excessive displacement (Day 11)
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
              << "(" << (failed_nodes.size() - floating_nodes.size()) << " displacement, "
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

    // Day 18: Validate parameters before analysis
    if (!ValidateParameters()) {
        std::cerr << "[StructuralAnalyzer] Warning: Parameters were corrected during validation\n";
    }

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
        return result;
    }

    // Step 2: Calculate mass supported (Day 12: Profile this step)
    auto start_mass = Clock::now();
    CalculateMassSupported();
    result.mass_calc_time_ms = std::chrono::duration<float, std::milli>(Clock::now() - start_mass).count();
    result.cache_hits = cache_hits;
    result.cache_misses = cache_misses;

    // Day 18: Check numerical stability after mass calculation
    if (!CheckNumericalStability()) {
        result.debug_info = "Numerical instability detected after mass calculation";
        result.calculation_time_ms = std::chrono::duration<float, std::milli>(Clock::now() - start_total).count();
        result.structure_failed = true;  // Treat as failure
        return result;
    }

    // Step 3: Solve for displacements (Day 12: Profile this step)
    auto start_solver = Clock::now();
    result.converged = SolveDisplacements();
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
