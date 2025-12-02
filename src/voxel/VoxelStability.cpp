#include "VoxelStability.h"
#include "Material.h"
#include <queue>
#include <algorithm>
#include <cmath>

// Check if a voxel is touching the ground
bool VoxelStability::IsGroundVoxel(const Vector3& position, float ground_level) {
    return position.y <= ground_level;
}

// Find all voxels that provide direct ground support
std::unordered_set<Vector3, Vector3::Hash> VoxelStability::FindGroundSupportVoxels(
    const VoxelWorld& world,
    const VoxelCluster& cluster,
    float ground_level)
{
    std::unordered_set<Vector3, Vector3::Hash> ground_voxels;

    for (const auto& pos : cluster.voxel_positions) {
        if (IsGroundVoxel(pos, ground_level)) {
            ground_voxels.insert(pos);
        }
    }

    return ground_voxels;
}

// Find all voxels that are supported (have path to ground)
std::unordered_set<Vector3, Vector3::Hash> VoxelStability::FindSupportedVoxels(
    const VoxelWorld& world,
    const VoxelCluster& cluster,
    float ground_level)
{
    // Start with ground voxels
    auto supported = FindGroundSupportVoxels(world, cluster, ground_level);
    if (supported.empty()) {
        return supported;  // No ground contact = nothing supported
    }

    // Create set of all cluster voxels for fast lookup
    std::unordered_set<Vector3, Vector3::Hash> cluster_voxels(
        cluster.voxel_positions.begin(),
        cluster.voxel_positions.end()
    );

    // BFS from ground voxels to find all supported voxels
    std::queue<Vector3> queue;
    for (const auto& pos : supported) {
        queue.push(pos);
    }

    while (!queue.empty()) {
        Vector3 current = queue.front();
        queue.pop();

        // Check all 6-connected neighbors
        auto neighbors = world.GetNeighbors(current, Connectivity::SIX);
        for (const auto& neighbor : neighbors) {
            // If neighbor is in cluster and not yet marked as supported
            if (cluster_voxels.count(neighbor) > 0 &&
                supported.count(neighbor) == 0) {
                supported.insert(neighbor);
                queue.push(neighbor);
            }
        }
    }

    return supported;
}

// Classify support type for each voxel
std::unordered_map<Vector3, SupportType, Vector3::Hash> VoxelStability::ClassifySupport(
    const VoxelWorld& world,
    const VoxelCluster& cluster,
    float ground_level)
{
    std::unordered_map<Vector3, SupportType, Vector3::Hash> classification;

    // Find grounded and supported voxels
    auto ground_voxels = FindGroundSupportVoxels(world, cluster, ground_level);
    auto supported_voxels = FindSupportedVoxels(world, cluster, ground_level);

    // Classify each voxel
    for (const auto& pos : cluster.voxel_positions) {
        if (ground_voxels.count(pos) > 0) {
            classification[pos] = SupportType::GROUNDED;
        } else if (supported_voxels.count(pos) > 0) {
            classification[pos] = SupportType::SUPPORTED;
        } else {
            classification[pos] = SupportType::UNSUPPORTED;
        }
    }

    return classification;
}

// Check if a specific voxel has support path to ground
bool VoxelStability::HasSupportPath(
    const VoxelWorld& world,
    const Vector3& position,
    float ground_level)
{
    // If already on ground, it's supported
    if (IsGroundVoxel(position, ground_level)) {
        return true;
    }

    // BFS to find path to ground
    std::unordered_set<Vector3, Vector3::Hash> visited;
    std::queue<Vector3> queue;
    queue.push(position);
    visited.insert(position);

    while (!queue.empty()) {
        Vector3 current = queue.front();
        queue.pop();

        // Check if we reached ground
        if (IsGroundVoxel(current, ground_level)) {
            return true;
        }

        // Explore neighbors
        auto neighbors = world.GetNeighbors(current, Connectivity::SIX);
        for (const auto& neighbor : neighbors) {
            if (visited.count(neighbor) == 0) {
                visited.insert(neighbor);
                queue.push(neighbor);
            }
        }
    }

    return false;  // No path to ground found
}

// Calculate center of support polygon
Vector3 VoxelStability::CalculateSupportCenter(
    const std::unordered_set<Vector3, Vector3::Hash>& support_voxels)
{
    if (support_voxels.empty()) {
        return Vector3::Zero();
    }

    Vector3 sum = Vector3::Zero();
    for (const auto& pos : support_voxels) {
        sum += pos;
    }

    return sum / static_cast<float>(support_voxels.size());
}

// Calculate support area (2D projection in XZ plane)
float VoxelStability::CalculateSupportArea(
    const std::unordered_set<Vector3, Vector3::Hash>& support_voxels)
{
    // Default voxel size (5cm)
    const float voxel_size = 0.05f;

    if (support_voxels.empty()) {
        return 0.0f;
    }

    if (support_voxels.size() == 1) {
        // Single voxel support = voxel size squared
        return voxel_size * voxel_size;
    }

    // Find bounding rectangle in XZ plane
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();

    for (const auto& pos : support_voxels) {
        min_x = std::min(min_x, pos.x);
        max_x = std::max(max_x, pos.x);
        min_z = std::min(min_z, pos.z);
        max_z = std::max(max_z, pos.z);
    }

    // Bounding rectangle area (rough approximation)
    float width = (max_x - min_x) + voxel_size;
    float depth = (max_z - min_z) + voxel_size;
    return width * depth;
}

// Check if COM is over support polygon (2D check in XZ plane)
bool VoxelStability::IsCenterOfMassSupported(
    const VoxelCluster& cluster,
    const std::unordered_set<Vector3, Vector3::Hash>& support_voxels)
{
    // Default voxel size (5cm)
    const float voxel_size = 0.05f;

    if (support_voxels.empty()) {
        return false;
    }

    // Single support voxel - check if COM is directly above
    if (support_voxels.size() == 1) {
        const Vector3& support = *support_voxels.begin();
        float dx = cluster.center_of_mass.x - support.x;
        float dz = cluster.center_of_mass.z - support.z;
        float dist_2d = std::sqrt(dx*dx + dz*dz);
        // Allow some tolerance (half voxel size)
        return dist_2d <= voxel_size * 0.5f;
    }

    // Multiple support voxels - check if COM is within bounding box
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();

    for (const auto& pos : support_voxels) {
        min_x = std::min(min_x, pos.x);
        max_x = std::max(max_x, pos.x);
        min_z = std::min(min_z, pos.z);
        max_z = std::max(max_z, pos.z);
    }

    // Check if COM is within support bounds (with small margin)
    const float margin = voxel_size * 0.5f;
    return cluster.center_of_mass.x >= (min_x - margin) &&
           cluster.center_of_mass.x <= (max_x + margin) &&
           cluster.center_of_mass.z >= (min_z - margin) &&
           cluster.center_of_mass.z <= (max_z + margin);
}

// Calculate stability score (0.0 = unstable, 1.0 = very stable)
float VoxelStability::CalculateStabilityScore(
    const VoxelCluster& cluster,
    const std::unordered_set<Vector3, Vector3::Hash>& support_voxels)
{
    // Default voxel size (5cm)
    const float voxel_size = 0.05f;

    if (support_voxels.empty()) {
        return 0.0f;  // No support = completely unstable
    }

    // Factor 1: COM distance from support center (0.0-1.0)
    Vector3 support_center = CalculateSupportCenter(support_voxels);
    float dx = cluster.center_of_mass.x - support_center.x;
    float dz = cluster.center_of_mass.z - support_center.z;
    float com_distance_2d = std::sqrt(dx*dx + dz*dz);

    // Normalize by cluster size (structures with larger bases are more stable)
    float cluster_size = (cluster.bounds.max - cluster.bounds.min).Length();
    float normalized_distance = com_distance_2d / std::max(cluster_size, 0.1f);
    float com_score = std::max(0.0f, 1.0f - normalized_distance * 2.0f);

    // Factor 2: Support area relative to structure (0.0-1.0)
    float support_area = CalculateSupportArea(support_voxels);
    float cluster_footprint = cluster.voxel_positions.size() * voxel_size * voxel_size;
    float area_ratio = support_area / std::max(cluster_footprint, 0.001f);
    float area_score = std::min(1.0f, area_ratio);

    // Factor 3: Number of support points (more = better, 0.0-1.0)
    float support_count_score = std::min(1.0f,
        static_cast<float>(support_voxels.size()) /
        std::max(4.0f, cluster.voxel_positions.size() * 0.1f));

    // Weighted combination
    float stability = com_score * 0.5f + area_score * 0.3f + support_count_score * 0.2f;
    return std::clamp(stability, 0.0f, 1.0f);
}

// Calculate tip risk (0.0 = safe, 1.0 = about to tip)
float VoxelStability::CalculateTipRisk(
    const VoxelCluster& cluster,
    const std::unordered_set<Vector3, Vector3::Hash>& support_voxels)
{
    // Default voxel size (5cm)
    const float voxel_size = 0.05f;

    if (support_voxels.empty()) {
        return 1.0f;  // No support = maximum risk
    }

    // Calculate support bounds in XZ plane
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();

    for (const auto& pos : support_voxels) {
        min_x = std::min(min_x, pos.x);
        max_x = std::max(max_x, pos.x);
        min_z = std::min(min_z, pos.z);
        max_z = std::max(max_z, pos.z);
    }

    Vector3 support_center = CalculateSupportCenter(support_voxels);

    // Calculate distances from COM to edges
    float dist_to_min_x = cluster.center_of_mass.x - min_x;
    float dist_to_max_x = max_x - cluster.center_of_mass.x;
    float dist_to_min_z = cluster.center_of_mass.z - min_z;
    float dist_to_max_z = max_z - cluster.center_of_mass.z;

    // Find minimum distance to any edge
    float min_edge_dist = std::min({dist_to_min_x, dist_to_max_x,
                                     dist_to_min_z, dist_to_max_z});

    // Calculate support radius (half the diagonal)
    float support_width = max_x - min_x + voxel_size;
    float support_depth = max_z - min_z + voxel_size;
    float support_radius = std::sqrt(support_width*support_width +
                                     support_depth*support_depth) * 0.5f;

    // If COM is outside support, maximum risk
    if (min_edge_dist < 0.0f) {
        return 1.0f;
    }

    // Risk increases as COM approaches edge
    // Risk = 1.0 when at edge, 0.0 when at center
    float risk = 1.0f - (min_edge_dist / std::max(support_radius, 0.01f));
    return std::clamp(risk, 0.0f, 1.0f);
}

// Main stability analysis function
StabilityInfo VoxelStability::AnalyzeStability(
    const VoxelWorld& world,
    const VoxelCluster& cluster,
    float ground_level)
{
    StabilityInfo info;

    // Basic info
    info.center_of_mass = cluster.center_of_mass;

    // Find support voxels
    auto ground_voxels = FindGroundSupportVoxels(world, cluster, ground_level);
    auto supported_voxels = FindSupportedVoxels(world, cluster, ground_level);

    // Grounding check
    info.is_grounded = !ground_voxels.empty();
    info.num_support_voxels = static_cast<int>(ground_voxels.size());
    info.num_unsupported_voxels = static_cast<int>(
        cluster.voxel_positions.size() - supported_voxels.size()
    );

    // If not grounded, structure is unstable
    if (!info.is_grounded) {
        info.is_stable = false;
        info.stability_score = 0.0f;
        info.tip_risk = 1.0f;
        return info;
    }

    // Calculate support metrics
    info.support_center = CalculateSupportCenter(ground_voxels);
    info.support_area = CalculateSupportArea(ground_voxels);

    // Check if COM is supported
    info.is_stable = IsCenterOfMassSupported(cluster, ground_voxels);

    // Calculate stability metrics
    info.stability_score = CalculateStabilityScore(cluster, ground_voxels);
    info.tip_risk = CalculateTipRisk(cluster, ground_voxels);

    return info;
}
