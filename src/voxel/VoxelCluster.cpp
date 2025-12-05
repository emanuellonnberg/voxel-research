#include "VoxelCluster.h"
#include "Material.h"
#include <algorithm>
#include <cmath>

VoxelCluster VoxelClustering::FloodFillCluster(
    const VoxelWorld& world,
    const Vector3& seed,
    std::unordered_set<Vector3, Vector3::Hash>& visited)
{
    VoxelCluster cluster;

    // BFS queue for flood fill
    std::queue<Vector3> queue;
    queue.push(seed);
    visited.insert(seed);

    // Flood fill using BFS
    while (!queue.empty()) {
        Vector3 current = queue.front();
        queue.pop();

        cluster.voxel_positions.push_back(current);

        // Get 6-connected neighbors (face neighbors only)
        auto neighbors = world.GetNeighbors(current, Connectivity::SIX);
        for (const auto& neighbor : neighbors) {
            // If not visited and has a voxel, add to queue
            if (visited.find(neighbor) == visited.end() && world.HasVoxel(neighbor)) {
                queue.push(neighbor);
                visited.insert(neighbor);
            }
        }
    }

    return cluster;
}

std::vector<VoxelCluster> VoxelClustering::FindAllClusters(const VoxelWorld& world) {
    std::vector<VoxelCluster> clusters;
    std::unordered_set<Vector3, Vector3::Hash> visited;

    // Get all voxel positions
    auto all_positions = world.GetAllVoxelPositions();

    size_t cluster_id = 0;

    // Find clusters by flood-filling from unvisited voxels
    for (const auto& pos : all_positions) {
        if (visited.find(pos) == visited.end()) {
            VoxelCluster cluster = FloodFillCluster(world, pos, visited);
            cluster.cluster_id = cluster_id++;

            // Calculate cluster properties
            CalculateClusterProperties(world, cluster);

            clusters.push_back(cluster);
        }
    }

    return clusters;
}

std::vector<VoxelCluster> VoxelClustering::FindClustersWithGrounding(
    const VoxelWorld& world,
    float ground_level,
    std::vector<bool>& out_grounded)
{
    auto clusters = FindAllClusters(world);
    out_grounded.resize(clusters.size());

    float voxel_size = world.GetVoxelSize();

    for (size_t i = 0; i < clusters.size(); i++) {
        bool is_grounded = false;

        // Check if any voxel in the cluster is touching the ground
        for (const auto& pos : clusters[i].voxel_positions) {
            if (IsGroundAnchor(pos, ground_level, voxel_size)) {
                is_grounded = true;
                break;
            }
        }

        out_grounded[i] = is_grounded;
    }

    return clusters;
}

void VoxelClustering::CalculateClusterProperties(const VoxelWorld& world, VoxelCluster& cluster) {
    if (cluster.voxel_positions.empty()) {
        cluster.center_of_mass = Vector3(0, 0, 0);
        cluster.total_mass = 0.0f;
        cluster.bounds = BoundingBox();
        cluster.dominant_material = 0;
        return;
    }

    // Calculate center of mass (mass-weighted)
    Vector3 com(0, 0, 0);
    float total_mass = 0.0f;
    float voxel_size = world.GetVoxelSize();
    float voxel_volume = voxel_size * voxel_size * voxel_size;

    // Initialize bounding box with first voxel
    Vector3 min_bounds = cluster.voxel_positions[0];
    Vector3 max_bounds = cluster.voxel_positions[0];

    cluster.dominant_material = 0;

    for (const auto& pos : cluster.voxel_positions) {
        Voxel voxel = world.GetVoxel(pos);
        const Material& mat = g_materials.GetMaterial(voxel.material_id);

        float voxel_mass = mat.density * voxel_volume;
        com += pos * voxel_mass;
        total_mass += voxel_mass;

        if (cluster.dominant_material == 0 && voxel.is_active) {
            cluster.dominant_material = voxel.material_id;
        }

        // Update bounds
        min_bounds.x = std::min(min_bounds.x, pos.x);
        min_bounds.y = std::min(min_bounds.y, pos.y);
        min_bounds.z = std::min(min_bounds.z, pos.z);
        max_bounds.x = std::max(max_bounds.x, pos.x);
        max_bounds.y = std::max(max_bounds.y, pos.y);
        max_bounds.z = std::max(max_bounds.z, pos.z);
    }

    cluster.center_of_mass = (total_mass > 0) ? (com / total_mass) : Vector3(0, 0, 0);
    cluster.total_mass = total_mass;
    cluster.bounds = BoundingBox(min_bounds, max_bounds);
}

bool VoxelClustering::IsGroundAnchor(const Vector3& position, float ground_level, float voxel_size) {
    // A voxel is grounded if its bottom edge is at or below ground level
    return position.y <= ground_level + voxel_size * 0.5f;
}
