/**
 * VoxelFragmentation.cpp
 *
 * Week 5 Day 25: Material-Specific Fracture Patterns
 */

#include "VoxelFragmentation.h"
#include "Material.h"
#include <algorithm>
#include <cmath>
#include <chrono>

VoxelFragmentation::VoxelFragmentation()
    : rng(std::chrono::steady_clock::now().time_since_epoch().count()),
      dist_0_1(0.0f, 1.0f)
{
}

VoxelFragmentation::~VoxelFragmentation() {
}

// ===== Main Fracture Function =====

std::vector<VoxelCluster> VoxelFragmentation::FractureCluster(
    const VoxelCluster& cluster,
    uint8_t material_id)
{
    // Empty cluster check
    if (cluster.voxel_positions.empty()) {
        return {};
    }

    // Route to material-specific fracture pattern
    if (material_id == MaterialDatabase::WOOD) {
        return SplitIntoSplinters(cluster);
    }
    else if (material_id == MaterialDatabase::CONCRETE) {
        return SplitIntoChunks(cluster, 5, 10);
    }
    else if (material_id == MaterialDatabase::BRICK) {
        return SplitIntoMasonryUnits(cluster);
    }
    else {
        // Default: simple split (3-5 chunks)
        return SplitIntoChunks(cluster, 3, 5);
    }
}

// ===== Wood: Splinters =====

std::vector<VoxelCluster> VoxelFragmentation::SplitIntoSplinters(
    const VoxelCluster& cluster)
{
    std::vector<VoxelCluster> splinters;

    // Get cluster dimensions
    Vector3 size = GetClusterSize(cluster);
    int longest_axis = GetLongestAxis(size);

    // Split perpendicular to longest axis (wood grain)
    int num_splinters = RandomRange(3, 6);

    // Get bounds along split axis
    float min_coord, max_coord;
    if (longest_axis == 0) {
        min_coord = cluster.bounds.min.x;
        max_coord = cluster.bounds.max.x;
    } else if (longest_axis == 1) {
        min_coord = cluster.bounds.min.y;
        max_coord = cluster.bounds.max.y;
    } else {
        min_coord = cluster.bounds.min.z;
        max_coord = cluster.bounds.max.z;
    }
    float step = (max_coord - min_coord) / num_splinters;

    // Distribute voxels into splinters
    for (int i = 0; i < num_splinters; i++) {
        VoxelCluster splinter;
        splinter.cluster_id = cluster.cluster_id;

        float slice_min = min_coord + i * step;
        float slice_max = min_coord + (i + 1) * step;

        // For last slice, include everything up to and including max_coord
        bool is_last = (i == num_splinters - 1);

        // Collect voxels in this slice
        for (const auto& pos : cluster.voxel_positions) {
            float coord;
            if (longest_axis == 0) {
                coord = pos.x;
            } else if (longest_axis == 1) {
                coord = pos.y;
            } else {
                coord = pos.z;
            }

            bool in_slice = (coord >= slice_min && coord < slice_max) ||
                           (is_last && coord >= slice_min && coord <= slice_max + 0.001f);

            if (in_slice) {
                splinter.voxel_positions.push_back(pos);
            }
        }

        // Only keep non-empty splinters
        if (!splinter.voxel_positions.empty()) {
            RebuildClusterProperties(splinter);
            splinters.push_back(splinter);
        }
    }

    // If fragmentation failed, return original cluster
    if (splinters.empty()) {
        return {cluster};
    }

    return splinters;
}

// ===== Concrete/Default: Irregular Chunks =====

std::vector<VoxelCluster> VoxelFragmentation::SplitIntoChunks(
    const VoxelCluster& cluster,
    int min_chunks,
    int max_chunks)
{
    std::vector<VoxelCluster> chunks = {cluster};

    int target_count = RandomRange(min_chunks, max_chunks);

    // Recursively split largest chunk until we reach target count
    while (chunks.size() < (size_t)target_count) {
        // Find largest chunk
        auto largest_it = std::max_element(chunks.begin(), chunks.end(),
            [](const VoxelCluster& a, const VoxelCluster& b) {
                return a.voxel_positions.size() < b.voxel_positions.size();
            });

        // Stop if largest chunk is too small to split
        if (largest_it->voxel_positions.size() < 5) {
            break;
        }

        // Choose random axis to split along
        Vector3 size = GetClusterSize(*largest_it);
        int axis = RandomRange(0, 2);

        // Split position (slightly randomized, not exactly in center)
        float min_coord, max_coord;
        if (axis == 0) {
            min_coord = largest_it->bounds.min.x;
            max_coord = largest_it->bounds.max.x;
        } else if (axis == 1) {
            min_coord = largest_it->bounds.min.y;
            max_coord = largest_it->bounds.max.y;
        } else {
            min_coord = largest_it->bounds.min.z;
            max_coord = largest_it->bounds.max.z;
        }
        float split_ratio = RandomRange(0.3f, 0.7f);
        float split_pos = min_coord + (max_coord - min_coord) * split_ratio;

        // Perform split
        auto split_result = SplitClusterInHalf(*largest_it, axis, split_pos);

        // Replace original with two halves
        chunks.erase(largest_it);

        if (!split_result.first.voxel_positions.empty()) {
            chunks.push_back(split_result.first);
        }
        if (!split_result.second.voxel_positions.empty()) {
            chunks.push_back(split_result.second);
        }
    }

    return chunks;
}

// ===== Brick: Masonry Units =====

std::vector<VoxelCluster> VoxelFragmentation::SplitIntoMasonryUnits(
    const VoxelCluster& cluster)
{
    std::vector<VoxelCluster> units;

    // Bricks break into regular rectangular pieces
    // Split along Y axis (height) first, then X/Z

    Vector3 size = GetClusterSize(cluster);

    // Determine grid dimensions (2-3 splits per axis)
    int splits_x = (size.x > 0.1f) ? RandomRange(2, 3) : 1;
    int splits_y = (size.y > 0.1f) ? RandomRange(2, 3) : 1;
    int splits_z = (size.z > 0.1f) ? RandomRange(2, 3) : 1;

    float step_x = (cluster.bounds.max.x - cluster.bounds.min.x) / splits_x;
    float step_y = (cluster.bounds.max.y - cluster.bounds.min.y) / splits_y;
    float step_z = (cluster.bounds.max.z - cluster.bounds.min.z) / splits_z;

    // Create grid of units
    for (int ix = 0; ix < splits_x; ix++) {
        for (int iy = 0; iy < splits_y; iy++) {
            for (int iz = 0; iz < splits_z; iz++) {
                VoxelCluster unit;
                unit.cluster_id = cluster.cluster_id;

                // Define bounds for this unit
                float min_x = cluster.bounds.min.x + ix * step_x;
                float max_x = cluster.bounds.min.x + (ix + 1) * step_x;
                float min_y = cluster.bounds.min.y + iy * step_y;
                float max_y = cluster.bounds.min.y + (iy + 1) * step_y;
                float min_z = cluster.bounds.min.z + iz * step_z;
                float max_z = cluster.bounds.min.z + (iz + 1) * step_z;

                // Collect voxels in this unit
                // For last units in each dimension, include boundary voxels
                bool is_last_x = (ix == splits_x - 1);
                bool is_last_y = (iy == splits_y - 1);
                bool is_last_z = (iz == splits_z - 1);

                for (const auto& pos : cluster.voxel_positions) {
                    bool in_x = (pos.x >= min_x && pos.x < max_x) ||
                               (is_last_x && pos.x >= min_x && pos.x <= max_x + 0.001f);
                    bool in_y = (pos.y >= min_y && pos.y < max_y) ||
                               (is_last_y && pos.y >= min_y && pos.y <= max_y + 0.001f);
                    bool in_z = (pos.z >= min_z && pos.z < max_z) ||
                               (is_last_z && pos.z >= min_z && pos.z <= max_z + 0.001f);

                    if (in_x && in_y && in_z) {
                        unit.voxel_positions.push_back(pos);
                    }
                }

                // Only keep non-empty units
                if (!unit.voxel_positions.empty()) {
                    RebuildClusterProperties(unit);
                    units.push_back(unit);
                }
            }
        }
    }

    // If fragmentation failed, return original cluster
    if (units.empty()) {
        return {cluster};
    }

    return units;
}

// ===== Glass: Shards (simulated, not true Voronoi yet) =====

std::vector<VoxelCluster> VoxelFragmentation::ShatterIntoShards(
    const VoxelCluster& cluster,
    int num_shards)
{
    // For now, use irregular chunks (similar to concrete but more pieces)
    // True Voronoi shattering would be more complex
    return SplitIntoChunks(cluster, num_shards, num_shards + 5);
}

// ===== Utility Functions =====

Vector3 VoxelFragmentation::GetClusterSize(const VoxelCluster& cluster) {
    return cluster.bounds.max - cluster.bounds.min;
}

int VoxelFragmentation::GetLongestAxis(const Vector3& size) {
    if (size.x >= size.y && size.x >= size.z) return 0;
    if (size.y >= size.z) return 1;
    return 2;
}

int VoxelFragmentation::RandomRange(int min, int max) {
    if (min >= max) return min;
    return min + (rng() % (max - min + 1));
}

float VoxelFragmentation::RandomRange(float min, float max) {
    if (min >= max) return min;
    return min + dist_0_1(rng) * (max - min);
}

Vector3 VoxelFragmentation::RandomDirection() {
    // Generate uniform random direction on unit sphere
    float theta = RandomRange(0.0f, 2.0f * 3.14159f);
    float phi = std::acos(RandomRange(-1.0f, 1.0f));

    return Vector3(
        std::sin(phi) * std::cos(theta),
        std::sin(phi) * std::sin(theta),
        std::cos(phi)
    );
}

// ===== Helper Functions =====

std::pair<VoxelCluster, VoxelCluster> VoxelFragmentation::SplitClusterInHalf(
    const VoxelCluster& cluster,
    int axis,
    float split_pos)
{
    VoxelCluster half1, half2;
    half1.cluster_id = cluster.cluster_id;
    half2.cluster_id = cluster.cluster_id;

    // Distribute voxels based on split plane
    for (const auto& pos : cluster.voxel_positions) {
        float coord;
        if (axis == 0) {
            coord = pos.x;
        } else if (axis == 1) {
            coord = pos.y;
        } else {
            coord = pos.z;
        }

        if (coord < split_pos) {
            half1.voxel_positions.push_back(pos);
        } else {
            half2.voxel_positions.push_back(pos);
        }
    }

    // Rebuild properties for both halves
    if (!half1.voxel_positions.empty()) {
        RebuildClusterProperties(half1);
    }
    if (!half2.voxel_positions.empty()) {
        RebuildClusterProperties(half2);
    }

    return {half1, half2};
}

void VoxelFragmentation::RebuildClusterProperties(VoxelCluster& cluster) {
    if (cluster.voxel_positions.empty()) {
        cluster.center_of_mass = Vector3(0, 0, 0);
        cluster.total_mass = 0.0f;
        cluster.bounds = BoundingBox(Vector3(0, 0, 0), Vector3(0, 0, 0));
        return;
    }

    // Calculate center of mass
    Vector3 sum(0, 0, 0);
    for (const auto& pos : cluster.voxel_positions) {
        sum = sum + pos;
    }
    cluster.center_of_mass = sum / static_cast<float>(cluster.voxel_positions.size());

    // Calculate bounds
    Vector3 min_bound = cluster.voxel_positions[0];
    Vector3 max_bound = cluster.voxel_positions[0];

    for (const auto& pos : cluster.voxel_positions) {
        min_bound.x = std::min(min_bound.x, pos.x);
        min_bound.y = std::min(min_bound.y, pos.y);
        min_bound.z = std::min(min_bound.z, pos.z);

        max_bound.x = std::max(max_bound.x, pos.x);
        max_bound.y = std::max(max_bound.y, pos.y);
        max_bound.z = std::max(max_bound.z, pos.z);
    }

    cluster.bounds = BoundingBox(min_bound, max_bound);

    // Total mass (assuming constant density per voxel for now)
    cluster.total_mass = cluster.voxel_positions.size() * 1.0f;  // Placeholder
}
