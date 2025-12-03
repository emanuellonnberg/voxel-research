#pragma once

#include "Vector3.h"
#include "VoxelWorld.h"
#include <vector>
#include <unordered_set>
#include <queue>

/**
 * VoxelCluster - Represents a connected group of voxels
 *
 * Used for:
 * - Detecting when structures break into separate pieces
 * - Calculating physics properties of disconnected parts
 * - Identifying which clusters are grounded vs falling
 *
 * A cluster is defined as a set of voxels where each voxel
 * is connected to at least one other voxel in the set through
 * face-adjacent neighbors (6-connectivity).
 */
struct VoxelCluster {
    std::vector<Vector3> voxel_positions;  // All voxel positions in this cluster
    Vector3 center_of_mass;                // Physics center (mass-weighted)
    float total_mass;                      // Sum of all voxel masses
    BoundingBox bounds;                    // Axis-aligned bounding box
    size_t cluster_id;                     // Unique identifier

    VoxelCluster() : center_of_mass(0, 0, 0), total_mass(0.0f), cluster_id(0) {}

    size_t GetVoxelCount() const { return voxel_positions.size(); }
    bool IsEmpty() const { return voxel_positions.empty(); }
};

/**
 * VoxelClustering - Utilities for finding and analyzing connected components
 *
 * Implements flood-fill based clustering to identify disconnected structures.
 * This is essential for:
 * - Structural integrity: Detecting when a structure loses connection
 * - Physics: Separating objects that should move independently
 * - Destruction: Identifying pieces after voxel removal
 */
class VoxelClustering {
public:
    /**
     * Find all connected clusters in the voxel world
     *
     * Uses BFS flood-fill to find connected components.
     * Connectivity: 6-connected (face neighbors only)
     *
     * @param world The voxel world to analyze
     * @return Vector of all disconnected clusters
     */
    static std::vector<VoxelCluster> FindAllClusters(const VoxelWorld& world);

    /**
     * Find cluster starting from a seed voxel
     *
     * @param world The voxel world
     * @param seed Starting position (must contain a voxel)
     * @param visited Set of already-visited positions (updated by this function)
     * @return Cluster containing all voxels connected to seed
     */
    static VoxelCluster FloodFillCluster(const VoxelWorld& world,
                                         const Vector3& seed,
                                         std::unordered_set<Vector3, Vector3::Hash>& visited);

    /**
     * Find all clusters and identify which are grounded
     *
     * A cluster is "grounded" if any of its voxels touch the ground plane (y <= ground_level).
     *
     * @param world The voxel world
     * @param ground_level Y-coordinate of ground plane
     * @param out_grounded Output: which clusters are grounded (indices into returned vector)
     * @return Vector of all clusters
     */
    static std::vector<VoxelCluster> FindClustersWithGrounding(
        const VoxelWorld& world,
        float ground_level,
        std::vector<bool>& out_grounded);

    /**
     * Calculate cluster properties from voxel positions
     *
     * Calculates:
     * - Center of mass (mass-weighted average position)
     * - Total mass (sum of voxel masses)
     * - Bounding box (min/max extents)
     *
     * @param world The voxel world (to get voxel materials)
     * @param cluster The cluster to calculate properties for (modified in-place)
     */
    static void CalculateClusterProperties(const VoxelWorld& world, VoxelCluster& cluster);

private:
    // Helper to check if position is at or below ground level
    static bool IsGroundAnchor(const Vector3& position, float ground_level, float voxel_size);
};
