/**
 * VoxelFragmentation.h
 *
 * Week 5 Day 25: Material-Specific Fracture Patterns
 *
 * Breaks VoxelClusters into realistic fragments based on material properties:
 * - Wood: Long splinters along grain
 * - Concrete: Irregular chunks
 * - Brick: Regular masonry units
 * - Glass: Many small shards
 * - Default: Simple recursive split
 */

#pragma once

#include "VoxelCluster.h"
#include "Material.h"
#include "Vector3.h"
#include <vector>
#include <random>

class VoxelFragmentation {
public:
    VoxelFragmentation();
    ~VoxelFragmentation();

    // Main fracture function - routes to material-specific pattern
    std::vector<VoxelCluster> FractureCluster(const VoxelCluster& cluster,
                                              uint8_t material_id);

    // Material-specific fracture patterns
    std::vector<VoxelCluster> SplitIntoSplinters(const VoxelCluster& cluster);
    std::vector<VoxelCluster> SplitIntoChunks(const VoxelCluster& cluster,
                                             int min_chunks,
                                             int max_chunks);
    std::vector<VoxelCluster> SplitIntoMasonryUnits(const VoxelCluster& cluster);
    std::vector<VoxelCluster> ShatterIntoShards(const VoxelCluster& cluster,
                                                int num_shards);

    // Utility functions
    static Vector3 GetClusterSize(const VoxelCluster& cluster);
    static int GetLongestAxis(const Vector3& size);

    // Random number generation
    int RandomRange(int min, int max);
    float RandomRange(float min, float max);
    Vector3 RandomDirection();

private:
    // Helper: Split cluster along a plane
    std::pair<VoxelCluster, VoxelCluster> SplitClusterInHalf(
        const VoxelCluster& cluster,
        int axis,
        float split_pos);

    // Helper: Rebuild cluster properties after split
    void RebuildClusterProperties(VoxelCluster& cluster);

    // Random number generator
    std::mt19937 rng;
    std::uniform_real_distribution<float> dist_0_1;
};
