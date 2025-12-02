#pragma once

#include "Vector3.h"
#include "Voxel.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>

/**
 * VoxelWorld - Sparse voxel storage and queries
 *
 * Uses a hash map for sparse storage (only stores solid voxels).
 * This is efficient for structures with lots of empty space.
 *
 * Design choices:
 * - Voxel size: 5cm (0.05m) - balances detail vs. performance
 * - Sparse storage: Only solid voxels stored (air is implicit)
 * - Hash-based: O(1) average lookup time
 *
 * Performance:
 * - Can handle 100K+ voxels efficiently
 * - Lookups: O(1) average
 * - Neighbor queries: O(1) per neighbor
 */
class VoxelWorld {
public:
    VoxelWorld(float voxel_size = 0.05f);

    // Basic voxel operations
    bool HasVoxel(const Vector3& position) const;
    Voxel GetVoxel(const Vector3& position) const;
    void SetVoxel(const Vector3& position, const Voxel& voxel);
    void RemoveVoxel(const Vector3& position);

    // Bulk operations
    void Clear();
    size_t GetVoxelCount() const { return voxels.size(); }
    std::vector<Vector3> GetAllVoxelPositions() const;

    // Neighbor queries
    std::vector<Vector3> GetNeighbors(const Vector3& position,
                                     Connectivity connectivity = Connectivity::SIX) const;
    int CountNeighbors(const Vector3& position,
                      Connectivity connectivity = Connectivity::SIX) const;

    // Spatial queries
    std::vector<Vector3> GetVoxelsInRadius(const Vector3& center, float radius) const;
    std::vector<Vector3> GetVoxelsInBox(const Vector3& min, const Vector3& max) const;
    std::vector<Vector3> Raycast(const Vector3& origin,
                                 const Vector3& direction,
                                 float max_distance) const;

    // Surface detection (cached for performance)
    const std::unordered_set<Vector3, Vector3::Hash>& GetSurfaceVoxels();
    bool IsSurfaceVoxel(const Vector3& position) const;
    void InvalidateSurfaceCache();

    // Utility
    float GetVoxelSize() const { return voxel_size; }
    Vector3 SnapToGrid(const Vector3& position) const;

private:
    float voxel_size;  // Size of each voxel (default 0.05m = 5cm)
    std::unordered_map<Vector3, Voxel, Vector3::Hash> voxels;

    // Surface cache for performance
    mutable std::unordered_set<Vector3, Vector3::Hash> surface_cache;
    mutable bool surface_cache_dirty;

    // Helper methods
    void UpdateSurfaceCache() const;
    void InvalidateSurfaceAt(const Vector3& position);

    // Helper for neighbor offsets
    static const std::vector<Vector3>& GetNeighborOffsets(Connectivity connectivity);
};

/**
 * BoundingBox - Axis-aligned bounding box
 *
 * Used for spatial queries and cluster bounds.
 */
struct BoundingBox {
    Vector3 min;
    Vector3 max;

    BoundingBox() : min(Vector3::Zero()), max(Vector3::Zero()) {}
    BoundingBox(const Vector3& min, const Vector3& max) : min(min), max(max) {}

    Vector3 GetCenter() const {
        return (min + max) * 0.5f;
    }

    Vector3 GetSize() const {
        return max - min;
    }

    float GetVolume() const {
        Vector3 size = GetSize();
        return size.x * size.y * size.z;
    }

    bool Contains(const Vector3& point) const {
        return point.x >= min.x && point.x <= max.x &&
               point.y >= min.y && point.y <= max.y &&
               point.z >= min.z && point.z <= max.z;
    }

    bool Intersects(const BoundingBox& other) const {
        return !(max.x < other.min.x || min.x > other.max.x ||
                 max.y < other.min.y || min.y > other.max.y ||
                 max.z < other.min.z || min.z > other.max.z);
    }
};
