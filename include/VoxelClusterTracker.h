#pragma once

#include "VoxelCluster.h"
#include <unordered_map>

/**
 * VoxelClusterTracker
 *
 * Lightweight helper that caches connected-component information for a
 * VoxelWorld. Consumers can query clusters without recomputing flood fills
 * every frame, and the tracker automatically invalidates its cache when the
 * VoxelWorld edit version changes.
 */
class VoxelClusterTracker {
public:
    explicit VoxelClusterTracker(const VoxelWorld* world = nullptr);

    void SetWorld(const VoxelWorld* world);

    /**
     * Force the tracker to rebuild on the next query, even if the world
     * version has not changed (useful after batch modifications).
     */
    void MarkDirty();

    /**
     * Returns the cached clusters. Rebuilds automatically if the world has
     * changed since the last query.
     */
    const std::vector<VoxelCluster>& GetClusters();

    /**
     * Returns the cluster that contains the given voxel position, or nullptr
     * if none exists or the cache is empty.
     */
    const VoxelCluster* FindClusterContaining(const Vector3& position) const;

    /**
     * Returns true if the tracker needs to rebuild because the world changed
     * or MarkDirty() was called.
     */
    bool NeedsUpdate() const;

    size_t GetTrackerVersion() const { return tracker_version; }
    size_t GetCachedWorldVersion() const { return cached_world_version; }

private:
    void RebuildIfNeeded();
    void Rebuild();
    void RebuildLookupTable();

    const VoxelWorld* world;
    size_t cached_world_version;
    size_t tracker_version;
    bool dirty;

    std::vector<VoxelCluster> clusters;
    std::unordered_map<Vector3, size_t, Vector3::Hash> voxel_to_cluster;
};
