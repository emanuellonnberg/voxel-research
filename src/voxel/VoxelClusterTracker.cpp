#include "VoxelClusterTracker.h"

#include "VoxelWorld.h"

VoxelClusterTracker::VoxelClusterTracker(const VoxelWorld* world)
    : world(world)
    , cached_world_version(0)
    , tracker_version(0)
    , dirty(true) {}

void VoxelClusterTracker::SetWorld(const VoxelWorld* new_world) {
    if (world == new_world) {
        return;
    }

    world = new_world;
    cached_world_version = 0;
    tracker_version = 0;
    clusters.clear();
    voxel_to_cluster.clear();
    dirty = true;
}

void VoxelClusterTracker::MarkDirty() {
    dirty = true;
}

bool VoxelClusterTracker::NeedsUpdate() const {
    if (!world) {
        return false;
    }
    return dirty || cached_world_version != world->GetEditVersion();
}

const std::vector<VoxelCluster>& VoxelClusterTracker::GetClusters() {
    RebuildIfNeeded();
    return clusters;
}

const VoxelCluster* VoxelClusterTracker::FindClusterContaining(const Vector3& position) const {
    if (voxel_to_cluster.empty()) {
        return nullptr;
    }

    Vector3 snapped = world ? world->SnapToGrid(position) : position;
    auto it = voxel_to_cluster.find(snapped);
    if (it == voxel_to_cluster.end()) {
        return nullptr;
    }

    size_t index = it->second;
    if (index >= clusters.size()) {
        return nullptr;
    }
    return &clusters[index];
}

void VoxelClusterTracker::RebuildIfNeeded() {
    if (!world) {
        clusters.clear();
        voxel_to_cluster.clear();
        cached_world_version = 0;
        dirty = false;
        return;
    }

    if (!NeedsUpdate()) {
        return;
    }

    Rebuild();
}

void VoxelClusterTracker::Rebuild() {
    clusters = VoxelClustering::FindAllClusters(*world);
    RebuildLookupTable();
    cached_world_version = world ? world->GetEditVersion() : 0;
    dirty = false;
    tracker_version++;
}

void VoxelClusterTracker::RebuildLookupTable() {
    voxel_to_cluster.clear();
    for (size_t i = 0; i < clusters.size(); ++i) {
        for (const auto& pos : clusters[i].voxel_positions) {
            voxel_to_cluster[pos] = i;
        }
    }
}
