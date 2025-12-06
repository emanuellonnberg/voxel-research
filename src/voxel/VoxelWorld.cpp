#include "VoxelWorld.h"
#include <algorithm>
#include <cmath>

VoxelWorld::VoxelWorld(float voxel_size)
    : voxel_size(voxel_size)
    , surface_cache_dirty(true)
    , edit_version(0) {}

bool VoxelWorld::HasVoxel(const Vector3& position) const {
    Vector3 snapped = SnapToGrid(position);
    auto it = voxels.find(snapped);
    return it != voxels.end() && it->second.IsSolid();
}

Voxel VoxelWorld::GetVoxel(const Vector3& position) const {
    Vector3 snapped = SnapToGrid(position);
    auto it = voxels.find(snapped);
    if (it != voxels.end()) {
        return it->second;
    }
    return Voxel();  // Return air voxel if not found
}

void VoxelWorld::SetVoxel(const Vector3& position, const Voxel& voxel) {
    Vector3 snapped = SnapToGrid(position);

    bool was_solid = HasVoxel(snapped);
    bool changed = false;

    if (voxel.IsSolid()) {
        voxels[snapped] = voxel;
        changed = true;

        // Update spatial hash if enabled (only insert if newly solid)
        if (spatial_hash && !was_solid) {
            spatial_hash->Insert(snapped);
            changed = true;
        }
    } else {
        // If setting to air, remove from map (sparse storage)
        auto erased = voxels.erase(snapped);
        changed = changed || erased > 0;

        // Remove from spatial hash if enabled
        if (spatial_hash && was_solid) {
            spatial_hash->Remove(snapped);
        }
    }

    if (changed) {
        ++edit_version;
        // Invalidate surface cache for this voxel and neighbors
        InvalidateSurfaceAt(snapped);
    }
}

void VoxelWorld::RemoveVoxel(const Vector3& position) {
    Vector3 snapped = SnapToGrid(position);

    bool was_solid = HasVoxel(snapped);
    size_t erased = voxels.erase(snapped);

    // Remove from spatial hash if enabled
    if (spatial_hash && was_solid) {
        spatial_hash->Remove(snapped);
    }

    if (erased > 0) {
        ++edit_version;
        // Invalidate surface cache for this voxel and neighbors
        InvalidateSurfaceAt(snapped);
    }
}

void VoxelWorld::Clear() {
    voxels.clear();
    surface_cache.clear();
    surface_cache_dirty = true;
    ++edit_version;

    // Clear spatial hash if enabled
    if (spatial_hash) {
        spatial_hash->Clear();
    }
}

std::vector<Vector3> VoxelWorld::GetAllVoxelPositions() const {
    std::vector<Vector3> positions;
    positions.reserve(voxels.size());

    for (const auto& pair : voxels) {
        if (pair.second.IsSolid()) {
            positions.push_back(pair.first);
        }
    }

    return positions;
}

std::vector<Vector3> VoxelWorld::GetNeighbors(const Vector3& position,
                                               Connectivity connectivity) const {
    std::vector<Vector3> neighbors;
    const auto& offsets = GetNeighborOffsets(connectivity);

    Vector3 snapped = SnapToGrid(position);

    for (const auto& offset : offsets) {
        Vector3 neighbor_pos = snapped + offset * voxel_size;
        if (HasVoxel(neighbor_pos)) {
            neighbors.push_back(neighbor_pos);
        }
    }

    return neighbors;
}

int VoxelWorld::CountNeighbors(const Vector3& position,
                               Connectivity connectivity) const {
    const auto& offsets = GetNeighborOffsets(connectivity);
    Vector3 snapped = SnapToGrid(position);

    int count = 0;
    for (const auto& offset : offsets) {
        Vector3 neighbor_pos = snapped + offset * voxel_size;
        if (HasVoxel(neighbor_pos)) {
            count++;
        }
    }

    return count;
}

std::vector<Vector3> VoxelWorld::GetVoxelsInRadius(const Vector3& center, float radius) const {
    // Use spatial hash if enabled (much faster for large structures)
    if (spatial_hash) {
        return spatial_hash->QueryRadius(center, radius);
    }

    // Fallback: Naive implementation - check all voxels
    std::vector<Vector3> result;
    float radius_sq = radius * radius;

    for (const auto& pair : voxels) {
        if (!pair.second.IsSolid()) continue;

        float dist_sq = center.DistanceSquared(pair.first);
        if (dist_sq <= radius_sq) {
            result.push_back(pair.first);
        }
    }

    return result;
}

std::vector<Vector3> VoxelWorld::GetVoxelsInBox(const Vector3& min, const Vector3& max) const {
    // Use spatial hash if enabled (much faster for large structures)
    if (spatial_hash) {
        return spatial_hash->QueryBox(min, max);
    }

    // Fallback: Iterate through grid cells in the box
    std::vector<Vector3> result;
    Vector3 min_snapped = SnapToGrid(min);
    Vector3 max_snapped = SnapToGrid(max);

    for (float x = min_snapped.x; x <= max_snapped.x; x += voxel_size) {
        for (float y = min_snapped.y; y <= max_snapped.y; y += voxel_size) {
            for (float z = min_snapped.z; z <= max_snapped.z; z += voxel_size) {
                Vector3 pos(x, y, z);
                if (HasVoxel(pos)) {
                    result.push_back(pos);
                }
            }
        }
    }

    return result;
}

std::vector<Vector3> VoxelWorld::Raycast(const Vector3& origin,
                                         const Vector3& direction,
                                         float max_distance) const {
    std::vector<Vector3> hits;

    // DDA (Digital Differential Analyzer) raycast algorithm
    // Optimized for voxel grids
    Vector3 ray_dir = direction.Normalized();

    // Step size (move by one voxel at a time)
    float step = voxel_size * 0.5f;  // Half voxel for accuracy
    int max_steps = static_cast<int>(max_distance / step);

    Vector3 current = origin;

    for (int i = 0; i < max_steps; i++) {
        if (HasVoxel(current)) {
            Vector3 snapped = SnapToGrid(current);

            // Avoid duplicate hits (multiple steps in same voxel)
            if (hits.empty() || hits.back() != snapped) {
                hits.push_back(snapped);
            }
        }

        current += ray_dir * step;
    }

    return hits;
}

Vector3 VoxelWorld::SnapToGrid(const Vector3& position) const {
    // Round to nearest voxel grid position
    return Vector3(
        std::round(position.x / voxel_size) * voxel_size,
        std::round(position.y / voxel_size) * voxel_size,
        std::round(position.z / voxel_size) * voxel_size
    );
}

const std::vector<Vector3>& VoxelWorld::GetNeighborOffsets(Connectivity connectivity) {
    // Static arrays to avoid reallocation
    static const std::vector<Vector3> offsets_6 = {
        Vector3( 1,  0,  0),  // Right
        Vector3(-1,  0,  0),  // Left
        Vector3( 0,  1,  0),  // Up
        Vector3( 0, -1,  0),  // Down
        Vector3( 0,  0,  1),  // Forward
        Vector3( 0,  0, -1)   // Back
    };

    static const std::vector<Vector3> offsets_26 = []() {
        std::vector<Vector3> offsets;
        offsets.reserve(26);

        // Generate all 27 offsets in 3x3x3 cube, excluding center (0,0,0)
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                for (int dz = -1; dz <= 1; dz++) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;  // Skip center
                    offsets.push_back(Vector3(
                        static_cast<float>(dx),
                        static_cast<float>(dy),
                        static_cast<float>(dz)
                    ));
                }
            }
        }

        return offsets;
    }();

    return (connectivity == Connectivity::SIX) ? offsets_6 : offsets_26;
}

// Surface detection methods

bool VoxelWorld::IsSurfaceVoxel(const Vector3& position) const {
    Vector3 snapped = SnapToGrid(position);

    // Not a voxel = not surface
    if (!HasVoxel(snapped)) {
        return false;
    }

    // Check if any neighbor is air (6-connected)
    const auto& offsets = GetNeighborOffsets(Connectivity::SIX);
    for (const auto& offset : offsets) {
        Vector3 neighbor_pos = snapped + (offset * voxel_size);
        if (!HasVoxel(neighbor_pos)) {
            return true;  // Has at least one air neighbor
        }
    }

    return false;  // Completely surrounded by solid voxels
}

const std::unordered_set<Vector3, Vector3::Hash>& VoxelWorld::GetSurfaceVoxels() {
    if (surface_cache_dirty) {
        UpdateSurfaceCache();
        surface_cache_dirty = false;
    }
    return surface_cache;
}

void VoxelWorld::InvalidateSurfaceCache() {
    surface_cache_dirty = true;
}

void VoxelWorld::UpdateSurfaceCache() const {
    surface_cache.clear();

    // Check all voxels to see if they're surface
    for (const auto& pair : voxels) {
        if (IsSurfaceVoxel(pair.first)) {
            surface_cache.insert(pair.first);
        }
    }
}

void VoxelWorld::InvalidateSurfaceAt(const Vector3& position) {
    // When a voxel changes, it and its neighbors might change surface status
    // For simplicity, mark entire cache as dirty
    // (More optimized version would only update affected voxels)
    surface_cache_dirty = true;

    // Optional: Partial cache update (more complex but faster for small changes)
    // Remove this voxel from cache
    // surface_cache.erase(position);
    //
    // Check neighbors - they might become surface or interior
    // for (auto& neighbor_pos : GetNeighbors(position, Connectivity::SIX)) {
    //     if (IsSurfaceVoxel(neighbor_pos)) {
    //         surface_cache.insert(neighbor_pos);
    //     } else {
    //         surface_cache.erase(neighbor_pos);
    //     }
    // }
}

// Spatial hashing methods

void VoxelWorld::EnableSpatialHashing(float cell_size) {
    if (!spatial_hash) {
        spatial_hash = std::make_unique<SpatialHash>(cell_size);

        // Populate spatial hash with existing voxels
        for (const auto& pair : voxels) {
            if (pair.second.IsSolid()) {
                spatial_hash->Insert(pair.first);
            }
        }
    }
}

void VoxelWorld::DisableSpatialHashing() {
    spatial_hash.reset();
}
