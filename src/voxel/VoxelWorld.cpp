#include "VoxelWorld.h"
#include <algorithm>
#include <cmath>
#include <mutex>          // For std::unique_lock
#include <shared_mutex>   // For std::shared_lock

VoxelWorld::VoxelWorld(float voxel_size)
    : voxel_size(voxel_size), surface_cache_dirty(true)
{
}

bool VoxelWorld::HasVoxel(const Vector3& position) const {
    std::shared_lock<std::shared_mutex> lock(voxel_mutex);
    Vector3 snapped = SnapToGrid(position);
    auto it = voxels.find(snapped);
    return it != voxels.end() && it->second.IsSolid();
}

Voxel VoxelWorld::GetVoxel(const Vector3& position) const {
    std::shared_lock<std::shared_mutex> lock(voxel_mutex);
    Vector3 snapped = SnapToGrid(position);
    auto it = voxels.find(snapped);
    if (it != voxels.end()) {
        return it->second;
    }
    return Voxel();  // Return air voxel if not found
}

void VoxelWorld::SetVoxel(const Vector3& position, const Voxel& voxel) {
    std::unique_lock<std::shared_mutex> lock(voxel_mutex);
    Vector3 snapped = SnapToGrid(position);

    // Check if voxel was solid (avoid nested lock by checking directly)
    auto it = voxels.find(snapped);
    bool was_solid = (it != voxels.end() && it->second.IsSolid());

    if (voxel.IsSolid()) {
        voxels[snapped] = voxel;

        // Update spatial hash if enabled (only insert if newly solid)
        if (spatial_hash && !was_solid) {
            spatial_hash->Insert(snapped);
        }
    } else {
        // If setting to air, remove from map (sparse storage)
        voxels.erase(snapped);

        // Remove from spatial hash if enabled
        if (spatial_hash && was_solid) {
            spatial_hash->Remove(snapped);
        }
    }

    // Invalidate surface cache for this voxel and neighbors
    InvalidateSurfaceAt(snapped);
}

void VoxelWorld::RemoveVoxel(const Vector3& position) {
    std::unique_lock<std::shared_mutex> lock(voxel_mutex);
    Vector3 snapped = SnapToGrid(position);

    // Check if voxel was solid (avoid nested lock by checking directly)
    auto it = voxels.find(snapped);
    bool was_solid = (it != voxels.end() && it->second.IsSolid());
    voxels.erase(snapped);

    // Remove from spatial hash if enabled
    if (spatial_hash && was_solid) {
        spatial_hash->Remove(snapped);
    }

    // Invalidate surface cache for this voxel and neighbors
    InvalidateSurfaceAt(snapped);
}

void VoxelWorld::Clear() {
    std::unique_lock<std::shared_mutex> voxel_lock(voxel_mutex);
    std::unique_lock<std::shared_mutex> surface_lock(surface_mutex);

    voxels.clear();
    surface_cache.clear();
    surface_cache_dirty = true;

    // Clear spatial hash if enabled
    if (spatial_hash) {
        spatial_hash->Clear();
    }
}

size_t VoxelWorld::GetVoxelCount() const {
    std::shared_lock<std::shared_mutex> lock(voxel_mutex);
    return voxels.size();
}

std::vector<Vector3> VoxelWorld::GetAllVoxelPositions() const {
    std::shared_lock<std::shared_mutex> lock(voxel_mutex);

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
    std::shared_lock<std::shared_mutex> lock(voxel_mutex);

    std::vector<Vector3> neighbors;
    const auto& offsets = GetNeighborOffsets(connectivity);

    Vector3 snapped = SnapToGrid(position);

    for (const auto& offset : offsets) {
        Vector3 neighbor_pos = snapped + offset * voxel_size;
        // Check directly instead of calling HasVoxel to avoid nested lock
        auto it = voxels.find(neighbor_pos);
        if (it != voxels.end() && it->second.IsSolid()) {
            neighbors.push_back(neighbor_pos);
        }
    }

    return neighbors;
}

int VoxelWorld::CountNeighbors(const Vector3& position,
                               Connectivity connectivity) const {
    std::shared_lock<std::shared_mutex> lock(voxel_mutex);

    const auto& offsets = GetNeighborOffsets(connectivity);
    Vector3 snapped = SnapToGrid(position);

    int count = 0;
    for (const auto& offset : offsets) {
        Vector3 neighbor_pos = snapped + offset * voxel_size;
        // Check directly instead of calling HasVoxel to avoid nested lock
        auto it = voxels.find(neighbor_pos);
        if (it != voxels.end() && it->second.IsSolid()) {
            count++;
        }
    }

    return count;
}

std::vector<Vector3> VoxelWorld::GetVoxelsInRadius(const Vector3& center, float radius) const {
    std::shared_lock<std::shared_mutex> lock(voxel_mutex);

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
    std::shared_lock<std::shared_mutex> lock(voxel_mutex);

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
                // Check directly instead of calling HasVoxel to avoid nested lock
                auto it = voxels.find(pos);
                if (it != voxels.end() && it->second.IsSolid()) {
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
    std::shared_lock<std::shared_mutex> lock(voxel_mutex);

    std::vector<Vector3> hits;

    // DDA (Digital Differential Analyzer) raycast algorithm
    // Optimized for voxel grids
    Vector3 ray_dir = direction.Normalized();

    // Step size (move by one voxel at a time)
    float step = voxel_size * 0.5f;  // Half voxel for accuracy
    int max_steps = static_cast<int>(max_distance / step);

    Vector3 current = origin;

    for (int i = 0; i < max_steps; i++) {
        // Check directly instead of calling HasVoxel to avoid nested lock
        Vector3 snapped = SnapToGrid(current);
        auto it = voxels.find(snapped);
        if (it != voxels.end() && it->second.IsSolid()) {
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
    std::shared_lock<std::shared_mutex> lock(voxel_mutex);

    Vector3 snapped = SnapToGrid(position);

    // Not a voxel = not surface (check directly to avoid nested lock)
    auto it = voxels.find(snapped);
    if (it == voxels.end() || !it->second.IsSolid()) {
        return false;
    }

    // Check if any neighbor is air (6-connected)
    const auto& offsets = GetNeighborOffsets(Connectivity::SIX);
    for (const auto& offset : offsets) {
        Vector3 neighbor_pos = snapped + (offset * voxel_size);
        // Check directly instead of calling HasVoxel to avoid nested lock
        auto neighbor_it = voxels.find(neighbor_pos);
        if (neighbor_it == voxels.end() || !neighbor_it->second.IsSolid()) {
            return true;  // Has at least one air neighbor
        }
    }

    return false;  // Completely surrounded by solid voxels
}

const std::unordered_set<Vector3, Vector3::Hash>& VoxelWorld::GetSurfaceVoxels() {
    // Double-checked locking pattern for lazy cache update
    {
        std::shared_lock<std::shared_mutex> lock(surface_mutex);
        if (!surface_cache_dirty) {
            return surface_cache;
        }
    }

    // Cache is dirty - need to update
    std::unique_lock<std::shared_mutex> lock(surface_mutex);
    // Check again in case another thread updated while we waited for unique lock
    if (surface_cache_dirty) {
        UpdateSurfaceCache();
        surface_cache_dirty = false;
    }
    return surface_cache;
}

void VoxelWorld::InvalidateSurfaceCache() {
    std::unique_lock<std::shared_mutex> lock(surface_mutex);
    surface_cache_dirty = true;
}

void VoxelWorld::UpdateSurfaceCache() const {
    // Called from GetSurfaceVoxels() which already holds surface_mutex
    // Need voxel_mutex to read voxels
    std::shared_lock<std::shared_mutex> lock(voxel_mutex);

    surface_cache.clear();

    // Check all voxels to see if they're surface (inline logic to avoid nested locks)
    const auto& offsets = GetNeighborOffsets(Connectivity::SIX);
    for (const auto& pair : voxels) {
        const Vector3& pos = pair.first;

        // Check if this voxel has any air neighbors
        bool is_surface = false;
        for (const auto& offset : offsets) {
            Vector3 neighbor_pos = pos + (offset * voxel_size);
            auto neighbor_it = voxels.find(neighbor_pos);
            if (neighbor_it == voxels.end() || !neighbor_it->second.IsSolid()) {
                is_surface = true;
                break;
            }
        }

        if (is_surface) {
            surface_cache.insert(pos);
        }
    }
}

void VoxelWorld::InvalidateSurfaceAt(const Vector3& position) {
    // Called from SetVoxel/RemoveVoxel which already hold voxel_mutex
    // When a voxel changes, it and its neighbors might change surface status
    // For simplicity, mark entire cache as dirty
    // (More optimized version would only update affected voxels)
    std::unique_lock<std::shared_mutex> lock(surface_mutex);
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
    std::unique_lock<std::shared_mutex> lock(voxel_mutex);

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
    std::unique_lock<std::shared_mutex> lock(voxel_mutex);
    spatial_hash.reset();
}
