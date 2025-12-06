/**
 * ChunkedVoxelCollision.h
 *
 * Debris-Voxel Collision System
 * Manages chunked collision meshes for voxel world
 */

#pragma once

#include "ChunkCoord.h"
#include "VoxelCollisionChunk.h"
#include "Vector3.h"
#include "VoxelWorld.h"

#include <unordered_map>
#include <set>

#ifdef USE_BULLET
#include <btBulletDynamicsCommon.h>
#endif

/**
 * Manages collision chunks for entire voxel world
 *
 * Divides world into spatial chunks, each with its own static collision mesh.
 * Tracks dirty chunks and rebuilds them as voxels are modified.
 */
class ChunkedVoxelCollision {
public:
    ChunkedVoxelCollision(float chunk_size = 10.0f);
    ~ChunkedVoxelCollision();

#ifdef USE_BULLET
    /**
     * Initialize collision system
     * @param physics_world Bullet physics world
     * @param voxel_world VoxelWorld to build collision for
     */
    void Initialize(btDiscreteDynamicsWorld* physics_world, VoxelWorld* voxel_world);
#endif

    /**
     * Shutdown and cleanup all chunks
     */
    void Shutdown();

    // ===== Initial Build =====

    /**
     * Build collision chunks for entire world
     */
    void BuildChunksForWorld();

    /**
     * Build chunks in radius around a point
     * @param center Center point
     * @param radius Radius in world units
     */
    void BuildChunksInRadius(const Vector3& center, float radius);

    // ===== Updates =====

    /**
     * Mark chunk containing position as dirty
     * Also marks neighboring chunks dirty (voxels on edges)
     * @param world_position Position in world space
     */
    void MarkChunkDirty(const Vector3& world_position);

    /**
     * Mark all chunks in radius as dirty
     * @param center Center point
     * @param radius Radius in world units
     */
    void MarkChunksDirtyInRadius(const Vector3& center, float radius);

    /**
     * Update collision system (rebuilds dirty chunks with time budgeting)
     * @param deltaTime Time since last update
     */
    void Update(float deltaTime);

    /**
     * Rebuild all dirty chunks
     */
    void RebuildDirtyChunks();

    /**
     * Rebuild dirty chunks immediately (no time budgeting)
     */
    void RebuildDirtyChunksImmediate();

    // ===== Queries =====

    /**
     * Get chunk coordinate for world position
     */
    ChunkCoord GetChunkCoord(const Vector3& world_position) const;

    /**
     * Get world-space center of chunk
     */
    Vector3 GetChunkCenter(const ChunkCoord& coord) const;

    /**
     * Get chunk at coordinate (nullptr if doesn't exist)
     */
    VoxelCollisionChunk* GetChunk(const ChunkCoord& coord);

    // ===== Debug =====

    /**
     * Print statistics about collision chunks
     */
    void PrintStatistics() const;

    // ===== Getters =====

    float GetChunkSize() const { return chunk_size; }
    int GetChunkCount() const { return static_cast<int>(chunks.size()); }
    int GetDirtyChunkCount() const { return static_cast<int>(dirty_chunks.size()); }

private:
    /**
     * Rebuild a single chunk
     */
    void RebuildSingleChunk(const ChunkCoord& coord);

    // Chunk storage
    std::unordered_map<ChunkCoord, VoxelCollisionChunk*, ChunkCoordHash> chunks;
    std::set<ChunkCoord> dirty_chunks;

#ifdef USE_BULLET
    btDiscreteDynamicsWorld* physics_world = nullptr;
#endif
    VoxelWorld* voxel_world = nullptr;

    // Configuration
    float chunk_size;  // Size of each chunk in world units (e.g., 10m)

    // Time budgeting for rebuilds
    float rebuild_timer = 0.0f;
    const float REBUILD_INTERVAL = 0.1f;  // Rebuild every 100ms

    // Prevent copying
    ChunkedVoxelCollision(const ChunkedVoxelCollision&) = delete;
    ChunkedVoxelCollision& operator=(const ChunkedVoxelCollision&) = delete;
};
