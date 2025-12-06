/**
 * ChunkedVoxelCollision.cpp
 *
 * Debris-Voxel Collision System
 * Chunk manager implementation
 */

#include "ChunkedVoxelCollision.h"
#include <iostream>
#include <chrono>
#include <cmath>

using Clock = std::chrono::high_resolution_clock;

static float GetElapsedMS(std::chrono::time_point<Clock> start) {
    auto end = Clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    return duration.count() / 1000.0f;
}

ChunkedVoxelCollision::ChunkedVoxelCollision(float chunk_size)
    : chunk_size(chunk_size)
#ifdef USE_BULLET
    , physics_world(nullptr)
#endif
    , voxel_world(nullptr)
{
}

ChunkedVoxelCollision::~ChunkedVoxelCollision() {
    Shutdown();
}

#ifdef USE_BULLET
void ChunkedVoxelCollision::Initialize(btDiscreteDynamicsWorld* physics_world,
                                      VoxelWorld* voxel_world) {
    this->physics_world = physics_world;
    this->voxel_world = voxel_world;

    std::cout << "[ChunkedVoxelCollision] Initialized with chunk size " << chunk_size << "m\n";
}
#endif

void ChunkedVoxelCollision::Shutdown() {
#ifdef USE_BULLET
    // Remove all chunks from physics world
    for (auto& [coord, chunk] : chunks) {
        chunk->RemoveFromPhysicsWorld(physics_world);
        chunk->Cleanup();
        delete chunk;
    }
#else
    // Without Bullet, just clean up
    for (auto& [coord, chunk] : chunks) {
        chunk->Cleanup();
        delete chunk;
    }
#endif

    chunks.clear();
    dirty_chunks.clear();

    std::cout << "[ChunkedVoxelCollision] Shutdown complete\n";
}

ChunkCoord ChunkedVoxelCollision::GetChunkCoord(const Vector3& world_position) const {
    return ChunkCoord{
        static_cast<int>(std::floor(world_position.x / chunk_size)),
        static_cast<int>(std::floor(world_position.y / chunk_size)),
        static_cast<int>(std::floor(world_position.z / chunk_size))
    };
}

Vector3 ChunkedVoxelCollision::GetChunkCenter(const ChunkCoord& coord) const {
    return Vector3(
        (coord.x + 0.5f) * chunk_size,
        (coord.y + 0.5f) * chunk_size,
        (coord.z + 0.5f) * chunk_size
    );
}

void ChunkedVoxelCollision::BuildChunksForWorld() {
    if (!voxel_world) {
        std::cerr << "[ChunkedVoxelCollision] ERROR: VoxelWorld not set!\n";
        return;
    }

    std::cout << "[ChunkedVoxelCollision] Building collision chunks for world...\n";
    auto start = Clock::now();

    // Get all voxels
    auto all_voxels = voxel_world->GetAllVoxelPositions();

    // Determine which chunks contain voxels
    std::set<ChunkCoord> affected_chunks;

    for (const auto& pos : all_voxels) {
        ChunkCoord chunk = GetChunkCoord(pos);
        affected_chunks.insert(chunk);
    }

    std::cout << "[ChunkedVoxelCollision] Found " << affected_chunks.size() << " chunks with voxels\n";

    // Build each chunk
    int built_count = 0;

    for (const auto& coord : affected_chunks) {
        // Create chunk
        auto* chunk = new VoxelCollisionChunk();
        chunk->coord = coord;

        // Build collision mesh
        chunk->BuildFromVoxels(*voxel_world, chunk_size);

#ifdef USE_BULLET
        // Add to physics world if not empty
        if (chunk->triangle_count > 0) {
            chunk->AddToPhysicsWorld(physics_world);
            built_count++;
        }
#else
        if (chunk->triangle_count > 0) {
            built_count++;
        }
#endif

        // Store chunk
        chunks[coord] = chunk;
    }

    float elapsed = GetElapsedMS(start);

    std::cout << "[ChunkedVoxelCollision] Built " << built_count << " collision chunks in "
              << elapsed << "ms\n";

    PrintStatistics();
}

void ChunkedVoxelCollision::BuildChunksInRadius(const Vector3& center, float radius) {
    // Calculate chunk radius
    int chunk_radius = static_cast<int>(std::ceil(radius / chunk_size));

    ChunkCoord center_chunk = GetChunkCoord(center);

    // Build chunks in radius
    for (int x = -chunk_radius; x <= chunk_radius; x++) {
        for (int y = -chunk_radius; y <= chunk_radius; y++) {
            for (int z = -chunk_radius; z <= chunk_radius; z++) {
                ChunkCoord coord{
                    center_chunk.x + x,
                    center_chunk.y + y,
                    center_chunk.z + z
                };

                // Check if chunk already exists
                if (chunks.count(coord) > 0) {
                    continue;
                }

                // Create and build chunk
                auto* chunk = new VoxelCollisionChunk();
                chunk->coord = coord;
                chunk->BuildFromVoxels(*voxel_world, chunk_size);

#ifdef USE_BULLET
                if (chunk->triangle_count > 0) {
                    chunk->AddToPhysicsWorld(physics_world);
                }
#endif

                chunks[coord] = chunk;
            }
        }
    }
}

void ChunkedVoxelCollision::MarkChunkDirty(const Vector3& world_position) {
    ChunkCoord coord = GetChunkCoord(world_position);

    // Mark this chunk dirty
    dirty_chunks.insert(coord);

    // Also mark neighboring chunks dirty (voxel might be on edge)
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            for (int dz = -1; dz <= 1; dz++) {
                if (dx == 0 && dy == 0 && dz == 0) continue;

                ChunkCoord neighbor{
                    coord.x + dx,
                    coord.y + dy,
                    coord.z + dz
                };

                if (chunks.count(neighbor) > 0) {
                    dirty_chunks.insert(neighbor);
                }
            }
        }
    }
}

void ChunkedVoxelCollision::MarkChunksDirtyInRadius(const Vector3& center, float radius) {
    int chunk_radius = static_cast<int>(std::ceil(radius / chunk_size));
    ChunkCoord center_chunk = GetChunkCoord(center);

    for (int x = -chunk_radius; x <= chunk_radius; x++) {
        for (int y = -chunk_radius; y <= chunk_radius; y++) {
            for (int z = -chunk_radius; z <= chunk_radius; z++) {
                ChunkCoord coord{
                    center_chunk.x + x,
                    center_chunk.y + y,
                    center_chunk.z + z
                };

                if (chunks.count(coord) > 0) {
                    dirty_chunks.insert(coord);
                }
            }
        }
    }
}

void ChunkedVoxelCollision::Update(float deltaTime) {
    rebuild_timer += deltaTime;

    // Rebuild dirty chunks periodically
    if (rebuild_timer >= REBUILD_INTERVAL && !dirty_chunks.empty()) {
        RebuildDirtyChunks();
        rebuild_timer = 0.0f;
    }
}

void ChunkedVoxelCollision::RebuildDirtyChunks() {
    if (dirty_chunks.empty()) return;

    auto start = Clock::now();
    int rebuilt = 0;

    for (const auto& coord : dirty_chunks) {
        RebuildSingleChunk(coord);
        rebuilt++;
    }

    float elapsed = GetElapsedMS(start);

    std::cout << "[ChunkedVoxelCollision] Rebuilt " << rebuilt << " dirty chunks in "
              << elapsed << "ms\n";

    dirty_chunks.clear();
}

void ChunkedVoxelCollision::RebuildDirtyChunksImmediate() {
    rebuild_timer = REBUILD_INTERVAL;  // Force immediate rebuild
    RebuildDirtyChunks();
}

void ChunkedVoxelCollision::RebuildSingleChunk(const ChunkCoord& coord) {
    auto it = chunks.find(coord);

    if (it == chunks.end()) {
        // Chunk doesn't exist yet, create it
        auto* chunk = new VoxelCollisionChunk();
        chunk->coord = coord;
        chunk->BuildFromVoxels(*voxel_world, chunk_size);

#ifdef USE_BULLET
        if (chunk->triangle_count > 0) {
            chunk->AddToPhysicsWorld(physics_world);
        }
#endif

        chunks[coord] = chunk;

    } else {
        // Chunk exists, rebuild it
        auto* chunk = it->second;

#ifdef USE_BULLET
        // Remove from physics world
        chunk->RemoveFromPhysicsWorld(physics_world);
#endif

        // Rebuild mesh
        chunk->BuildFromVoxels(*voxel_world, chunk_size);

#ifdef USE_BULLET
        // Add back to physics world (if not empty)
        if (chunk->triangle_count > 0) {
            chunk->AddToPhysicsWorld(physics_world);
        }
#endif
    }
}

VoxelCollisionChunk* ChunkedVoxelCollision::GetChunk(const ChunkCoord& coord) {
    auto it = chunks.find(coord);
    return (it != chunks.end()) ? it->second : nullptr;
}

void ChunkedVoxelCollision::PrintStatistics() const {
    int total_triangles = 0;
    int active_chunks = 0;
    int empty_chunks = 0;

    for (const auto& [coord, chunk] : chunks) {
        if (chunk->triangle_count > 0) {
            total_triangles += chunk->triangle_count;
            active_chunks++;
        } else {
            empty_chunks++;
        }
    }

    std::cout << "\n=== Voxel Collision Statistics ===\n";
    std::cout << "Total chunks: " << chunks.size() << "\n";
    std::cout << "Active chunks: " << active_chunks << "\n";
    std::cout << "Empty chunks: " << empty_chunks << "\n";
    std::cout << "Total triangles: " << total_triangles << "\n";

    if (active_chunks > 0) {
        std::cout << "Avg triangles/chunk: "
                  << (total_triangles / active_chunks) << "\n";
    }

    std::cout << "Dirty chunks: " << dirty_chunks.size() << "\n";
    std::cout << "==================================\n\n";
}
