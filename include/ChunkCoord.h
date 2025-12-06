/**
 * ChunkCoord.h
 *
 * Debris-Voxel Collision System
 * Chunk coordinate structure for spatial partitioning
 */

#pragma once

#include <functional>

/**
 * Chunk coordinate in 3D space
 * Used to identify which chunk a position belongs to
 */
struct ChunkCoord {
    int x, y, z;

    ChunkCoord() : x(0), y(0), z(0) {}
    ChunkCoord(int x, int y, int z) : x(x), y(y), z(z) {}

    bool operator==(const ChunkCoord& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    bool operator!=(const ChunkCoord& other) const {
        return !(*this == other);
    }

    bool operator<(const ChunkCoord& other) const {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
    }
};

/**
 * Hash function for ChunkCoord (for unordered_map)
 */
struct ChunkCoordHash {
    size_t operator()(const ChunkCoord& c) const {
        // FNV-1a hash
        size_t hash = 2166136261u;
        hash ^= static_cast<size_t>(c.x);
        hash *= 16777619u;
        hash ^= static_cast<size_t>(c.y);
        hash *= 16777619u;
        hash ^= static_cast<size_t>(c.z);
        hash *= 16777619u;
        return hash;
    }
};
