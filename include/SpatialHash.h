#pragma once

#include "Vector3.h"
#include <unordered_map>
#include <vector>

/**
 * SpatialHash - Accelerate spatial queries with grid-based partitioning
 *
 * Divides 3D space into a uniform grid of cells. Each cell contains
 * a list of positions within that cell. This accelerates queries like
 * "find all voxels within radius R" from O(n) to O(cells * voxels_per_cell).
 *
 * Design choices:
 * - Cell size: Typically 0.5m - 1.0m (10-20 voxels per cell)
 * - Sparse grid: Only stores non-empty cells
 * - Hash-based: O(1) average cell lookup
 *
 * Performance:
 * - Radius queries: O(cells_in_radius * avg_voxels_per_cell)
 * - For 50K voxels, 1m cells: ~50x speedup vs naive O(n) search
 */

struct GridCell {
    int x, y, z;

    GridCell() : x(0), y(0), z(0) {}
    GridCell(int x, int y, int z) : x(x), y(y), z(z) {}

    bool operator==(const GridCell& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    // Hash function for unordered_map
    struct Hash {
        size_t operator()(const GridCell& cell) const {
            // Combine coordinates with bit shifts to avoid collisions
            size_t h1 = std::hash<int>{}(cell.x);
            size_t h2 = std::hash<int>{}(cell.y);
            size_t h3 = std::hash<int>{}(cell.z);
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };
};

class SpatialHash {
public:
    explicit SpatialHash(float cell_size = 1.0f);

    // Basic operations
    void Insert(const Vector3& position);
    void Remove(const Vector3& position);
    void Clear();

    // Queries
    std::vector<Vector3> QueryRadius(const Vector3& center, float radius) const;
    std::vector<Vector3> QueryBox(const Vector3& min, const Vector3& max) const;

    // Utility
    GridCell GetCell(const Vector3& position) const;
    size_t GetCellCount() const { return grid.size(); }
    size_t GetTotalPositions() const;

private:
    float cell_size;
    std::unordered_map<GridCell, std::vector<Vector3>, GridCell::Hash> grid;

    // Helper to get all grid cells that intersect with a sphere
    std::vector<GridCell> GetCellsInRadius(const Vector3& center, float radius) const;

    // Helper to get all grid cells that intersect with a box
    std::vector<GridCell> GetCellsInBox(const Vector3& min, const Vector3& max) const;
};
