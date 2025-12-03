#include "SpatialHash.h"
#include <algorithm>
#include <cmath>

SpatialHash::SpatialHash(float cell_size)
    : cell_size(cell_size)
{
}

GridCell SpatialHash::GetCell(const Vector3& position) const {
    // Use floor division to get grid cell coordinates
    return GridCell(
        static_cast<int>(std::floor(position.x / cell_size)),
        static_cast<int>(std::floor(position.y / cell_size)),
        static_cast<int>(std::floor(position.z / cell_size))
    );
}

void SpatialHash::Insert(const Vector3& position) {
    GridCell cell = GetCell(position);
    grid[cell].push_back(position);
}

void SpatialHash::Remove(const Vector3& position) {
    GridCell cell = GetCell(position);

    auto it = grid.find(cell);
    if (it != grid.end()) {
        auto& positions = it->second;

        // Remove the position from the cell's list
        positions.erase(
            std::remove_if(positions.begin(), positions.end(),
                [&position](const Vector3& p) {
                    return (p - position).LengthSquared() < 0.0001f;  // Epsilon comparison
                }),
            positions.end()
        );

        // Remove empty cells
        if (positions.empty()) {
            grid.erase(it);
        }
    }
}

void SpatialHash::Clear() {
    grid.clear();
}

std::vector<GridCell> SpatialHash::GetCellsInRadius(const Vector3& center, float radius) const {
    std::vector<GridCell> cells;

    // Calculate how many cells the radius spans
    int cell_radius = static_cast<int>(std::ceil(radius / cell_size));

    GridCell center_cell = GetCell(center);

    // Check all cells in a cube around the center
    for (int dx = -cell_radius; dx <= cell_radius; dx++) {
        for (int dy = -cell_radius; dy <= cell_radius; dy++) {
            for (int dz = -cell_radius; dz <= cell_radius; dz++) {
                GridCell cell(
                    center_cell.x + dx,
                    center_cell.y + dy,
                    center_cell.z + dz
                );
                cells.push_back(cell);
            }
        }
    }

    return cells;
}

std::vector<Vector3> SpatialHash::QueryRadius(const Vector3& center, float radius) const {
    std::vector<Vector3> results;

    // Get all cells that might contain positions within radius
    auto cells = GetCellsInRadius(center, radius);

    // Check each cell
    for (const auto& cell : cells) {
        auto it = grid.find(cell);
        if (it != grid.end()) {
            // Check each position in the cell
            for (const auto& pos : it->second) {
                float dist_sq = (pos - center).LengthSquared();
                if (dist_sq <= radius * radius) {
                    results.push_back(pos);
                }
            }
        }
    }

    return results;
}

std::vector<GridCell> SpatialHash::GetCellsInBox(const Vector3& min, const Vector3& max) const {
    std::vector<GridCell> cells;

    GridCell min_cell = GetCell(min);
    GridCell max_cell = GetCell(max);

    // Iterate through all cells in the box
    for (int x = min_cell.x; x <= max_cell.x; x++) {
        for (int y = min_cell.y; y <= max_cell.y; y++) {
            for (int z = min_cell.z; z <= max_cell.z; z++) {
                cells.push_back(GridCell(x, y, z));
            }
        }
    }

    return cells;
}

std::vector<Vector3> SpatialHash::QueryBox(const Vector3& min, const Vector3& max) const {
    std::vector<Vector3> results;

    // Get all cells that intersect the box
    auto cells = GetCellsInBox(min, max);

    // Check each cell
    for (const auto& cell : cells) {
        auto it = grid.find(cell);
        if (it != grid.end()) {
            // Check each position in the cell
            for (const auto& pos : it->second) {
                if (pos.x >= min.x && pos.x <= max.x &&
                    pos.y >= min.y && pos.y <= max.y &&
                    pos.z >= min.z && pos.z <= max.z) {
                    results.push_back(pos);
                }
            }
        }
    }

    return results;
}

size_t SpatialHash::GetTotalPositions() const {
    size_t total = 0;
    for (const auto& pair : grid) {
        total += pair.second.size();
    }
    return total;
}
