#pragma once

#include "VoxelWorld.h"
#include "Material.h"
#include <vector>
#include <unordered_set>

/**
 * VoxelUtils - Utility functions for voxel operations
 *
 * Provides:
 * - Surface voxel detection
 * - Structure generation (walls, floors, buildings)
 * - Voxel manipulation (flood fill, painting)
 * - Analysis tools
 */
class VoxelUtils {
public:
    // Surface detection
    static bool IsSurfaceVoxel(const VoxelWorld& world, const Vector3& position,
                               Connectivity connectivity = Connectivity::SIX);
    static std::vector<Vector3> FindSurfaceVoxels(const VoxelWorld& world,
                                                  Connectivity connectivity = Connectivity::SIX);
    static int CountExposedFaces(const VoxelWorld& world, const Vector3& position);

    // Structure generation
    static void CreateBox(VoxelWorld& world, const Vector3& min, const Vector3& max,
                         uint8_t material_id, bool hollow = false);
    static void CreateSphere(VoxelWorld& world, const Vector3& center, float radius,
                            uint8_t material_id, bool hollow = false);
    static void CreateCylinder(VoxelWorld& world, const Vector3& base, float radius,
                              float height, uint8_t material_id, bool hollow = false);
    static void CreateWall(VoxelWorld& world, const Vector3& start, const Vector3& end,
                          float thickness, float height, uint8_t material_id);
    static void CreateFloor(VoxelWorld& world, const Vector3& corner, float width,
                           float depth, float thickness, uint8_t material_id);
    static void CreatePillar(VoxelWorld& world, const Vector3& base, float width,
                            float depth, float height, uint8_t material_id);

    // Voxel manipulation
    static void FloodFill(VoxelWorld& world, const Vector3& start, uint8_t new_material_id);
    static void ReplaceAll(VoxelWorld& world, uint8_t old_material_id, uint8_t new_material_id);
    static void RemoveDisconnected(VoxelWorld& world, const Vector3& anchor);

    // Analysis
    static BoundingBox GetBounds(const VoxelWorld& world);
    static Vector3 GetCenterOfMass(const VoxelWorld& world);
    static float GetTotalMass(const VoxelWorld& world);
    static int CountVoxelsByMaterial(const VoxelWorld& world, uint8_t material_id);

private:
    // Helper for flood fill
    static void FloodFillRecursive(VoxelWorld& world, const Vector3& position,
                                   uint8_t old_material_id, uint8_t new_material_id,
                                   std::unordered_set<Vector3, Vector3::Hash>& visited);
};

/**
 * StructureBuilder - Builder pattern for complex structures
 *
 * Example:
 *   StructureBuilder(world)
 *       .SetMaterial(MaterialDatabase::BRICK)
 *       .AddWall(start, end, height)
 *       .AddFloor(corner, width, depth)
 *       .Build();
 */
class StructureBuilder {
public:
    explicit StructureBuilder(VoxelWorld& world);

    // Configuration
    StructureBuilder& SetMaterial(uint8_t material_id);
    StructureBuilder& SetHollow(bool hollow);
    StructureBuilder& SetPosition(const Vector3& position);

    // Add components
    StructureBuilder& AddBox(const Vector3& size);
    StructureBuilder& AddWall(const Vector3& direction, float length, float height);
    StructureBuilder& AddFloor(float width, float depth, float thickness = 1.0f);
    StructureBuilder& AddPillar(float width, float depth, float height);
    StructureBuilder& AddRoom(float width, float depth, float height);

    // Build
    void Build();
    void Clear();

private:
    VoxelWorld& world;
    Vector3 current_position;
    uint8_t current_material;
    bool is_hollow;

    struct BuildCommand {
        enum Type { BOX, WALL, FLOOR, PILLAR, ROOM };
        Type type;
        Vector3 position;
        Vector3 size;
        uint8_t material_id;
        bool hollow;
    };

    std::vector<BuildCommand> commands;
};
