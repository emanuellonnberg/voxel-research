#include "VoxelUtils.h"
#include <cmath>
#include <queue>
#include <algorithm>

// Surface detection
bool VoxelUtils::IsSurfaceVoxel(const VoxelWorld& world, const Vector3& position,
                                Connectivity connectivity) {
    // A voxel is a surface voxel if it's solid and has at least one air neighbor
    if (!world.HasVoxel(position)) {
        return false;  // Not solid
    }

    int neighbor_count = world.CountNeighbors(position, connectivity);
    int max_neighbors = (connectivity == Connectivity::SIX) ? 6 : 26;

    return neighbor_count < max_neighbors;  // Has at least one air neighbor
}

std::vector<Vector3> VoxelUtils::FindSurfaceVoxels(const VoxelWorld& world,
                                                   Connectivity connectivity) {
    std::vector<Vector3> surface_voxels;
    auto all_positions = world.GetAllVoxelPositions();

    for (const auto& pos : all_positions) {
        if (IsSurfaceVoxel(world, pos, connectivity)) {
            surface_voxels.push_back(pos);
        }
    }

    return surface_voxels;
}

int VoxelUtils::CountExposedFaces(const VoxelWorld& world, const Vector3& position) {
    if (!world.HasVoxel(position)) {
        return 0;
    }

    // Count how many of the 6 faces are exposed to air
    int max_faces = 6;
    int solid_neighbors = world.CountNeighbors(position, Connectivity::SIX);

    return max_faces - solid_neighbors;
}

// Structure generation
void VoxelUtils::CreateBox(VoxelWorld& world, const Vector3& min, const Vector3& max,
                           uint8_t material_id, bool hollow) {
    float voxel_size = world.GetVoxelSize();

    // Iterate through all positions in the box
    for (float x = min.x; x <= max.x; x += voxel_size) {
        for (float y = min.y; y <= max.y; y += voxel_size) {
            for (float z = min.z; z <= max.z; z += voxel_size) {
                Vector3 pos(x, y, z);

                if (hollow) {
                    // Only create voxels on the surface
                    bool is_edge = (std::abs(x - min.x) < voxel_size * 0.1f ||
                                   std::abs(x - max.x) < voxel_size * 0.1f ||
                                   std::abs(y - min.y) < voxel_size * 0.1f ||
                                   std::abs(y - max.y) < voxel_size * 0.1f ||
                                   std::abs(z - min.z) < voxel_size * 0.1f ||
                                   std::abs(z - max.z) < voxel_size * 0.1f);

                    if (is_edge) {
                        world.SetVoxel(pos, Voxel(material_id));
                    }
                } else {
                    // Solid box
                    world.SetVoxel(pos, Voxel(material_id));
                }
            }
        }
    }
}

void VoxelUtils::CreateSphere(VoxelWorld& world, const Vector3& center, float radius,
                              uint8_t material_id, bool hollow) {
    float voxel_size = world.GetVoxelSize();

    // Bounding box around sphere
    Vector3 min = center - Vector3(radius, radius, radius);
    Vector3 max = center + Vector3(radius, radius, radius);

    for (float x = min.x; x <= max.x; x += voxel_size) {
        for (float y = min.y; y <= max.y; y += voxel_size) {
            for (float z = min.z; z <= max.z; z += voxel_size) {
                Vector3 pos(x, y, z);
                float distance = center.Distance(pos);

                if (hollow) {
                    // Shell thickness = 1 voxel
                    if (distance <= radius && distance >= radius - voxel_size) {
                        world.SetVoxel(pos, Voxel(material_id));
                    }
                } else {
                    if (distance <= radius) {
                        world.SetVoxel(pos, Voxel(material_id));
                    }
                }
            }
        }
    }
}

void VoxelUtils::CreateCylinder(VoxelWorld& world, const Vector3& base, float radius,
                                float height, uint8_t material_id, bool hollow) {
    float voxel_size = world.GetVoxelSize();

    // Iterate vertically
    for (float y = 0; y < height; y += voxel_size) {
        // For each horizontal slice, create a circle
        for (float x = -radius; x <= radius; x += voxel_size) {
            for (float z = -radius; z <= radius; z += voxel_size) {
                float distance_from_center = std::sqrt(x * x + z * z);

                if (hollow) {
                    if (distance_from_center <= radius &&
                        distance_from_center >= radius - voxel_size) {
                        Vector3 pos = base + Vector3(x, y, z);
                        world.SetVoxel(pos, Voxel(material_id));
                    }
                } else {
                    if (distance_from_center <= radius) {
                        Vector3 pos = base + Vector3(x, y, z);
                        world.SetVoxel(pos, Voxel(material_id));
                    }
                }
            }
        }
    }
}

void VoxelUtils::CreateWall(VoxelWorld& world, const Vector3& start, const Vector3& end,
                            float thickness, float height, uint8_t material_id) {
    float voxel_size = world.GetVoxelSize();

    // Direction vector from start to end
    Vector3 direction = (end - start).Normalized();
    float length = start.Distance(end);

    // Perpendicular vector for thickness
    Vector3 up(0, 1, 0);
    Vector3 perpendicular = Cross(direction, up).Normalized();

    // Build wall
    for (float dist = 0; dist <= length; dist += voxel_size) {
        for (float h = 0; h < height; h += voxel_size) {
            for (float t = -thickness / 2; t <= thickness / 2; t += voxel_size) {
                Vector3 pos = start + direction * dist + up * h + perpendicular * t;
                world.SetVoxel(pos, Voxel(material_id));
            }
        }
    }
}

void VoxelUtils::CreateFloor(VoxelWorld& world, const Vector3& corner, float width,
                             float depth, float thickness, uint8_t material_id) {
    float voxel_size = world.GetVoxelSize();
    Vector3 min = corner;

    // Adjust thickness for inclusive bounds in CreateBox
    // thickness = voxel_size should create 1 layer, not 2
    float adjusted_thickness = std::max(0.0f, thickness - voxel_size);

    Vector3 max = corner + Vector3(width, adjusted_thickness, depth);
    CreateBox(world, min, max, material_id, false);
}

void VoxelUtils::CreatePillar(VoxelWorld& world, const Vector3& base, float width,
                              float depth, float height, uint8_t material_id) {
    Vector3 min = base;
    Vector3 max = base + Vector3(width, height, depth);
    CreateBox(world, min, max, material_id, false);
}

// Voxel manipulation
void VoxelUtils::FloodFill(VoxelWorld& world, const Vector3& start, uint8_t new_material_id) {
    if (!world.HasVoxel(start)) {
        return;  // No voxel to fill
    }

    Voxel start_voxel = world.GetVoxel(start);
    uint8_t old_material_id = start_voxel.material_id;

    if (old_material_id == new_material_id) {
        return;  // Already the target material
    }

    std::unordered_set<Vector3, Vector3::Hash> visited;
    FloodFillRecursive(world, start, old_material_id, new_material_id, visited);
}

void VoxelUtils::FloodFillRecursive(VoxelWorld& world, const Vector3& position,
                                    uint8_t old_material_id, uint8_t new_material_id,
                                    std::unordered_set<Vector3, Vector3::Hash>& visited) {
    // Check if already visited
    if (visited.find(position) != visited.end()) {
        return;
    }

    // Check if voxel exists and has old material
    if (!world.HasVoxel(position)) {
        return;
    }

    Voxel voxel = world.GetVoxel(position);
    if (voxel.material_id != old_material_id) {
        return;
    }

    // Mark as visited
    visited.insert(position);

    // Change material
    Voxel new_voxel(new_material_id);
    new_voxel.current_hp = voxel.current_hp;  // Preserve health
    world.SetVoxel(position, new_voxel);

    // Recursively fill neighbors
    auto neighbors = world.GetNeighbors(position, Connectivity::SIX);
    for (const auto& neighbor : neighbors) {
        FloodFillRecursive(world, neighbor, old_material_id, new_material_id, visited);
    }
}

void VoxelUtils::ReplaceAll(VoxelWorld& world, uint8_t old_material_id, uint8_t new_material_id) {
    auto all_positions = world.GetAllVoxelPositions();

    for (const auto& pos : all_positions) {
        Voxel voxel = world.GetVoxel(pos);
        if (voxel.material_id == old_material_id) {
            Voxel new_voxel(new_material_id);
            new_voxel.current_hp = voxel.current_hp;
            world.SetVoxel(pos, new_voxel);
        }
    }
}

void VoxelUtils::RemoveDisconnected(VoxelWorld& world, const Vector3& anchor) {
    // Find all voxels connected to anchor
    std::unordered_set<Vector3, Vector3::Hash> connected;
    std::queue<Vector3> to_visit;

    if (!world.HasVoxel(anchor)) {
        return;  // No anchor voxel
    }

    to_visit.push(anchor);
    connected.insert(anchor);

    // BFS to find all connected voxels
    while (!to_visit.empty()) {
        Vector3 current = to_visit.front();
        to_visit.pop();

        auto neighbors = world.GetNeighbors(current, Connectivity::SIX);
        for (const auto& neighbor : neighbors) {
            if (connected.find(neighbor) == connected.end()) {
                connected.insert(neighbor);
                to_visit.push(neighbor);
            }
        }
    }

    // Remove all voxels not in connected set
    auto all_positions = world.GetAllVoxelPositions();
    for (const auto& pos : all_positions) {
        if (connected.find(pos) == connected.end()) {
            world.RemoveVoxel(pos);
        }
    }
}

// Analysis
BoundingBox VoxelUtils::GetBounds(const VoxelWorld& world) {
    auto positions = world.GetAllVoxelPositions();

    if (positions.empty()) {
        return BoundingBox();
    }

    Vector3 min = positions[0];
    Vector3 max = positions[0];

    for (const auto& pos : positions) {
        min = Vector3::Min(min, pos);
        max = Vector3::Max(max, pos);
    }

    return BoundingBox(min, max);
}

Vector3 VoxelUtils::GetCenterOfMass(const VoxelWorld& world) {
    auto positions = world.GetAllVoxelPositions();

    if (positions.empty()) {
        return Vector3::Zero();
    }

    float total_mass = 0;
    Vector3 weighted_sum = Vector3::Zero();
    float voxel_size = world.GetVoxelSize();

    for (const auto& pos : positions) {
        Voxel voxel = world.GetVoxel(pos);
        const Material& mat = g_materials.GetMaterial(voxel.material_id);
        float mass = mat.GetVoxelMass(voxel_size);

        weighted_sum += pos * mass;
        total_mass += mass;
    }

    if (total_mass > 0) {
        return weighted_sum / total_mass;
    }

    return Vector3::Zero();
}

float VoxelUtils::GetTotalMass(const VoxelWorld& world) {
    auto positions = world.GetAllVoxelPositions();
    float total_mass = 0;
    float voxel_size = world.GetVoxelSize();

    for (const auto& pos : positions) {
        Voxel voxel = world.GetVoxel(pos);
        const Material& mat = g_materials.GetMaterial(voxel.material_id);
        total_mass += mat.GetVoxelMass(voxel_size);
    }

    return total_mass;
}

int VoxelUtils::CountVoxelsByMaterial(const VoxelWorld& world, uint8_t material_id) {
    auto positions = world.GetAllVoxelPositions();
    int count = 0;

    for (const auto& pos : positions) {
        Voxel voxel = world.GetVoxel(pos);
        if (voxel.material_id == material_id) {
            count++;
        }
    }

    return count;
}

// StructureBuilder implementation
StructureBuilder::StructureBuilder(VoxelWorld& world)
    : world(world)
    , current_position(Vector3::Zero())
    , current_material(MaterialDatabase::BRICK)
    , is_hollow(false)
{
}

StructureBuilder& StructureBuilder::SetMaterial(uint8_t material_id) {
    current_material = material_id;
    return *this;
}

StructureBuilder& StructureBuilder::SetHollow(bool hollow) {
    is_hollow = hollow;
    return *this;
}

StructureBuilder& StructureBuilder::SetPosition(const Vector3& position) {
    current_position = position;
    return *this;
}

StructureBuilder& StructureBuilder::AddBox(const Vector3& size) {
    BuildCommand cmd;
    cmd.type = BuildCommand::BOX;
    cmd.position = current_position;
    cmd.size = size;
    cmd.material_id = current_material;
    cmd.hollow = is_hollow;
    commands.push_back(cmd);
    return *this;
}

StructureBuilder& StructureBuilder::AddWall(const Vector3& direction, float length, float height) {
    BuildCommand cmd;
    cmd.type = BuildCommand::WALL;
    cmd.position = current_position;
    cmd.size = Vector3(length, height, world.GetVoxelSize());  // thickness = 1 voxel
    cmd.material_id = current_material;
    cmd.hollow = false;
    commands.push_back(cmd);

    // Update position for next component
    current_position += direction.Normalized() * length;
    return *this;
}

StructureBuilder& StructureBuilder::AddFloor(float width, float depth, float thickness) {
    BuildCommand cmd;
    cmd.type = BuildCommand::FLOOR;
    cmd.position = current_position;
    cmd.size = Vector3(width, thickness, depth);
    cmd.material_id = current_material;
    cmd.hollow = false;
    commands.push_back(cmd);
    return *this;
}

StructureBuilder& StructureBuilder::AddPillar(float width, float depth, float height) {
    BuildCommand cmd;
    cmd.type = BuildCommand::PILLAR;
    cmd.position = current_position;
    cmd.size = Vector3(width, height, depth);
    cmd.material_id = current_material;
    cmd.hollow = false;
    commands.push_back(cmd);
    return *this;
}

StructureBuilder& StructureBuilder::AddRoom(float width, float depth, float height) {
    BuildCommand cmd;
    cmd.type = BuildCommand::ROOM;
    cmd.position = current_position;
    cmd.size = Vector3(width, height, depth);
    cmd.material_id = current_material;
    cmd.hollow = true;  // Rooms are always hollow
    commands.push_back(cmd);
    return *this;
}

void StructureBuilder::Build() {
    for (const auto& cmd : commands) {
        Vector3 max_pos = cmd.position + cmd.size;

        switch (cmd.type) {
            case BuildCommand::BOX:
            case BuildCommand::ROOM:
                VoxelUtils::CreateBox(world, cmd.position, max_pos,
                                     cmd.material_id, cmd.hollow);
                break;

            case BuildCommand::FLOOR:
                VoxelUtils::CreateFloor(world, cmd.position, cmd.size.x,
                                       cmd.size.z, cmd.size.y, cmd.material_id);
                break;

            case BuildCommand::PILLAR:
                VoxelUtils::CreatePillar(world, cmd.position, cmd.size.x,
                                        cmd.size.z, cmd.size.y, cmd.material_id);
                break;

            case BuildCommand::WALL:
                // Wall handled specially with direction
                break;
        }
    }

    commands.clear();
}

void StructureBuilder::Clear() {
    commands.clear();
}
