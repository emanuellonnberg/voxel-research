#include "Material.h"
#include <stdexcept>

// Global material database instance
MaterialDatabase g_materials;

MaterialDatabase::MaterialDatabase() {
    // Reserve space for materials
    materials.reserve(16);

    // Initialize with default materials
    InitializeDefaultMaterials();
}

void MaterialDatabase::RegisterMaterial(const Material& mat) {
    if (mat.id >= materials.size()) {
        materials.resize(mat.id + 1);
    }
    materials[mat.id] = mat;
    name_to_id[mat.name] = mat.id;
}

void MaterialDatabase::InitializeDefaultMaterials() {
    // Air (empty space)
    {
        Material air;
        air.name = "air";
        air.id = AIR;
        air.density = 0.0f;
        air.compressive_strength = 0.0f;
        air.spring_constant = 0.0f;
        air.max_displacement = 0.0f;
        air.max_support_distance = 0.0f;
        air.min_support_stack = 0.0f;
        air.max_load_ratio = 0.0f;
        air.color = Color(0, 0, 0);
        RegisterMaterial(air);
    }

    // Wood
    // Real properties: density ~600 kg/m³, compressive strength ~10 MPa
    {
        Material wood;
        wood.name = "wood";
        wood.id = WOOD;
        wood.density = 600.0f;                    // kg/m³
        wood.compressive_strength = 10e6f;        // 10 MPa = 10,000,000 Pa
        wood.spring_constant = 10000.0f;          // N/m (tuned for gameplay)
        wood.max_displacement = 0.020f;           // 2cm (flexible)
        wood.static_friction = 0.6f;
        wood.dynamic_friction = 0.5f;
        wood.restitution = 0.4f;                  // Slightly bouncy
        wood.max_support_distance = 0.5f;         // Supports up to ~0.5m from ground
        wood.min_support_stack = 0.02f;           // Needs at least a thin support stack
        wood.max_load_ratio = 1.5f;               // Can carry ~1.5x its own weight
        wood.color = Color(0.72f, 0.5f, 0.28f);   // Warm brown
        RegisterMaterial(wood);
    }

    // Brick
    // Real properties: density ~1900 kg/m³, compressive strength ~30 MPa
    {
        Material brick;
        brick.name = "brick";
        brick.id = BRICK;
        brick.density = 1900.0f;                  // kg/m³
        brick.compressive_strength = 30e6f;       // 30 MPa
        brick.spring_constant = 50000.0f;         // N/m (tuned for gameplay)
        brick.max_displacement = 0.010f;          // 1cm (less flexible)
        brick.static_friction = 0.8f;
        brick.dynamic_friction = 0.7f;
        brick.restitution = 0.2f;                 // Not very bouncy
        brick.max_support_distance = 2.0f;        // Can span moderate distances
        brick.min_support_stack = 0.05f;          // Needs at least 1 voxel of support
        brick.max_load_ratio = 2.5f;
        brick.color = Color(0.85f, 0.23f, 0.18f); // Vibrant brick red
        RegisterMaterial(brick);
    }

    // Concrete
    // Real properties: density ~2400 kg/m³, compressive strength ~40 MPa
    {
        Material concrete;
        concrete.name = "concrete";
        concrete.id = CONCRETE;
        concrete.density = 2400.0f;               // kg/m³
        concrete.compressive_strength = 40e6f;    // 40 MPa
        concrete.spring_constant = 100000.0f;     // N/m (tuned for gameplay)
        concrete.max_displacement = 0.005f;       // 0.5cm (very stiff)
        concrete.static_friction = 0.9f;
        concrete.dynamic_friction = 0.8f;
        concrete.restitution = 0.1f;              // Very little bounce
        concrete.max_support_distance = 3.0f;
        concrete.min_support_stack = 0.1f;        // Requires multiple voxels of support
        concrete.max_load_ratio = 3.0f;
        concrete.color = Color(0.75f, 0.77f, 0.80f); // Light gray
        RegisterMaterial(concrete);
    }

    // Steel (for future use)
    // Real properties: density ~7850 kg/m³, compressive strength ~250 MPa
    {
        Material steel;
        steel.name = "steel";
        steel.id = STEEL;
        steel.density = 7850.0f;                  // kg/m³
        steel.compressive_strength = 250e6f;      // 250 MPa
        steel.spring_constant = 500000.0f;        // N/m (very stiff)
        steel.max_displacement = 0.002f;          // 0.2cm (extremely stiff)
        steel.static_friction = 0.7f;
        steel.dynamic_friction = 0.6f;
        steel.restitution = 0.5f;                 // Moderately bouncy
        steel.max_support_distance = 5.0f;
        steel.min_support_stack = 0.0f;           // Steel beams can self-span
        steel.max_load_ratio = 5.0f;
        steel.color = Color(0.6f, 0.72f, 0.85f);  // Cool metallic
        RegisterMaterial(steel);
    }
}

const Material& MaterialDatabase::GetMaterial(uint8_t id) const {
    if (id >= materials.size()) {
        throw std::out_of_range("Material ID out of range: " + std::to_string(id));
    }
    return materials[id];
}

const Material& MaterialDatabase::GetMaterial(const std::string& name) const {
    auto it = name_to_id.find(name);
    if (it == name_to_id.end()) {
        throw std::runtime_error("Material not found: " + name);
    }
    return materials[it->second];
}

uint8_t MaterialDatabase::GetMaterialID(const std::string& name) const {
    auto it = name_to_id.find(name);
    if (it == name_to_id.end()) {
        throw std::runtime_error("Material not found: " + name);
    }
    return it->second;
}
