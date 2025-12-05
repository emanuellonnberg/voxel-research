#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <cstdint>

/**
 * Color - RGBA color representation
 */
struct Color {
    float r, g, b, a;

    Color() : r(0), g(0), b(0), a(1.0f) {}
    Color(float r, float g, float b, float a = 1.0f) : r(r), g(g), b(b), a(a) {}

    // Named colors
    static Color Black() { return Color(0.0f, 0.0f, 0.0f); }
    static Color White() { return Color(1.0f, 1.0f, 1.0f); }
    static Color Red() { return Color(1.0f, 0.0f, 0.0f); }
    static Color Red(float alpha) { return Color(0.8f, 0.3f, 0.2f, alpha); }
    static Color Green() { return Color(0.0f, 1.0f, 0.0f); }
    static Color Blue() { return Color(0.0f, 0.0f, 1.0f); }
    static Color Yellow() { return Color(1.0f, 1.0f, 0.0f); }
    static Color Brown() { return Color(0.6f, 0.4f, 0.2f); }
    static Color Brown(float alpha) { return Color(0.55f, 0.35f, 0.2f, alpha); }
    static Color Gray() { return Color(0.5f, 0.5f, 0.5f); }
    static Color Gray(float brightness, float alpha) { return Color(brightness, brightness, brightness * 0.9f, alpha); }
    static Color DarkGray() { return Color(0.3f, 0.3f, 0.3f); }

    // Linear interpolation
    static Color Lerp(const Color& a, const Color& b, float t) {
        return Color(
            a.r + (b.r - a.r) * t,
            a.g + (b.g - a.g) * t,
            a.b + (b.b - a.b) * t,
            a.a + (b.a - a.a) * t
        );
    }
};

/**
 * Material - Physical properties of voxel materials
 *
 * Properties needed for:
 * - Structural analysis (spring constants, max displacement)
 * - Physics simulation (density, friction)
 * - Rendering (color)
 */
struct Material {
    // Identification
    std::string name;
    uint8_t id;  // Material ID (0-255)

    // Physical properties
    float density;                // kg/m³ (for mass calculation)
    float compressive_strength;   // N/m² (Pascals) - resistance to crushing

    // Structural analysis properties (Track 1)
    float spring_constant;        // N/m - stiffness in spring system
    float max_displacement;       // meters - failure threshold

    // Physics properties (Track 3)
    float static_friction;        // 0-1 - friction when stationary
    float dynamic_friction;       // 0-1 - friction when moving
    float restitution;            // 0-1 - bounciness (0 = no bounce, 1 = perfect bounce)

    // Rendering
    Color color;

    // Constructors
    Material()
        : name("unknown"), id(0)
        , density(0), compressive_strength(0)
        , spring_constant(0), max_displacement(0)
        , static_friction(0.5f), dynamic_friction(0.4f), restitution(0.3f)
        , color(Color::Gray())
    {}

    Material(const std::string& name, uint8_t id, float density, float strength)
        : name(name), id(id)
        , density(density), compressive_strength(strength)
        , spring_constant(strength * 0.001f)  // Default: proportional to strength
        , max_displacement(0.01f)              // Default: 1cm
        , static_friction(0.5f), dynamic_friction(0.4f), restitution(0.3f)
        , color(Color::Gray())
    {}

    // Calculate mass of a voxel of this material
    float GetVoxelMass(float voxel_size) const {
        float volume = voxel_size * voxel_size * voxel_size;
        return density * volume;
    }
};

/**
 * MaterialDatabase - Centralized material definitions
 *
 * Provides access to all materials used in the game.
 * Materials are defined once and referenced by ID.
 */
class MaterialDatabase {
public:
    // Material IDs (indices into materials array)
    enum MaterialID : uint8_t {
        AIR = 0,        // Empty space
        WOOD = 1,       // Light, flexible
        BRICK = 2,      // Medium strength
        CONCRETE = 3,   // Heavy, strong
        STEEL = 4,      // Very strong (future use)
        // Add more as needed
        MAX_MATERIALS = 255
    };

private:
    std::vector<Material> materials;
    std::unordered_map<std::string, uint8_t> name_to_id;

public:
    MaterialDatabase();

    // Access materials
    const Material& GetMaterial(uint8_t id) const;
    const Material& GetMaterial(const std::string& name) const;
    uint8_t GetMaterialID(const std::string& name) const;

    // Query
    size_t GetMaterialCount() const { return materials.size(); }
    const std::vector<Material>& GetAllMaterials() const { return materials; }

private:
    void RegisterMaterial(const Material& mat);
    void InitializeDefaultMaterials();
};

// Global material database instance (defined in Material.cpp)
extern MaterialDatabase g_materials;
