#pragma once

#include <cstdint>

/**
 * Voxel - Individual voxel data
 *
 * Represents a single voxel in the world. Uses minimal memory (3 bytes).
 * Material properties are looked up from the global material database.
 *
 * Design: Keep this struct small since we'll have millions of voxels.
 */
struct Voxel {
    uint8_t material_id;     // Index into material database (0 = air)
    uint8_t current_hp;      // Current health (0-255, for damage visualization)
    bool is_active;          // true = solid voxel, false = air/removed

    // Default constructor - creates air voxel
    Voxel()
        : material_id(0)
        , current_hp(0)
        , is_active(false)
    {}

    // Constructor with material
    Voxel(uint8_t mat_id)
        : material_id(mat_id)
        , current_hp(255)  // Full health
        , is_active(true)
    {}

    // Check if this voxel is solid (not air)
    bool IsSolid() const {
        return is_active && material_id != 0;
    }

    // Get health as percentage (0.0 = destroyed, 1.0 = full health)
    float GetHealthPercent() const {
        return current_hp / 255.0f;
    }

    // Apply damage to this voxel
    void TakeDamage(uint8_t damage) {
        if (current_hp > damage) {
            current_hp -= damage;
        } else {
            current_hp = 0;
            is_active = false;  // Destroyed
        }
    }
};

/**
 * Connectivity - How many neighbors to check
 *
 * 6-connected: Only face neighbors (N, S, E, W, U, D)
 * 26-connected: Face + edge + corner neighbors (full 3x3x3 cube)
 */
enum class Connectivity {
    SIX = 6,           // Face neighbors only (faster)
    TWENTY_SIX = 26    // All 26 neighbors (more accurate)
};
