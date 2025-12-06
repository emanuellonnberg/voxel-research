#pragma once

#include "Vector3.h"
#include "Material.h"
#include <vector>
#include <cstdint>

/**
 * Rubble System (Week 5 Day 27)
 *
 * Tracks settled debris as static "rubble" objects for gameplay features.
 * When physics debris settle, they can be converted to rubble to:
 * - Reduce physics simulation cost
 * - Provide cover height queries for gameplay
 * - Enable rubble cleanup/removal
 * - Support environmental effects
 *
 * Use Cases:
 * - Cover system: Calculate cover height at player position
 * - Navigation: Query rubble obstacles for pathfinding
 * - Cleanup: Remove old rubble to manage performance
 * - Visual effects: Dust, debris, particle effects
 */

/**
 * Individual rubble piece
 * Represents a settled debris object that's no longer simulated
 */
struct RubbleObject {
    Vector3 position;       // Center position of rubble
    float height;           // Height of rubble pile (for cover calculation)
    float radius;           // Influence radius (for queries)
    uint8_t material_id;    // Material type (for rendering/effects)

    RubbleObject()
        : position(Vector3::Zero())
        , height(0.0f)
        , radius(0.5f)
        , material_id(0)
    {}

    RubbleObject(const Vector3& pos, float h, float r, uint8_t mat_id)
        : position(pos)
        , height(h)
        , radius(r)
        , material_id(mat_id)
    {}
};

/**
 * Rubble system for tracking settled debris
 */
class RubbleSystem {
public:
    RubbleSystem();
    ~RubbleSystem() = default;

    // ===== Rubble Management =====

    /**
     * Add a rubble piece at the given location
     *
     * @param position Center position of rubble
     * @param height Height of rubble pile
     * @param material_id Material type
     * @param radius Influence radius (default: 0.5m)
     */
    void AddRubble(const Vector3& position, float height, uint8_t material_id, float radius = 0.5f);

    /**
     * Remove rubble near a position (within removal_radius)
     * Returns number of rubble pieces removed
     *
     * @param position Center of removal area
     * @param removal_radius Radius within which to remove rubble
     */
    int RemoveRubbleNear(const Vector3& position, float removal_radius);

    /**
     * Clear all rubble
     */
    void ClearAll();

    // ===== Queries =====

    /**
     * Get all rubble pieces within a radius of a position
     * Useful for proximity queries, navigation, etc.
     *
     * @param position Query center
     * @param radius Query radius
     * @return Vector of rubble objects within radius
     */
    std::vector<RubbleObject> GetRubbleNearby(const Vector3& position, float radius) const;

    /**
     * Get cover height at a position (for cover system)
     * Averages height of all rubble pieces within their influence radius
     *
     * @param position Query position (e.g., player position)
     * @return Average rubble height (0 if no rubble nearby)
     */
    float GetCoverHeight(const Vector3& position) const;

    /**
     * Check if there's significant rubble at a position
     * Useful for quick checks without height calculation
     *
     * @param position Query position
     * @param min_height Minimum height to consider as cover (default: 0.5m)
     * @return True if cover height >= min_height
     */
    bool HasCover(const Vector3& position, float min_height = 0.5f) const;

    /**
     * Get total number of rubble pieces
     */
    int GetRubbleCount() const;

    /**
     * Get all rubble pieces (for rendering, etc.)
     */
    const std::vector<RubbleObject>& GetAllRubble() const;

private:
    std::vector<RubbleObject> rubble_pieces;

    // Helper: Calculate 2D distance (ignoring Y axis)
    float Distance2D(const Vector3& a, const Vector3& b) const;
};
