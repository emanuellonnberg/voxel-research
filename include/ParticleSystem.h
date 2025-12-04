#pragma once

#include "Vector3.h"
#include <vector>
#include <cstdint>

// Week 5 Day 28: Particle Effects System

/**
 * @brief RGBA color representation for particles
 */
struct Color {
    float r, g, b, a;

    Color() : r(1.0f), g(1.0f), b(1.0f), a(1.0f) {}
    Color(float red, float green, float blue, float alpha = 1.0f)
        : r(red), g(green), b(blue), a(alpha) {}

    static Color Gray(float brightness = 0.7f, float alpha = 0.5f) {
        return Color(brightness, brightness, brightness * 0.9f, alpha);
    }

    static Color Brown(float alpha = 1.0f) {
        return Color(0.55f, 0.35f, 0.2f, alpha);
    }

    static Color Red(float alpha = 1.0f) {
        return Color(0.8f, 0.3f, 0.2f, alpha);
    }
};

/**
 * @brief Individual particle for visual effects
 *
 * Represents a single particle in the particle system with position,
 * velocity, visual properties, and lifetime management.
 */
struct Particle {
    Vector3 position;      // World position
    Vector3 velocity;      // Velocity (m/s)
    Color color;           // RGBA color
    float size;            // Size in meters
    float lifetime;        // Total lifetime in seconds
    float age;             // Current age in seconds
    uint8_t material_id;   // Material type for rendering

    Particle()
        : position(0, 0, 0), velocity(0, 0, 0), color(),
          size(0.1f), lifetime(1.0f), age(0.0f), material_id(0) {}

    /**
     * @brief Check if particle is still alive
     * @return true if age < lifetime
     */
    bool IsAlive() const { return age < lifetime; }

    /**
     * @brief Get fade factor based on age (0.0 = dead, 1.0 = full opacity)
     */
    float GetFade() const {
        if (lifetime <= 0.0f) return 0.0f;
        return 1.0f - (age / lifetime);
    }
};

/**
 * @brief Particle system for visual effects
 *
 * Manages particles for debris impacts, dust clouds, and destruction effects.
 * Handles particle spawning, physics simulation, and lifetime management.
 *
 * Week 5 Day 28: Visual polish for destruction
 */
class ParticleSystem {
private:
    std::vector<Particle> particles;
    Vector3 gravity;           // Gravity acceleration (default: 0, -9.81, 0)
    float air_resistance;      // Air resistance coefficient (0.0 - 1.0)
    int max_particles;         // Maximum particle count (for performance)

public:
    ParticleSystem();
    ~ParticleSystem() = default;

    // Configuration
    void SetGravity(const Vector3& g) { gravity = g; }
    void SetAirResistance(float resistance) { air_resistance = resistance; }
    void SetMaxParticles(int max) { max_particles = max; }

    /**
     * @brief Spawn a single particle
     * @param position Starting position
     * @param velocity Initial velocity
     * @param color Particle color (RGBA)
     * @param size Size in meters
     * @param lifetime Lifetime in seconds
     * @param material_id Material type for rendering
     */
    void Spawn(const Vector3& position, const Vector3& velocity,
               const Color& color, float size, float lifetime,
               uint8_t material_id = 0);

    /**
     * @brief Update all particles
     * @param deltaTime Time step in seconds
     *
     * Updates particle positions, applies gravity and air resistance,
     * ages particles, and removes dead particles.
     */
    void Update(float deltaTime);

    /**
     * @brief Clear all particles
     */
    void Clear();

    /**
     * @brief Get all active particles (for rendering)
     */
    const std::vector<Particle>& GetParticles() const { return particles; }

    /**
     * @brief Get active particle count
     */
    int GetParticleCount() const { return static_cast<int>(particles.size()); }

    // Material-specific particle effects

    /**
     * @brief Spawn concrete impact particles (dust + rock chips)
     * @param position Impact position
     * @param normal Surface normal at impact
     * @param intensity Impact intensity (0.0 - 1.0)
     */
    void SpawnConcreteImpact(const Vector3& position, const Vector3& normal,
                            float intensity = 1.0f);

    /**
     * @brief Spawn wood impact particles (splinters + sawdust)
     * @param position Impact position
     * @param normal Surface normal at impact
     * @param intensity Impact intensity (0.0 - 1.0)
     */
    void SpawnWoodImpact(const Vector3& position, const Vector3& normal,
                        float intensity = 1.0f);

    /**
     * @brief Spawn brick impact particles (dust + brick pieces)
     * @param position Impact position
     * @param normal Surface normal at impact
     * @param intensity Impact intensity (0.0 - 1.0)
     */
    void SpawnBrickImpact(const Vector3& position, const Vector3& normal,
                         float intensity = 1.0f);

    /**
     * @brief Spawn ambient dust cloud over area
     * @param epicenter Center of dust cloud
     * @param radius Radius of dust cloud
     * @param particle_count Number of particles to spawn
     */
    void SpawnDustCloud(const Vector3& epicenter, float radius,
                       int particle_count = 50);
};
