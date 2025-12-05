#pragma once

#ifdef USE_BULLET

#include "ParticleSystem.h"
#include "Material.h"
#include "Vector3.h"
#include <btBulletDynamicsCommon.h>
#include <vector>
#include <unordered_map>

// Week 5 Day 28: Impact Detection for Particle Effects

/**
 * @brief Detected impact event
 */
struct ImpactEvent {
    Vector3 position;        // World position of impact
    Vector3 normal;          // Surface normal at impact
    float impulse;           // Impact impulse magnitude (N·s)
    uint8_t material_id;     // Material type at impact point

    ImpactEvent() : position(), normal(0, 1, 0), impulse(0.0f), material_id(0) {}
    ImpactEvent(const Vector3& pos, const Vector3& norm, float imp, uint8_t mat)
        : position(pos), normal(norm), impulse(imp), material_id(mat) {}
};

/**
 * @brief Bullet collision callback for detecting impacts
 */
class ImpactCollisionCallback : public btCollisionWorld::ContactResultCallback {
private:
    float min_impulse_threshold;
    std::vector<ImpactEvent>& impact_events;
    std::unordered_map<const btCollisionObject*, uint8_t>& body_materials;

public:
    ImpactCollisionCallback(float min_impulse,
                           std::vector<ImpactEvent>& events,
                           std::unordered_map<const btCollisionObject*, uint8_t>& materials)
        : min_impulse_threshold(min_impulse),
          impact_events(events),
          body_materials(materials) {}

    btScalar addSingleResult(
        btManifoldPoint& cp,
        const btCollisionObjectWrapper* obj0, int partId0, int index0,
        const btCollisionObjectWrapper* obj1, int partId1, int index1) override;
};

/**
 * @brief Impact detection and particle spawning system
 *
 * Detects collisions in Bullet Physics and spawns appropriate particles
 * based on material type and impact intensity.
 */
class ImpactDetector {
private:
    ParticleSystem* particle_system;
    btDynamicsWorld* dynamics_world;

    float min_impulse_threshold;           // Minimum impulse to trigger effects (N·s)
    std::vector<ImpactEvent> impact_events; // Detected impacts this frame
    std::unordered_map<const btCollisionObject*, uint8_t> body_materials; // Body -> material mapping

public:
    ImpactDetector(ParticleSystem* particles, btDynamicsWorld* world);
    ~ImpactDetector() = default;

    /**
     * @brief Set minimum impulse threshold for impact detection
     * @param threshold Minimum impulse in Newton-seconds (default: 10.0)
     */
    void SetImpulseThreshold(float threshold) { min_impulse_threshold = threshold; }

    /**
     * @brief Get current impulse threshold
     */
    float GetImpulseThreshold() const { return min_impulse_threshold; }

    /**
     * @brief Register material for a physics body
     * @param body Bullet collision object/rigid body
     * @param material_id Material ID (from MaterialDatabase)
     */
    void RegisterBodyMaterial(const btCollisionObject* body, uint8_t material_id);

    /**
     * @brief Unregister a physics body (call when destroying body)
     */
    void UnregisterBody(const btCollisionObject* body);

    /**
     * @brief Detect impacts and spawn particles
     *
     * Should be called after physics step to process collisions
     * from the current frame.
     */
    void DetectAndSpawnParticles();

    /**
     * @brief Get impacts detected in last frame (for debugging/testing)
     */
    const std::vector<ImpactEvent>& GetLastImpacts() const { return impact_events; }

    /**
     * @brief Clear impact history
     */
    void ClearImpacts() { impact_events.clear(); }

private:
    /**
     * @brief Spawn particles for a detected impact
     */
    void SpawnImpactParticles(const ImpactEvent& impact);

    /**
     * @brief Calculate intensity factor from impulse (0.0 - 1.0)
     */
    float CalculateIntensity(float impulse) const;
};

// Helper functions for converting between Bullet and custom types
inline Vector3 BulletToVector3(const btVector3& v) {
    return Vector3(v.x(), v.y(), v.z());
}

inline btVector3 Vector3ToBullet(const Vector3& v) {
    return btVector3(v.x, v.y, v.z);
}

#endif // USE_BULLET
