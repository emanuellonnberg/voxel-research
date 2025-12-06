#ifdef USE_BULLET

#include "ImpactDetector.h"
#include "Material.h"
#include <algorithm>
#include <cmath>

// Week 5 Day 28: Impact Detection Implementation

btScalar ImpactCollisionCallback::addSingleResult(
    btManifoldPoint& cp,
    const btCollisionObjectWrapper* obj0, int partId0, int index0,
    const btCollisionObjectWrapper* obj1, int partId1, int index1) {

    // Get impact impulse
    float impulse = cp.getAppliedImpulse();

    // Only process impacts above threshold
    if (impulse < min_impulse_threshold) {
        return 0;
    }

    // Get impact position and normal
    Vector3 position = BulletToVector3(cp.getPositionWorldOnA());
    Vector3 normal = BulletToVector3(cp.m_normalWorldOnB);

    // Determine material from body
    uint8_t material_id = MaterialDatabase::CONCRETE; // Default
    auto it0 = body_materials.find(obj0->getCollisionObject());
    auto it1 = body_materials.find(obj1->getCollisionObject());

    if (it0 != body_materials.end()) {
        material_id = it0->second;
    } else if (it1 != body_materials.end()) {
        material_id = it1->second;
    }

    // Record impact event
    impact_events.emplace_back(position, normal, impulse, material_id);

    return 0;
}

ImpactDetector::ImpactDetector(ParticleSystem* particles, btDynamicsWorld* world)
    : particle_system(particles),
      dynamics_world(world),
      min_impulse_threshold(10.0f) {
    impact_events.reserve(100); // Pre-allocate for performance
}

void ImpactDetector::RegisterBodyMaterial(const btCollisionObject* body, uint8_t material_id) {
    body_materials[body] = material_id;
}

void ImpactDetector::UnregisterBody(const btCollisionObject* body) {
    body_materials.erase(body);
}

void ImpactDetector::DetectAndSpawnParticles() {
    if (!particle_system || !dynamics_world) {
        return;
    }

    // Clear previous frame's impacts
    impact_events.clear();

    // Get all collision manifolds from the dispatcher
    btDispatcher* dispatcher = dynamics_world->getDispatcher();
    int num_manifolds = dispatcher->getNumManifolds();

    for (int i = 0; i < num_manifolds; i++) {
        btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(i);

        const btCollisionObject* obj0 = manifold->getBody0();
        const btCollisionObject* obj1 = manifold->getBody1();

        int num_contacts = manifold->getNumContacts();

        for (int j = 0; j < num_contacts; j++) {
            btManifoldPoint& point = manifold->getContactPoint(j);

            // Get impact impulse
            float impulse = point.getAppliedImpulse();

            // Only process impacts above threshold
            if (impulse < min_impulse_threshold) {
                continue;
            }

            // Get impact position and normal
            Vector3 position = BulletToVector3(point.getPositionWorldOnA());
            Vector3 normal = BulletToVector3(point.m_normalWorldOnB);

            // Determine material from body
            uint8_t material_id = MaterialDatabase::CONCRETE; // Default
            auto it0 = body_materials.find(obj0);
            auto it1 = body_materials.find(obj1);

            if (it0 != body_materials.end()) {
                material_id = it0->second;
            } else if (it1 != body_materials.end()) {
                material_id = it1->second;
            }

            // Record and spawn particles for this impact
            ImpactEvent impact(position, normal, impulse, material_id);
            impact_events.push_back(impact);
            SpawnImpactParticles(impact);
        }
    }
}

void ImpactDetector::SpawnImpactParticles(const ImpactEvent& impact) {
    if (!particle_system) {
        return;
    }

    // Calculate intensity based on impulse
    float intensity = CalculateIntensity(impact.impulse);

    // Spawn material-specific particles
    switch (impact.material_id) {
        case MaterialDatabase::CONCRETE:
            particle_system->SpawnConcreteImpact(impact.position, impact.normal, intensity);
            break;

        case MaterialDatabase::WOOD:
            particle_system->SpawnWoodImpact(impact.position, impact.normal, intensity);
            break;

        case MaterialDatabase::BRICK:
            particle_system->SpawnBrickImpact(impact.position, impact.normal, intensity);
            break;

        default:
            // For unknown materials, use concrete as fallback
            particle_system->SpawnConcreteImpact(impact.position, impact.normal, intensity);
            break;
    }
}

float ImpactDetector::CalculateIntensity(float impulse) const {
    // Map impulse to intensity (0.0 - 1.0)
    // Uses logarithmic scale for better range

    // impulse <= threshold -> intensity = 0.3 (minimum)
    // impulse = 100 -> intensity = 1.0 (maximum)

    if (impulse <= min_impulse_threshold) {
        return 0.3f;
    }

    // Logarithmic mapping
    float log_impulse = std::log10(impulse);
    float log_min = std::log10(min_impulse_threshold);
    float log_max = std::log10(100.0f);

    float intensity = (log_impulse - log_min) / (log_max - log_min);
    intensity = 0.3f + intensity * 0.7f; // Scale to [0.3, 1.0]

    // Clamp to valid range
    return std::max(0.3f, std::min(1.0f, intensity));
}

#endif // USE_BULLET
