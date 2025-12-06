#include "ParticleSystem.h"
#include "JobSystem.h"  // Week 13 Day 43: Parallel particle updates
#include <cmath>
#include <algorithm>

// Week 5 Day 28: Particle Effects Implementation

ParticleSystem::ParticleSystem()
    : gravity(0.0f, -9.81f, 0.0f),
      air_resistance(0.02f),
      max_particles(10000) {
    particles.reserve(1000); // Pre-allocate for performance
}

void ParticleSystem::Spawn(const Vector3& position, const Vector3& velocity,
                          const Color& color, float size, float lifetime,
                          uint8_t material_id) {
    // Check particle limit
    if (static_cast<int>(particles.size()) >= max_particles) {
        return; // Drop particle if at limit
    }

    Particle p;
    p.position = position;
    p.velocity = velocity;
    p.color = color;
    p.size = size;
    p.lifetime = lifetime;
    p.age = 0.0f;
    p.material_id = material_id;

    particles.push_back(p);
}

void ParticleSystem::Update(float deltaTime) {
    // Week 13 Day 43: Parallel particle updates for large particle counts
    const bool use_parallel = g_JobSystem &&
                              g_JobSystem->IsRunning() &&
                              particles.size() > 500;  // Only parallelize if many particles

    if (use_parallel) {
        // Step 1: Update particles in parallel
        g_JobSystem->ParallelFor(static_cast<int>(particles.size()), [&](int i) {
            Particle& p = particles[i];

            // Age the particle
            p.age += deltaTime;

            // Only update if alive
            if (p.IsAlive()) {
                // Apply gravity
                p.velocity = p.velocity + gravity * deltaTime;

                // Apply air resistance (simple drag)
                if (air_resistance > 0.0f) {
                    float drag = 1.0f - (air_resistance * deltaTime);
                    drag = std::max(0.0f, drag);
                    p.velocity = p.velocity * drag;
                }

                // Update position (Euler integration)
                p.position = p.position + p.velocity * deltaTime;

                // Update alpha based on fade
                float fade = p.GetFade();
                p.color.a = p.color.a * fade;
            }
        });

        // Step 2: Remove dead particles (must be serial)
        auto it = particles.begin();
        while (it != particles.end()) {
            if (!it->IsAlive()) {
                it = particles.erase(it);
            } else {
                ++it;
            }
        }
    } else {
        // Serial path: Original implementation
        auto it = particles.begin();

        while (it != particles.end()) {
            // Age the particle
            it->age += deltaTime;

            // Remove if dead
            if (!it->IsAlive()) {
                it = particles.erase(it);
                continue;
            }

            // Apply gravity
            it->velocity = it->velocity + gravity * deltaTime;

            // Apply air resistance (simple drag)
            if (air_resistance > 0.0f) {
                float drag = 1.0f - (air_resistance * deltaTime);
                drag = std::max(0.0f, drag);
                it->velocity = it->velocity * drag;
            }

            // Update position (Euler integration)
            it->position = it->position + it->velocity * deltaTime;

            // Update alpha based on fade
            float fade = it->GetFade();
            it->color.a = it->color.a * fade;

            ++it;
        }
    }
}

void ParticleSystem::Clear() {
    particles.clear();
}

// Helper function for random range
static float RandomRange(float min, float max) {
    float rand_val = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    return min + rand_val * (max - min);
}

void ParticleSystem::SpawnConcreteImpact(const Vector3& position,
                                        const Vector3& normal,
                                        float intensity) {
    int dust_count = static_cast<int>(20.0f * intensity);
    int chip_count = static_cast<int>(10.0f * intensity);

    // Spawn dust cloud
    for (int i = 0; i < dust_count; i++) {
        // Random direction biased by surface normal
        Vector3 vel(
            RandomRange(-1.0f, 1.0f) + normal.x * 0.5f,
            RandomRange(0.0f, 2.0f) + normal.y * 0.5f,
            RandomRange(-1.0f, 1.0f) + normal.z * 0.5f
        );

        // Gray dust with some variation
        Color dust_color = Color::Gray(RandomRange(0.6f, 0.8f), 0.5f);

        Spawn(
            position,
            vel,
            dust_color,
            RandomRange(0.15f, 0.25f) * intensity,  // Size
            RandomRange(1.5f, 2.5f),                // Lifetime
            1  // Concrete material ID
        );
    }

    // Spawn rock chips (smaller, faster, more solid)
    for (int i = 0; i < chip_count; i++) {
        Vector3 vel(
            RandomRange(-2.0f, 2.0f) + normal.x * 2.0f,
            RandomRange(1.0f, 4.0f) + normal.y * 2.0f,
            RandomRange(-2.0f, 2.0f) + normal.z * 2.0f
        );

        // Darker gray for rock chips
        Color chip_color(0.5f, 0.5f, 0.5f, 1.0f);

        Spawn(
            position,
            vel,
            chip_color,
            RandomRange(0.03f, 0.07f) * intensity,  // Smaller
            RandomRange(0.8f, 1.2f),                // Shorter lifetime
            1  // Concrete material ID
        );
    }
}

void ParticleSystem::SpawnWoodImpact(const Vector3& position,
                                    const Vector3& normal,
                                    float intensity) {
    int splinter_count = static_cast<int>(15.0f * intensity);
    int dust_count = static_cast<int>(8.0f * intensity);

    // Spawn wood splinters (elongated, faster)
    for (int i = 0; i < splinter_count; i++) {
        Vector3 vel(
            RandomRange(-3.0f, 3.0f) + normal.x * 2.0f,
            RandomRange(0.5f, 3.0f) + normal.y * 2.0f,
            RandomRange(-3.0f, 3.0f) + normal.z * 2.0f
        );

        // Brown color for wood
        Color splinter_color = Color::Brown(1.0f);

        Spawn(
            position,
            vel,
            splinter_color,
            RandomRange(0.08f, 0.15f) * intensity,  // Elongated
            RandomRange(1.0f, 1.5f),                // Medium lifetime
            2  // Wood material ID
        );
    }

    // Spawn sawdust (light, floaty)
    for (int i = 0; i < dust_count; i++) {
        Vector3 vel(
            RandomRange(-0.5f, 0.5f) + normal.x * 0.3f,
            RandomRange(0.5f, 1.5f) + normal.y * 0.3f,
            RandomRange(-0.5f, 0.5f) + normal.z * 0.3f
        );

        // Light brown/tan dust
        Color dust_color(0.7f, 0.55f, 0.35f, 0.6f);

        Spawn(
            position,
            vel,
            dust_color,
            RandomRange(0.05f, 0.1f) * intensity,
            RandomRange(2.0f, 3.0f),  // Floats longer
            2  // Wood material ID
        );
    }
}

void ParticleSystem::SpawnBrickImpact(const Vector3& position,
                                     const Vector3& normal,
                                     float intensity) {
    int dust_count = static_cast<int>(18.0f * intensity);
    int brick_count = static_cast<int>(12.0f * intensity);

    // Spawn brick dust
    for (int i = 0; i < dust_count; i++) {
        Vector3 vel(
            RandomRange(-1.0f, 1.0f) + normal.x * 0.5f,
            RandomRange(0.0f, 2.0f) + normal.y * 0.5f,
            RandomRange(-1.0f, 1.0f) + normal.z * 0.5f
        );

        // Reddish-brown dust
        Color dust_color(0.7f, 0.4f, 0.3f, 0.5f);

        Spawn(
            position,
            vel,
            dust_color,
            RandomRange(0.12f, 0.22f) * intensity,
            RandomRange(1.8f, 2.5f),
            3  // Brick material ID
        );
    }

    // Spawn brick pieces
    for (int i = 0; i < brick_count; i++) {
        Vector3 vel(
            RandomRange(-2.5f, 2.5f) + normal.x * 2.0f,
            RandomRange(1.0f, 3.5f) + normal.y * 2.0f,
            RandomRange(-2.5f, 2.5f) + normal.z * 2.0f
        );

        // Red brick pieces
        Color brick_color = Color::Red(1.0f);

        Spawn(
            position,
            vel,
            brick_color,
            RandomRange(0.04f, 0.08f) * intensity,
            RandomRange(0.9f, 1.3f),
            3  // Brick material ID
        );
    }
}

void ParticleSystem::SpawnDustCloud(const Vector3& epicenter, float radius,
                                   int particle_count) {
    for (int i = 0; i < particle_count; i++) {
        // Random position within radius
        Vector3 offset(
            RandomRange(-radius, radius),
            RandomRange(0.0f, radius),  // Bias upward
            RandomRange(-radius, radius)
        );

        Vector3 pos = epicenter + offset;

        // Slow, upward-biased velocity
        Vector3 vel(
            RandomRange(-0.5f, 0.5f),
            RandomRange(0.2f, 1.0f),  // Float upward
            RandomRange(-0.5f, 0.5f)
        );

        // Large, translucent dust clouds
        Color dust_color = Color::Gray(0.6f, 0.3f);

        Spawn(
            pos,
            vel,
            dust_color,
            RandomRange(0.3f, 0.6f),  // Large clouds
            RandomRange(4.0f, 6.0f),  // Long lifetime
            0  // Generic dust
        );
    }
}
