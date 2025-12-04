#include <gtest/gtest.h>
#include "ParticleSystem.h"
#include <cmath>

// Week 5 Day 28: Particle System Tests

class ParticleSystemTest : public ::testing::Test {
protected:
    ParticleSystem* particle_system;

    void SetUp() override {
        particle_system = new ParticleSystem();
        srand(12345); // Fixed seed for reproducibility
    }

    void TearDown() override {
        delete particle_system;
    }
};

// ===== Color Tests =====

TEST_F(ParticleSystemTest, ColorConstruction) {
    Color c1;
    EXPECT_FLOAT_EQ(c1.r, 1.0f);
    EXPECT_FLOAT_EQ(c1.g, 1.0f);
    EXPECT_FLOAT_EQ(c1.b, 1.0f);
    EXPECT_FLOAT_EQ(c1.a, 1.0f);

    Color c2(0.5f, 0.6f, 0.7f, 0.8f);
    EXPECT_FLOAT_EQ(c2.r, 0.5f);
    EXPECT_FLOAT_EQ(c2.g, 0.6f);
    EXPECT_FLOAT_EQ(c2.b, 0.7f);
    EXPECT_FLOAT_EQ(c2.a, 0.8f);
}

TEST_F(ParticleSystemTest, ColorPresets) {
    Color gray = Color::Gray(0.7f, 0.5f);
    EXPECT_FLOAT_EQ(gray.a, 0.5f);
    EXPECT_NEAR(gray.r, 0.7f, 0.01f);

    Color brown = Color::Brown(0.8f);
    EXPECT_FLOAT_EQ(brown.a, 0.8f);
    EXPECT_NEAR(brown.r, 0.55f, 0.01f);

    Color red = Color::Red(0.9f);
    EXPECT_FLOAT_EQ(red.a, 0.9f);
    EXPECT_NEAR(red.r, 0.8f, 0.1f);
}

// ===== Particle Tests =====

TEST_F(ParticleSystemTest, ParticleConstruction) {
    Particle p;
    EXPECT_FLOAT_EQ(p.position.x, 0.0f);
    EXPECT_FLOAT_EQ(p.velocity.x, 0.0f);
    EXPECT_FLOAT_EQ(p.size, 0.1f);
    EXPECT_FLOAT_EQ(p.lifetime, 1.0f);
    EXPECT_FLOAT_EQ(p.age, 0.0f);
    EXPECT_TRUE(p.IsAlive());
}

TEST_F(ParticleSystemTest, ParticleIsAlive) {
    Particle p;
    p.lifetime = 2.0f;
    p.age = 1.0f;
    EXPECT_TRUE(p.IsAlive());

    p.age = 2.0f;
    EXPECT_FALSE(p.IsAlive());

    p.age = 2.5f;
    EXPECT_FALSE(p.IsAlive());
}

TEST_F(ParticleSystemTest, ParticleGetFade) {
    Particle p;
    p.lifetime = 2.0f;

    p.age = 0.0f;
    EXPECT_FLOAT_EQ(p.GetFade(), 1.0f);

    p.age = 1.0f;
    EXPECT_FLOAT_EQ(p.GetFade(), 0.5f);

    p.age = 2.0f;
    EXPECT_FLOAT_EQ(p.GetFade(), 0.0f);
}

// ===== ParticleSystem Basic Tests =====

TEST_F(ParticleSystemTest, InitialState) {
    EXPECT_EQ(particle_system->GetParticleCount(), 0);
    EXPECT_EQ(particle_system->GetParticles().size(), 0);
}

TEST_F(ParticleSystemTest, SpawnSingleParticle) {
    Vector3 pos(1, 2, 3);
    Vector3 vel(0.5f, 1.0f, 0.5f);
    Color color(1, 0, 0, 1);

    particle_system->Spawn(pos, vel, color, 0.2f, 1.5f, 1);

    EXPECT_EQ(particle_system->GetParticleCount(), 1);

    const auto& particles = particle_system->GetParticles();
    EXPECT_EQ(particles[0].position, pos);
    EXPECT_EQ(particles[0].velocity, vel);
    EXPECT_FLOAT_EQ(particles[0].size, 0.2f);
    EXPECT_FLOAT_EQ(particles[0].lifetime, 1.5f);
    EXPECT_EQ(particles[0].material_id, 1);
}

TEST_F(ParticleSystemTest, SpawnMultipleParticles) {
    for (int i = 0; i < 10; i++) {
        Vector3 pos(static_cast<float>(i), 0, 0);
        particle_system->Spawn(pos, Vector3::Zero(), Color(), 0.1f, 1.0f);
    }

    EXPECT_EQ(particle_system->GetParticleCount(), 10);
}

TEST_F(ParticleSystemTest, ClearParticles) {
    particle_system->Spawn(Vector3::Zero(), Vector3::Zero(), Color(), 0.1f, 1.0f);
    particle_system->Spawn(Vector3(1, 0, 0), Vector3::Zero(), Color(), 0.1f, 1.0f);

    EXPECT_EQ(particle_system->GetParticleCount(), 2);

    particle_system->Clear();
    EXPECT_EQ(particle_system->GetParticleCount(), 0);
}

TEST_F(ParticleSystemTest, MaxParticleLimit) {
    particle_system->SetMaxParticles(5);

    // Spawn 10 particles, but only 5 should be created
    for (int i = 0; i < 10; i++) {
        particle_system->Spawn(Vector3::Zero(), Vector3::Zero(), Color(), 0.1f, 1.0f);
    }

    EXPECT_EQ(particle_system->GetParticleCount(), 5);
}

// ===== Update & Physics Tests =====

TEST_F(ParticleSystemTest, UpdateAgesParticles) {
    particle_system->Spawn(Vector3::Zero(), Vector3::Zero(), Color(), 0.1f, 2.0f);

    particle_system->Update(0.5f);

    const auto& particles = particle_system->GetParticles();
    EXPECT_FLOAT_EQ(particles[0].age, 0.5f);

    particle_system->Update(0.5f);
    EXPECT_FLOAT_EQ(particles[0].age, 1.0f);
}

TEST_F(ParticleSystemTest, UpdateRemovesDeadParticles) {
    particle_system->Spawn(Vector3::Zero(), Vector3::Zero(), Color(), 0.1f, 1.0f);
    EXPECT_EQ(particle_system->GetParticleCount(), 1);

    // Age particle past lifetime
    particle_system->Update(0.5f);
    EXPECT_EQ(particle_system->GetParticleCount(), 1);

    particle_system->Update(0.6f); // Total: 1.1s > 1.0s lifetime
    EXPECT_EQ(particle_system->GetParticleCount(), 0);
}

TEST_F(ParticleSystemTest, UpdateAppliesGravity) {
    particle_system->SetGravity(Vector3(0, -10.0f, 0));
    particle_system->SetAirResistance(0.0f); // Disable air resistance for precise test
    particle_system->Spawn(Vector3::Zero(), Vector3::Zero(), Color(), 0.1f, 10.0f);

    particle_system->Update(1.0f);

    const auto& particles = particle_system->GetParticles();
    EXPECT_NEAR(particles[0].velocity.y, -10.0f, 0.01f);

    particle_system->Update(1.0f);
    EXPECT_NEAR(particles[0].velocity.y, -20.0f, 0.01f);
}

TEST_F(ParticleSystemTest, UpdateMovesParticles) {
    Vector3 initial_pos(0, 10, 0);
    Vector3 initial_vel(1, 0, 0);

    particle_system->SetGravity(Vector3::Zero()); // Disable gravity for this test
    particle_system->SetAirResistance(0.0f); // Disable air resistance for precise test
    particle_system->Spawn(initial_pos, initial_vel, Color(), 0.1f, 10.0f);

    particle_system->Update(1.0f);

    const auto& particles = particle_system->GetParticles();
    EXPECT_NEAR(particles[0].position.x, 1.0f, 0.01f);
    EXPECT_NEAR(particles[0].position.y, 10.0f, 0.01f);
    EXPECT_NEAR(particles[0].position.z, 0.0f, 0.01f);
}

TEST_F(ParticleSystemTest, UpdateAppliesAirResistance) {
    particle_system->SetGravity(Vector3::Zero());
    particle_system->SetAirResistance(0.1f);

    Vector3 initial_vel(10, 0, 0);
    particle_system->Spawn(Vector3::Zero(), initial_vel, Color(), 0.1f, 10.0f);

    particle_system->Update(1.0f);

    const auto& particles = particle_system->GetParticles();
    // Velocity should decrease due to air resistance
    EXPECT_LT(particles[0].velocity.x, 10.0f);
    EXPECT_GT(particles[0].velocity.x, 8.0f); // Should still be reasonable
}

TEST_F(ParticleSystemTest, UpdateFadesAlpha) {
    particle_system->Spawn(Vector3::Zero(), Vector3::Zero(), Color(1, 1, 1, 1), 0.1f, 2.0f);

    particle_system->Update(1.0f); // Age = 1.0s, lifetime = 2.0s, fade = 0.5

    const auto& particles = particle_system->GetParticles();
    EXPECT_NEAR(particles[0].color.a, 0.5f, 0.01f);
}

// ===== Material-Specific Impact Tests =====

TEST_F(ParticleSystemTest, SpawnConcreteImpact) {
    Vector3 pos(5, 0, 5);
    Vector3 normal(0, 1, 0);

    particle_system->SpawnConcreteImpact(pos, normal, 1.0f);

    // Should spawn ~30 particles (20 dust + 10 chips)
    EXPECT_GT(particle_system->GetParticleCount(), 20);
    EXPECT_LT(particle_system->GetParticleCount(), 35);
}

TEST_F(ParticleSystemTest, SpawnConcreteImpactIntensity) {
    Vector3 pos(5, 0, 5);
    Vector3 normal(0, 1, 0);

    particle_system->SpawnConcreteImpact(pos, normal, 0.5f);

    // Lower intensity = fewer particles
    int count_half = particle_system->GetParticleCount();

    particle_system->Clear();
    particle_system->SpawnConcreteImpact(pos, normal, 1.0f);

    int count_full = particle_system->GetParticleCount();

    EXPECT_LT(count_half, count_full);
}

TEST_F(ParticleSystemTest, SpawnWoodImpact) {
    Vector3 pos(5, 0, 5);
    Vector3 normal(0, 1, 0);

    particle_system->SpawnWoodImpact(pos, normal, 1.0f);

    // Should spawn ~23 particles (15 splinters + 8 dust)
    EXPECT_GT(particle_system->GetParticleCount(), 18);
    EXPECT_LT(particle_system->GetParticleCount(), 28);
}

TEST_F(ParticleSystemTest, SpawnBrickImpact) {
    Vector3 pos(5, 0, 5);
    Vector3 normal(0, 1, 0);

    particle_system->SpawnBrickImpact(pos, normal, 1.0f);

    // Should spawn ~30 particles (18 dust + 12 pieces)
    EXPECT_GT(particle_system->GetParticleCount(), 25);
    EXPECT_LT(particle_system->GetParticleCount(), 35);
}

TEST_F(ParticleSystemTest, SpawnDustCloud) {
    Vector3 epicenter(10, 5, 10);
    float radius = 5.0f;

    particle_system->SpawnDustCloud(epicenter, radius, 100);

    EXPECT_EQ(particle_system->GetParticleCount(), 100);

    // All particles should be within radius
    const auto& particles = particle_system->GetParticles();
    for (const auto& p : particles) {
        float dist = (p.position - epicenter).Length();
        EXPECT_LE(dist, radius * 1.6f); // Allow margin for diagonal spread
    }
}

TEST_F(ParticleSystemTest, DustCloudHasUpwardVelocity) {
    particle_system->SpawnDustCloud(Vector3::Zero(), 5.0f, 50);

    const auto& particles = particle_system->GetParticles();

    // Most particles should have upward velocity
    int upward_count = 0;
    for (const auto& p : particles) {
        if (p.velocity.y > 0) {
            upward_count++;
        }
    }

    // At least 80% should be moving upward
    EXPECT_GT(upward_count, 40);
}

// ===== Integration Test =====

TEST_F(ParticleSystemTest, FullSimulationCycle) {
    // Spawn impact particles
    particle_system->SpawnConcreteImpact(Vector3(0, 5, 0), Vector3(0, 1, 0), 1.0f);

    int initial_count = particle_system->GetParticleCount();
    EXPECT_GT(initial_count, 0);

    // Simulate for 1 second
    for (int i = 0; i < 60; i++) {
        particle_system->Update(1.0f / 60.0f);
    }

    // Some particles may have died, but most should still be alive
    EXPECT_GT(particle_system->GetParticleCount(), initial_count / 2);

    // Simulate until all particles are dead
    for (int i = 0; i < 300; i++) {
        particle_system->Update(1.0f / 60.0f);
    }

    // All particles should be dead after 5+ seconds
    EXPECT_EQ(particle_system->GetParticleCount(), 0);
}

TEST_F(ParticleSystemTest, ParticlesFallDueToGravity) {
    particle_system->SetGravity(Vector3(0, -9.81f, 0));
    particle_system->Spawn(Vector3(0, 10, 0), Vector3::Zero(), Color(), 0.1f, 10.0f);

    float initial_y = particle_system->GetParticles()[0].position.y;

    // Simulate for 1 second
    for (int i = 0; i < 60; i++) {
        particle_system->Update(1.0f / 60.0f);
    }

    float final_y = particle_system->GetParticles()[0].position.y;

    // Particle should have fallen significantly
    EXPECT_LT(final_y, initial_y);
    EXPECT_LT(final_y, 5.5f); // Should have fallen significantly (accounting for air resistance)
}
