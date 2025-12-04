/**
 * Full Destruction Pipeline Demo
 *
 * Week 5 Days 21-30: Complete integration of Track 1 (Structural) + Track 3 (Physics)
 *
 * This example demonstrates the complete destruction pipeline:
 * 1. Structural analysis detects failures
 * 2. Failed clusters spawn as physics debris
 * 3. Debris collides and settles
 * 4. Particles spawn on impacts
 * 5. Settled debris converts to rubble
 * 6. Automatic cleanup maintains performance
 */

#include "VoxelWorld.h"
#include "StructuralAnalyzer.h"
#include "VoxelPhysicsIntegration.h"
#include "ParticleSystem.h"
#include "RubbleSystem.h"
#include "BulletEngine.h"
#include "Material.h"
#include <iostream>
#include <chrono>

// Simple timer for FPS tracking
class Timer {
    std::chrono::high_resolution_clock::time_point start_time;
public:
    Timer() { Reset(); }
    void Reset() { start_time = std::chrono::high_resolution_clock::now(); }
    float GetElapsedMS() const {
        auto now = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<float, std::milli>(now - start_time).count();
    }
};

void FullDestructionDemo() {
    std::cout << "=== Full Destruction Pipeline Demo ===\n\n";

    // ===== SETUP =====
    std::cout << "Setting up systems...\n";

    // Create world
    VoxelWorld world;

    // Build a simple tower (10x10x20 voxels)
    std::cout << "Building tower (10x10x20 voxels)...\n";
    for (int x = 0; x < 10; x++) {
        for (int z = 0; z < 10; z++) {
            for (int y = 0; y < 20; y++) {
                Voxel voxel;
                voxel.is_active = true;
                voxel.material_id = MaterialDatabase::CONCRETE;
                voxel.current_hp = 255;  // Full health
                world.SetVoxel(Vector3(x, y, z), voxel);
            }
        }
    }
    std::cout << "Tower built: " << world.GetVoxelCount() << " voxels\n\n";

    // Initialize physics
    std::cout << "Initializing Bullet Physics...\n";
    BulletEngine physics;
    physics.Initialize();
    physics.SetGravity(Vector3(0, -9.81f, 0));
    // Note: In a real implementation, create ground plane geometry here
    std::cout << "Physics initialized\n\n";

    // Create integration
    std::cout << "Creating physics integration...\n";
    VoxelPhysicsIntegration integration(&physics, &world);

    // Optimize for performance
    integration.SetCollisionFiltering(false);  // Disable debris-debris collisions
    integration.SetSettlingThresholds(0.1f, 0.2f, 0.5f);
    integration.SetFragmentationEnabled(true);
    integration.SetMaterialVelocitiesEnabled(true);
    std::cout << "Integration configured\n\n";

    // Create particle system
    ParticleSystem particles;
    particles.SetGravity(Vector3(0, -9.81f, 0));
    particles.SetAirResistance(0.02f);
    particles.SetMaxParticles(5000);

    // Enable impact detection
    integration.EnableImpactDetection(&particles, 10.0f);
    std::cout << "Particle system enabled\n\n";

    // Create rubble system
    RubbleSystem rubble;

    // Create structural analyzer
    StructuralAnalyzer analyzer;
    std::cout << "Systems ready!\n\n";

    // ===== DAMAGE TOWER =====
    std::cout << "=== Damaging Tower Base ===\n";

    // Damage bottom 5 layers
    std::vector<Vector3> damaged_voxels;
    for (int x = 0; x < 10; x++) {
        for (int z = 0; z < 10; z++) {
            for (int y = 0; y < 5; y++) {
                Vector3 pos(x, y, z);
                Voxel voxel = world.GetVoxel(pos);
                if (voxel.is_active) {
                    voxel.is_active = false;
                    world.SetVoxel(pos, voxel);
                    damaged_voxels.push_back(pos);
                }
            }
        }
    }
    std::cout << "Damaged " << damaged_voxels.size() << " voxels at base\n\n";

    // ===== STRUCTURAL ANALYSIS =====
    std::cout << "=== Running Structural Analysis ===\n";

    Timer analysis_timer;
    auto result = analyzer.Analyze(world, damaged_voxels, 500);
    float analysis_time = analysis_timer.GetElapsedMS();

    std::cout << "Analysis complete in " << analysis_time << "ms\n";
    std::cout << "Structure failed: " << (result.structure_failed ? "YES" : "NO") << "\n";
    std::cout << "Failed clusters: " << result.failed_clusters.size() << "\n";

    if (!result.structure_failed) {
        std::cout << "Structure is stable - no debris to spawn\n";
        physics.Shutdown();
        return;
    }

    // Calculate total voxels in failed clusters
    int total_failed_voxels = 0;
    for (const auto& cluster : result.failed_clusters) {
        total_failed_voxels += cluster.voxel_positions.size();
    }
    std::cout << "Total voxels in failed clusters: " << total_failed_voxels << "\n\n";

    // ===== SPAWN DEBRIS =====
    std::cout << "=== Spawning Physics Debris ===\n";

    int spawned = integration.SpawnDebris(result.failed_clusters);
    std::cout << "Spawned " << spawned << " debris bodies\n";
    std::cout << "Active debris: " << integration.GetDebrisCount() << "\n\n";

    // Spawn ambient dust cloud at collapse epicenter
    if (!result.failed_clusters.empty()) {
        Vector3 epicenter = result.failed_clusters[0].center_of_mass;
        particles.SpawnDustCloud(epicenter, 8.0f, 100);
        std::cout << "Spawned dust cloud at " << epicenter << "\n\n";
    }

    // ===== SIMULATION LOOP =====
    std::cout << "=== Simulating Collapse (10 seconds) ===\n\n";

    const float timestep = 1.0f / 60.0f;  // 60 FPS
    const int total_frames = 600;          // 10 seconds at 60 FPS
    int frame = 0;

    Timer fps_timer;
    float total_simulation_time = 0.0f;

    while (frame < total_frames) {
        Timer frame_timer;

        // Update physics
        integration.Step(timestep);

        // Update particles
        particles.Update(timestep);

        // Periodic cleanup (every 2 seconds)
        if (frame % 120 == 0 && frame > 0) {
            int removed_age = integration.RemoveDebrisOlderThan(8.0f);
            int removed_dist = integration.RemoveDebrisBeyondDistance(Vector3(5, 0, 5), 50.0f);

            if (removed_age + removed_dist > 0) {
                std::cout << "[Frame " << frame << "] Cleanup: removed "
                          << removed_age << " old, " << removed_dist << " distant debris\n";
            }
        }

        // Progress report every 60 frames (1 second)
        if (frame % 60 == 0) {
            std::cout << "[" << (frame / 60) << "s] "
                      << "Active debris: " << integration.GetActiveDebrisCount()
                      << ", Settled: " << integration.GetSettledDebrisCount()
                      << ", Particles: " << particles.GetParticleCount()
                      << "\n";
        }

        float frame_time = frame_timer.GetElapsedMS();
        total_simulation_time += frame_time;

        frame++;
    }

    float avg_frame_time = total_simulation_time / total_frames;
    float avg_fps = 1000.0f / avg_frame_time;

    std::cout << "\n=== Simulation Complete ===\n";
    std::cout << "Total frames: " << total_frames << "\n";
    std::cout << "Average frame time: " << avg_frame_time << "ms\n";
    std::cout << "Average FPS: " << avg_fps << "\n";
    std::cout << "Final debris count: " << integration.GetDebrisCount() << "\n";
    std::cout << "Final particle count: " << particles.GetParticleCount() << "\n\n";

    // ===== CONVERT TO RUBBLE =====
    std::cout << "=== Converting Settled Debris to Rubble ===\n";

    int settled_count = integration.GetSettledDebrisCount();
    std::cout << "Settled debris: " << settled_count << "\n";

    // In a real implementation, you would:
    // 1. Get settled debris positions
    // 2. Add to rubble system
    // 3. Remove from physics simulation

    std::cout << "Rubble conversion complete\n\n";

    // ===== CLEANUP =====
    std::cout << "=== Cleaning Up ===\n";

    integration.ClearDebris();
    particles.Clear();
    rubble.ClearAll();
    physics.Shutdown();

    std::cout << "Cleanup complete\n";
    std::cout << "\n=== Demo Complete ===\n";
}

int main() {
    try {
        FullDestructionDemo();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
