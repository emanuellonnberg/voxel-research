/**
 * test_debris_collision.cpp
 *
 * Test debris-voxel collision system
 * Verifies that debris actually lands on voxel structures instead of falling through
 */

#include "VoxelWorld.h"
#include "Material.h"
#include "BulletEngine.h"
#include "VoxelPhysicsIntegration.h"
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "\n╔══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  Debris-Voxel Collision System Test                     ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════╝\n\n";

    // Create voxel world
    VoxelWorld world;
    std::cout << "Creating voxel tower (10 voxels tall)...\n";

    // Build a simple tower
    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < 10; y++) {
        Vector3 pos(0, y * voxel_size, 0);
        world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
    }

    std::cout << "  Tower created: " << world.GetVoxelCount() << " voxels\n";
    std::cout << "  Tower height: " << (10 * voxel_size) << "m\n\n";

    // Initialize physics
    std::cout << "Initializing Bullet physics...\n";
    BulletEngine physics;
    physics.Initialize();

    // CRITICAL: Set voxel world and build collision
    std::cout << "Setting up debris-voxel collision...\n";
    physics.SetVoxelWorld(&world);
    physics.BuildVoxelCollision();

    std::cout << "\n";

    // Create falling debris above the tower
    std::cout << "Creating falling debris 1m above tower...\n";

    float drop_height = (10 * voxel_size) + 1.0f;  // 1m above tower top
    Vector3 debris_start_pos(0, drop_height, 0);

    VoxelCluster debris_cluster;
    debris_cluster.voxel_positions = {debris_start_pos};
    debris_cluster.center_of_mass = debris_start_pos;
    debris_cluster.total_mass = 10.0f;  // 10kg
    debris_cluster.bounds = BoundingBox{
        debris_start_pos,
        debris_start_pos + Vector3(voxel_size, voxel_size, voxel_size)
    };

    VoxelPhysicsIntegration integration(&physics, &world);
    integration.SpawnDebris({debris_cluster});

    std::cout << "  Debris spawned at y = " << debris_start_pos.y << "m\n";
    std::cout << "  Expected landing y = " << (10 * voxel_size) << "m (top of tower)\n\n";

    // Simulate physics
    std::cout << "Simulating physics (3 seconds at 60 FPS)...\n\n";

    const int TOTAL_FRAMES = 180;  // 3 seconds @ 60 FPS
    const float TIMESTEP = 1.0f / 60.0f;

    for (int frame = 0; frame < TOTAL_FRAMES; frame++) {
        // Update voxel collision (rebuilds dirty chunks)
        physics.UpdateVoxelCollision(TIMESTEP);

        // Step physics
        physics.Step(TIMESTEP);
        integration.Step(TIMESTEP);

        // Print status every 30 frames (0.5 seconds)
        if (frame % 30 == 0) {
            if (integration.GetDebrisCount() > 0) {
                std::vector<Transform> transforms;
                std::vector<uint8_t> material_ids;
                integration.GetDebrisRenderData(transforms, material_ids);

                if (!transforms.empty()) {
                    Vector3 pos = transforms[0].position;

                    std::cout << std::fixed << std::setprecision(3);
                    std::cout << "  Frame " << std::setw(3) << frame << ": "
                              << "y = " << std::setw(6) << pos.y << "m\n";
                }
            }
        }
    }

    // Check final position
    std::cout << "\n";
    std::cout << "══════════════════════════════════════════════════════════\n";
    std::cout << "  TEST RESULTS\n";
    std::cout << "══════════════════════════════════════════════════════════\n\n";

    if (integration.GetDebrisCount() == 0) {
        std::cout << "❌ FAIL: Debris disappeared!\n";
        physics.Shutdown();
        return 1;
    }

    std::vector<Transform> transforms;
    std::vector<uint8_t> material_ids;
    integration.GetDebrisRenderData(transforms, material_ids);

    if (transforms.empty()) {
        std::cout << "❌ FAIL: Could not get debris position!\n";
        physics.Shutdown();
        return 1;
    }

    Vector3 final_pos = transforms[0].position;
    float tower_top_y = 10 * voxel_size;

    std::cout << "Debris final position: y = " << final_pos.y << "m\n";
    std::cout << "Tower top position:    y = " << tower_top_y << "m\n";
    std::cout << "Ground plane:          y = 0.0m\n\n";

    // Test criteria:
    // - Debris should land on tower top (y ≈ 0.5m)
    // - NOT fall through to ground (y ≈ 0.0m)
    float tolerance = 0.2f;  // 20cm tolerance
    bool landed_on_tower = (final_pos.y > (tower_top_y - tolerance) &&
                            final_pos.y < (tower_top_y + tolerance + voxel_size));

    if (landed_on_tower) {
        std::cout << "✓ PASS: Debris landed on tower!\n";
        std::cout << "  The debris-voxel collision system is working correctly.\n";
        std::cout << "  Debris interacted with voxel structure as expected.\n\n";

        physics.Shutdown();
        return 0;

    } else if (final_pos.y < 0.1f) {
        std::cout << "❌ FAIL: Debris fell through tower to ground!\n";
        std::cout << "  Expected: y ≈ " << tower_top_y << "m (landing on tower)\n";
        std::cout << "  Got:      y = " << final_pos.y << "m (fell to ground)\n\n";
        std::cout << "  ⚠️  This means voxel collision meshes are NOT working.\n";
        std::cout << "  Check that BuildVoxelCollision() was called.\n\n";

        physics.Shutdown();
        return 1;

    } else {
        std::cout << "⚠️  UNCERTAIN: Debris position unexpected\n";
        std::cout << "  Expected: y ≈ " << tower_top_y << "m\n";
        std::cout << "  Got:      y = " << final_pos.y << "m\n\n";

        physics.Shutdown();
        return 1;
    }
}
