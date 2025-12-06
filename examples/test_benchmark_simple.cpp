/**
 * test_benchmark_simple.cpp
 *
 * Minimal benchmark test to isolate segfault
 */

#include "JobSystem.h"
#include "VoxelWorld.h"
#include "Material.h"
#include <iostream>

int main() {
    std::cout << "Step 1: Creating JobSystem...\n";
    g_JobSystem = new JobSystem();
    g_JobSystem->Initialize(4);
    std::cout << "  Workers: " << g_JobSystem->GetWorkerCount() << "\n";

    std::cout << "\nStep 2: Creating VoxelWorld...\n";
    VoxelWorld* world = new VoxelWorld();
    std::cout << "  Voxel size: " << world->GetVoxelSize() << "\n";

    std::cout << "\nStep 3: Adding voxels...\n";
    float voxel_size = world->GetVoxelSize();
    for (int x = 0; x < 5; x++) {
        for (int y = 0; y < 5; y++) {
            for (int z = 0; z < 5; z++) {
                Vector3 pos(x * voxel_size, y * voxel_size, z * voxel_size);
                world->SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
            }
        }
    }
    std::cout << "  Voxel count: " << world->GetVoxelCount() << "\n";

    std::cout << "\nStep 4: Removing some voxels...\n";
    for (int x = 0; x < 5; x++) {
        for (int z = 0; z < 5; z++) {
            Vector3 pos(x * voxel_size, 0, z * voxel_size);
            world->RemoveVoxel(pos);
        }
    }
    std::cout << "  Voxel count: " << world->GetVoxelCount() << "\n";

    std::cout << "\nStep 5: Cleanup...\n";
    delete world;
    g_JobSystem->Shutdown();
    delete g_JobSystem;
    g_JobSystem = nullptr;

    std::cout << "\nTest complete!\n";
    return 0;
}
