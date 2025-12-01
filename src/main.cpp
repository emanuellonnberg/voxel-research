#include <iostream>
#include "VoxelWorld.h"
#include "Material.h"
#include "Camera.h"
#include "Renderer.h"

/**
 * Main demo application for Voxel Destruction Engine
 *
 * This demonstrates:
 * - Creating a voxel world
 * - Building simple test structures
 * - Camera navigation
 * - Voxel rendering (when OpenGL is available)
 *
 * Controls (when OpenGL available):
 * - WASD: Move camera
 * - Mouse: Look around
 * - Space/Shift: Move up/down
 * - ESC: Exit
 */

void CreateTestTower(VoxelWorld& world, int height) {
    std::cout << "Creating test tower (height: " << height << ")...\n";

    float voxel_size = world.GetVoxelSize();
    for (int y = 0; y < height; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0),
                      Voxel(MaterialDatabase::BRICK));
    }

    std::cout << "  Created " << world.GetVoxelCount() << " voxels\n";
}

void CreateTestWall(VoxelWorld& world, int width, int height) {
    std::cout << "Creating test wall (" << width << "x" << height << ")...\n";

    float voxel_size = world.GetVoxelSize();
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, 0),
                          Voxel(MaterialDatabase::BRICK));
        }
    }

    std::cout << "  Created " << world.GetVoxelCount() << " voxels\n";
}

void CreateTestBuilding(VoxelWorld& world, int width, int depth, int height) {
    std::cout << "Creating test building (" << width << "x" << depth << "x" << height << ")...\n";

    float voxel_size = world.GetVoxelSize();

    // Create hollow building with walls
    for (int y = 0; y < height; y++) {
        // Four walls
        for (int x = 0; x < width; x++) {
            // Front and back walls
            world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, 0),
                          Voxel(MaterialDatabase::BRICK));
            world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, (depth-1) * voxel_size),
                          Voxel(MaterialDatabase::BRICK));
        }

        for (int z = 0; z < depth; z++) {
            // Left and right walls
            world.SetVoxel(Vector3(0, y * voxel_size, z * voxel_size),
                          Voxel(MaterialDatabase::BRICK));
            world.SetVoxel(Vector3((width-1) * voxel_size, y * voxel_size, z * voxel_size),
                          Voxel(MaterialDatabase::BRICK));
        }
    }

    std::cout << "  Created " << world.GetVoxelCount() << " voxels\n";
}

int main(int argc, char** argv) {
    std::cout << "========================================\n";
    std::cout << "  Voxel Destruction Engine - Demo\n";
    std::cout << "========================================\n\n";

    // Print material database
    std::cout << "Material Database:\n";
    for (size_t i = 0; i < g_materials.GetMaterialCount(); i++) {
        const Material& mat = g_materials.GetMaterial(static_cast<uint8_t>(i));
        if (mat.density > 0) {
            std::cout << "  " << mat.name << ": "
                     << mat.density << " kg/m³, "
                     << (mat.compressive_strength / 1e6) << " MPa\n";
        }
    }
    std::cout << "\n";

    // Create voxel world
    VoxelWorld world(0.05f);  // 5cm voxels
    std::cout << "Created voxel world (voxel size: " << world.GetVoxelSize() << "m)\n\n";

    // Create test structures
    if (argc > 1) {
        std::string scene = argv[1];
        if (scene == "tower") {
            CreateTestTower(world, 20);
        } else if (scene == "wall") {
            CreateTestWall(world, 10, 10);
        } else if (scene == "building") {
            CreateTestBuilding(world, 10, 10, 5);
        } else {
            std::cout << "Unknown scene: " << scene << "\n";
            std::cout << "Available scenes: tower, wall, building\n";
            return 1;
        }
    } else {
        // Default: Create a small building
        CreateTestBuilding(world, 5, 5, 3);
    }

    // Test spatial queries
    std::cout << "\nSpatial Queries:\n";

    Vector3 query_center(0.1f, 0.1f, 0.1f);
    auto nearby = world.GetVoxelsInRadius(query_center, 0.2f);
    std::cout << "  Voxels within 0.2m of " << query_center << ": "
              << nearby.size() << "\n";

    auto all_positions = world.GetAllVoxelPositions();
    if (!all_positions.empty()) {
        Vector3 test_pos = all_positions[0];
        int neighbor_count = world.CountNeighbors(test_pos, Connectivity::SIX);
        std::cout << "  Neighbors of " << test_pos << ": "
                  << neighbor_count << " (6-connected)\n";
    }

    // Initialize rendering (stub in environments without OpenGL)
    std::cout << "\nInitializing rendering...\n";

    Window window(1280, 720, "Voxel Destruction Demo");
    if (!window.Initialize()) {
        std::cerr << "Failed to initialize window\n";
        std::cout << "Note: OpenGL not available in this environment.\n";
        std::cout << "Rendering features disabled. Core systems functional.\n";
        return 0;
    }

    VoxelRenderer renderer;
    if (!renderer.Initialize()) {
        std::cerr << "Failed to initialize renderer\n";
        return 1;
    }

    // Create camera
    Camera camera(Vector3(2, 2, 5));
    camera.SetSpeed(2.0f);

    std::cout << "\nCamera initialized at " << camera.GetPosition() << "\n";
    std::cout << "  Forward: " << camera.GetForward() << "\n";
    std::cout << "  FOV: " << camera.GetFov() << "°\n";

    // Update renderer with voxels
    renderer.UpdateVoxels(world);

    std::cout << "\nStarting render loop...\n";
    std::cout << "Controls:\n";
    std::cout << "  WASD: Move camera\n";
    std::cout << "  Mouse: Look around\n";
    std::cout << "  Space/Shift: Up/Down\n";
    std::cout << "  ESC: Exit\n\n";

    // Main loop
    while (!window.ShouldClose()) {
        window.UpdateDeltaTime();
        float dt = window.GetDeltaTime();

        // Handle input (stub in non-OpenGL environment)
        float forward = 0, right = 0, up = 0;
        // In real implementation: check GLFW keys
        // if (window.IsKeyPressed(GLFW_KEY_W)) forward = 1.0f;
        // etc.

        camera.ProcessKeyboard(forward, right, up, dt);

        // Render
        renderer.Render(camera, window.GetAspectRatio());

        window.SwapBuffers();
        window.PollEvents();

        // In stub mode, exit after one frame
        break;
    }

    std::cout << "Rendered " << renderer.GetRenderedVoxelCount() << " voxels\n";

    // Cleanup
    renderer.Shutdown();
    window.Shutdown();

    std::cout << "\nDemo complete.\n";
    std::cout << "========================================\n";

    return 0;
}
