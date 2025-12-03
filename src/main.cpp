#include <iostream>
#include <cmath>
#include <chrono>
#include "VoxelWorld.h"
#include "VoxelUtils.h"
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

void CreateTestBridge(VoxelWorld& world, int span_length, int pillar_height) {
    std::cout << "Creating test bridge (span: " << span_length
              << ", pillar height: " << pillar_height << ")...\n";

    float voxel_size = world.GetVoxelSize();

    // Left support pillar
    for (int y = 0; y < pillar_height; y++) {
        world.SetVoxel(Vector3(0, y * voxel_size, 0),
                      Voxel(MaterialDatabase::BRICK));
    }

    // Right support pillar
    for (int y = 0; y < pillar_height; y++) {
        world.SetVoxel(Vector3((span_length - 1) * voxel_size, y * voxel_size, 0),
                      Voxel(MaterialDatabase::BRICK));
    }

    // Bridge deck (concrete for strength)
    for (int x = 0; x < span_length; x++) {
        world.SetVoxel(Vector3(x * voxel_size, pillar_height * voxel_size, 0),
                      Voxel(MaterialDatabase::CONCRETE));
    }

    std::cout << "  Created " << world.GetVoxelCount() << " voxels\n";
}

void CreateTestArch(VoxelWorld& world, int width, int height) {
    std::cout << "Creating test arch (" << width << "x" << height << ")...\n";

    float voxel_size = world.GetVoxelSize();
    float radius = width / 2.0f;
    Vector3 center(radius * voxel_size, 0, 0);

    // Create arch using circular geometry
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            Vector3 pos(x * voxel_size, y * voxel_size, 0);
            float dx = pos.x - center.x;
            float dy = pos.y - center.y;
            float dist = std::sqrt(dx * dx + dy * dy);

            // Create arch ring (outer radius = width/2, inner radius = width/2 - 2 voxels)
            if (dist <= radius * voxel_size &&
                dist >= (radius - 2) * voxel_size &&
                y < height * 0.8f) {
                world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
            }
        }
    }

    std::cout << "  Created " << world.GetVoxelCount() << " voxels\n";
}

void CreateTestOverhang(VoxelWorld& world, int base_width, int overhang_length) {
    std::cout << "Creating test overhang (base: " << base_width
              << ", overhang: " << overhang_length << ")...\n";

    float voxel_size = world.GetVoxelSize();

    // Base pillar
    for (int y = 0; y < 10; y++) {
        for (int x = 0; x < base_width; x++) {
            world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, 0),
                          Voxel(MaterialDatabase::CONCRETE));
        }
    }

    // Cantilevered overhang (will stress structural analysis)
    int overhang_height = 10;
    for (int x = 0; x < base_width + overhang_length; x++) {
        world.SetVoxel(Vector3(x * voxel_size, overhang_height * voxel_size, 0),
                      Voxel(MaterialDatabase::CONCRETE));
    }

    std::cout << "  Created " << world.GetVoxelCount() << " voxels\n";
}

/**
 * Performance Benchmarking Utilities
 */
void RunPerformanceBenchmarks(VoxelWorld& world) {
    using namespace std::chrono;
    std::cout << "\n========================================\n";
    std::cout << "  Performance Benchmarks\n";
    std::cout << "========================================\n\n";

    size_t initial_voxel_count = world.GetVoxelCount();
    std::cout << "Testing with " << initial_voxel_count << " voxels\n\n";

    // Benchmark 1: Bulk voxel creation
    {
        VoxelWorld test_world(0.05f);
        auto start = high_resolution_clock::now();

        for (int i = 0; i < 1000; i++) {
            test_world.SetVoxel(Vector3(i * 0.05f, 0, 0), Voxel(MaterialDatabase::BRICK));
        }

        auto end = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(end - start);

        std::cout << "Bulk voxel insertion (1000 voxels):\n";
        std::cout << "  Time: " << duration.count() << " μs\n";
        std::cout << "  Rate: " << (1000.0 / duration.count() * 1e6) << " voxels/sec\n\n";
    }

    // Benchmark 2: Spatial queries (radius)
    {
        auto positions = world.GetAllVoxelPositions();
        if (!positions.empty()) {
            Vector3 center = positions[positions.size() / 2];
            auto start = high_resolution_clock::now();

            for (int i = 0; i < 100; i++) {
                auto results = world.GetVoxelsInRadius(center, 0.5f);
            }

            auto end = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(end - start);

            std::cout << "Radius queries (100 queries, r=0.5m):\n";
            std::cout << "  Time: " << duration.count() << " μs\n";
            std::cout << "  Avg: " << (duration.count() / 100.0) << " μs/query\n\n";
        }
    }

    // Benchmark 3: Neighbor finding
    {
        auto positions = world.GetAllVoxelPositions();
        if (!positions.empty()) {
            Vector3 test_pos = positions[0];
            auto start = high_resolution_clock::now();

            for (int i = 0; i < 1000; i++) {
                auto neighbors = world.GetNeighbors(test_pos, Connectivity::SIX);
            }

            auto end = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(end - start);

            std::cout << "Neighbor queries (1000 queries, 6-connected):\n";
            std::cout << "  Time: " << duration.count() << " μs\n";
            std::cout << "  Avg: " << (duration.count() / 1000.0) << " μs/query\n\n";
        }
    }

    // Benchmark 4: Surface detection
    {
        auto start = high_resolution_clock::now();
        auto surface_voxels = VoxelUtils::FindSurfaceVoxels(world);
        auto end = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(end - start);

        std::cout << "Surface detection (" << initial_voxel_count << " voxels):\n";
        std::cout << "  Time: " << duration.count() << " μs\n";
        std::cout << "  Found: " << surface_voxels.size() << " surface voxels\n";
        std::cout << "  Percentage: " << (100.0 * surface_voxels.size() / initial_voxel_count) << "%\n\n";
    }

    // Benchmark 5: Bounding box calculation
    {
        auto start = high_resolution_clock::now();

        for (int i = 0; i < 100; i++) {
            auto bounds = VoxelUtils::GetBounds(world);
        }

        auto end = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(end - start);

        std::cout << "Bounding box calculation (100 iterations):\n";
        std::cout << "  Time: " << duration.count() << " μs\n";
        std::cout << "  Avg: " << (duration.count() / 100.0) << " μs/calc\n\n";
    }

    // Benchmark 6: Center of mass calculation
    {
        auto start = high_resolution_clock::now();

        for (int i = 0; i < 100; i++) {
            auto com = VoxelUtils::GetCenterOfMass(world);
        }

        auto end = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(end - start);

        std::cout << "Center of mass calculation (100 iterations):\n";
        std::cout << "  Time: " << duration.count() << " μs\n";
        std::cout << "  Avg: " << (duration.count() / 100.0) << " μs/calc\n\n";
    }

    std::cout << "========================================\n";
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
        } else if (scene == "bridge") {
            CreateTestBridge(world, 20, 10);  // 20 voxel span, 10 voxel height
        } else if (scene == "arch") {
            CreateTestArch(world, 20, 15);
        } else if (scene == "overhang") {
            CreateTestOverhang(world, 5, 8);  // 5 voxel base, 8 voxel overhang
        } else {
            std::cout << "Unknown scene: " << scene << "\n";
            std::cout << "\nAvailable scenes:\n";
            std::cout << "  tower     - Vertical tower (tests vertical loads)\n";
            std::cout << "  wall      - 2D wall (tests basic stability)\n";
            std::cout << "  building  - Hollow building with 4 walls\n";
            std::cout << "  bridge    - Span with support pillars (tests horizontal loads)\n";
            std::cout << "  arch      - Architectural arch (tests compression)\n";
            std::cout << "  overhang  - Cantilevered structure (tests tension)\n";
            std::cout << "\nUsage: " << argv[0] << " [scene] [--benchmark]\n";
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

    // Run performance benchmarks if requested
    if (argc > 2 && std::string(argv[2]) == "--benchmark") {
        RunPerformanceBenchmarks(world);
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
