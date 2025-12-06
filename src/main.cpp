#include <iostream>
#include <cmath>
#include <chrono>
#include <memory>
#include "VoxelWorld.h"
#include "VoxelUtils.h"
#include "Material.h"
#include "Camera.h"
#include "Renderer.h"
#include "VoxelCluster.h"
#include "VoxelClusterTracker.h"
#include "StructuralAnalyzer.h"
#ifdef USE_BULLET
#include "VoxelPhysicsIntegration.h"
#include "BulletEngine.h"
#endif

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
            uint8_t material_id = MaterialDatabase::BRICK;

            if (y < 2) {
                material_id = MaterialDatabase::CONCRETE;  // Reinforced foundation strip
            } else if ((x + y) % 5 == 0) {
                material_id = MaterialDatabase::WOOD;  // Accent beams
            }

            world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, 0),
                           Voxel(material_id));
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
            uint8_t wall_material = MaterialDatabase::BRICK;
            if (y == 0) {
                wall_material = MaterialDatabase::CONCRETE; // base ring
            } else if (y == height - 1) {
                wall_material = MaterialDatabase::WOOD; // top trim
            }

            // Front and back walls
            world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, 0),
                           Voxel(wall_material));
            world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, (depth-1) * voxel_size),
                           Voxel(wall_material));
        }

        for (int z = 0; z < depth; z++) {
            uint8_t wall_material = MaterialDatabase::BRICK;
            if (y == 0) {
                wall_material = MaterialDatabase::CONCRETE;
            } else if (y == height - 1) {
                wall_material = MaterialDatabase::WOOD;
            }

            // Left and right walls
            world.SetVoxel(Vector3(0, y * voxel_size, z * voxel_size),
                           Voxel(wall_material));
            world.SetVoxel(Vector3((width-1) * voxel_size, y * voxel_size, z * voxel_size),
                           Voxel(wall_material));
        }
    }

    // Simple roof
    for (int x = 0; x < width; x++) {
        for (int z = 0; z < depth; z++) {
            world.SetVoxel(Vector3(x * voxel_size, height * voxel_size, z * voxel_size),
                           Voxel(MaterialDatabase::WOOD));
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

#ifdef RENDERING_ENABLED
namespace {
const float kEditRange = 6.0f;

struct RemovedVoxelInfo {
    Vector3 position;
    Voxel voxel;
};

bool RemoveVoxelUnderCrosshair(VoxelWorld& world, const Camera& camera, RemovedVoxelInfo* out_info) {
    auto hits = world.Raycast(camera.GetPosition(), camera.GetForward(), kEditRange);
    if (hits.empty()) {
        return false;
    }

    Vector3 snapped = world.SnapToGrid(hits.front());
    if (!world.HasVoxel(snapped)) {
        return false;
    }

    Voxel voxel = world.GetVoxel(snapped);
    world.RemoveVoxel(snapped);

    if (out_info) {
        out_info->position = snapped;
        out_info->voxel = voxel;
    }

    return true;
}

bool AddVoxelUnderCrosshair(VoxelWorld& world,
                            const Camera& camera,
                            uint8_t material_id,
                            Vector3* out_position = nullptr) {
    Vector3 forward = camera.GetForward().Normalized();
    auto hits = world.Raycast(camera.GetPosition(), camera.GetForward(), kEditRange);

    Vector3 target;
    if (!hits.empty()) {
        target = hits.front() - forward * world.GetVoxelSize();
    } else {
        target = camera.GetPosition() + forward * world.GetVoxelSize() * 3.0f;
    }

    target = world.SnapToGrid(target);
    if (world.HasVoxel(target)) {
        return false;
    }

    world.SetVoxel(target, Voxel(material_id));
    if (out_position) {
        *out_position = target;
    }
    return true;
}

#ifdef USE_BULLET
void SpawnSingleVoxelDebris(VoxelPhysicsIntegration& integration,
                            const RemovedVoxelInfo& removed,
                            float voxel_size,
                            uint32_t& next_cluster_id) {
    if (!removed.voxel.is_active) {
        return;
    }

    VoxelCluster cluster;
    cluster.cluster_id = next_cluster_id++;
    cluster.voxel_positions.push_back(removed.position);
    cluster.center_of_mass = removed.position;
    cluster.total_mass = g_materials.GetMaterial(removed.voxel.material_id).GetVoxelMass(voxel_size);
    cluster.dominant_material = removed.voxel.material_id;

    Vector3 half(voxel_size * 0.5f, voxel_size * 0.5f, voxel_size * 0.5f);
    cluster.bounds = BoundingBox(removed.position - half, removed.position + half);

    integration.SpawnDebris({cluster});
}
#endif
} // namespace
#endif

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

    VoxelClusterTracker cluster_tracker(&world);
    StructuralAnalyzer analyzer;
    std::vector<Vector3> pending_analysis_positions;
    pending_analysis_positions.reserve(32);
    bool analysis_pending = false;
    std::string pending_analysis_reason;

#ifdef USE_BULLET
    std::unique_ptr<BulletEngine> bullet_engine = std::make_unique<BulletEngine>();
    std::unique_ptr<VoxelPhysicsIntegration> physics_integration;
    uint32_t next_cluster_id = 1;

    if (bullet_engine->Initialize()) {
        bullet_engine->SetGravity(Vector3(0.0f, -9.81f, 0.0f));
        physics_integration = std::make_unique<VoxelPhysicsIntegration>(bullet_engine.get(), &world);
        std::cout << "[Physics] Bullet engine initialized.\n";
    } else {
        std::cerr << "[Physics] Failed to initialize Bullet engine. Physics disabled for this run.\n";
        bullet_engine.reset();
    }
#endif

#if defined(USE_BULLET) && defined(RENDERING_ENABLED)
    std::vector<Transform> debris_transforms;
    std::vector<uint8_t> debris_material_ids;
    std::vector<Vector3> debris_positions;
    std::vector<Color> debris_colors;
#endif

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
#ifdef RENDERING_ENABLED
    renderer.ClearDebrisInstances();
#endif

    std::cout << "\nStarting render loop...\n";
    std::cout << "Controls:\n";
    std::cout << "  WASD: Move camera\n";
    std::cout << "  Mouse: Look around\n";
    std::cout << "  Space/Shift: Up/Down\n";
    std::cout << "  ESC: Exit\n\n";
#ifdef RENDERING_ENABLED
    std::cout << "  Left Click: Remove voxel\n";
    std::cout << "  Right Click: Add voxel (brick)\n\n";
    std::cout << "  F1: Toggle wireframe\n";
    std::cout << "  F2: Force structural analysis at crosshair\n\n";
#endif

#ifdef RENDERING_ENABLED
    auto request_analysis = [&](const std::vector<Vector3>& positions, const std::string& reason) {
        if (positions.empty()) {
            return;
        }
        pending_analysis_positions.insert(pending_analysis_positions.end(), positions.begin(), positions.end());
        pending_analysis_reason = reason;
        analysis_pending = true;
        cluster_tracker.MarkDirty();
    };

    auto request_analysis_single = [&](const Vector3& position, const std::string& reason) {
        request_analysis({position}, reason);
    };

    auto run_analysis_if_needed = [&]() {
        if (!analysis_pending || pending_analysis_positions.empty()) {
            return;
        }

        std::vector<Vector3> damaged_positions = pending_analysis_positions;
        pending_analysis_positions.clear();

        std::string reason = pending_analysis_reason.empty()
                                 ? std::string("voxel edit")
                                 : pending_analysis_reason;

        AnalysisResult result = analyzer.Analyze(world, damaged_positions);
        analysis_pending = false;
        pending_analysis_reason.clear();

        size_t cluster_count_before = cluster_tracker.GetClusters().size();

        if (result.structure_failed && !result.failed_clusters.empty()) {
            std::cout << "[Analyzer] " << reason << ": instability detected - "
                      << result.failed_clusters.size() << " failing clusters, "
                      << result.num_failed_nodes << " failed nodes ("
                      << result.calculation_time_ms << "ms, "
                      << cluster_count_before << " clusters)\n";

            std::vector<VoxelCluster> spawn_clusters = result.failed_clusters;
            for (auto& cluster : spawn_clusters) {
                if (cluster.cluster_id == 0) {
                    cluster.cluster_id = next_cluster_id++;
                }

                for (const auto& pos : cluster.voxel_positions) {
                    world.RemoveVoxel(pos);
                }
            }

            renderer.UpdateVoxels(world);
            cluster_tracker.MarkDirty();

#ifdef USE_BULLET
            if (physics_integration) {
                physics_integration->SpawnDebris(spawn_clusters);
            }
#endif

            if (!spawn_clusters.empty() && !spawn_clusters[0].voxel_positions.empty()) {
                request_analysis_single(spawn_clusters[0].voxel_positions[0], "post-collapse follow-up");
            }
        } else {
            std::cout << "[Analyzer] " << reason << ": structure stable ("
                      << result.nodes_analyzed << " nodes, "
                      << cluster_count_before << " connected clusters, "
                      << result.calculation_time_ms << "ms)\n";
        }
    };

    window.SetCursorMode(true);
    bool mouse_initialized = false;
    double last_mouse_x = 0.0;
    double last_mouse_y = 0.0;
    bool left_mouse_down = false;
    bool right_mouse_down = false;
    bool wireframe_enabled = false;
    bool wireframe_key_down = false;
    bool analyze_key_down = false;

    auto queue_initial_analysis = [&]() {
        std::vector<Vector3> seed_positions;
        const auto& surface_voxels = world.GetSurfaceVoxels();
        for (const auto& pos : surface_voxels) {
            seed_positions.push_back(pos);
            if (seed_positions.size() >= 64) {
                break;
            }
        }
        if (seed_positions.empty()) {
            auto all = world.GetAllVoxelPositions();
            for (const auto& pos : all) {
                seed_positions.push_back(pos);
                if (seed_positions.size() >= 32) {
                    break;
                }
            }
        }
        request_analysis(seed_positions, "initial stability check");
    };

    queue_initial_analysis();
#endif

    // Main loop
    while (!window.ShouldClose()) {
        window.PollEvents();
        window.UpdateDeltaTime();
        float dt = window.GetDeltaTime();

#ifdef USE_BULLET
        if (physics_integration) {
            physics_integration->Step(dt);
            physics_integration->RemoveDebrisBeyondDistance(camera.GetPosition(), 150.0f);
        }
#endif

#ifdef RENDERING_ENABLED
#ifdef USE_BULLET
        if (physics_integration) {
            debris_transforms.clear();
            debris_material_ids.clear();
            physics_integration->GetDebrisRenderData(debris_transforms, debris_material_ids);

            debris_positions.clear();
            debris_positions.reserve(debris_transforms.size());
            debris_colors.clear();
            debris_colors.reserve(debris_transforms.size());

            for (size_t i = 0; i < debris_transforms.size(); ++i) {
                debris_positions.push_back(debris_transforms[i].position);
                uint8_t mat_id = (i < debris_material_ids.size()) ? debris_material_ids[i]
                                                                  : MaterialDatabase::BRICK;
                debris_colors.push_back(g_materials.GetMaterial(mat_id).color);
            }

            renderer.UpdateDebrisInstances(debris_positions, debris_colors, world.GetVoxelSize());
        } else {
            renderer.ClearDebrisInstances();
        }
#else
        renderer.ClearDebrisInstances();
#endif
#endif

        float forward = 0.0f;
        float right = 0.0f;
        float up = 0.0f;

#ifdef RENDERING_ENABLED
        if (window.IsKeyPressed(GLFW_KEY_W)) forward += 1.0f;
        if (window.IsKeyPressed(GLFW_KEY_S)) forward -= 1.0f;
        if (window.IsKeyPressed(GLFW_KEY_D)) right += 1.0f;
        if (window.IsKeyPressed(GLFW_KEY_A)) right -= 1.0f;
        if (window.IsKeyPressed(GLFW_KEY_SPACE)) up += 1.0f;
        if (window.IsKeyPressed(GLFW_KEY_LEFT_SHIFT)) up -= 1.0f;
        if (window.IsKeyPressed(GLFW_KEY_ESCAPE)) {
            window.Close();
        }

        double mouse_x = 0.0;
        double mouse_y = 0.0;
        window.GetCursorPos(&mouse_x, &mouse_y);

        if (!mouse_initialized) {
            last_mouse_x = mouse_x;
            last_mouse_y = mouse_y;
            mouse_initialized = true;
        }

        double delta_x = mouse_x - last_mouse_x;
        double delta_y = last_mouse_y - mouse_y;  // invert Y for intuitive look
        last_mouse_x = mouse_x;
        last_mouse_y = mouse_y;

        if (window.IsCursorCaptured()) {
            camera.ProcessMouseMovement(static_cast<float>(delta_x),
                                        static_cast<float>(delta_y));
        }

        bool left_pressed = window.IsMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT);
        if (left_pressed && !left_mouse_down) {
            RemovedVoxelInfo removed_info;
            if (RemoveVoxelUnderCrosshair(world, camera, &removed_info)) {
                renderer.UpdateVoxels(world);
                request_analysis_single(removed_info.position, "voxel removed");
#ifdef USE_BULLET
                if (physics_integration) {
                    SpawnSingleVoxelDebris(*physics_integration, removed_info, world.GetVoxelSize(), next_cluster_id);
                }
#endif
                std::cout << "[Edit] Removed voxel under crosshair\n";
            }
        }
        left_mouse_down = left_pressed;

        bool right_pressed = window.IsMouseButtonPressed(GLFW_MOUSE_BUTTON_RIGHT);
        if (right_pressed && !right_mouse_down) {
            Vector3 added_position;
            if (AddVoxelUnderCrosshair(world, camera, MaterialDatabase::BRICK, &added_position)) {
                renderer.UpdateVoxels(world);
                request_analysis_single(added_position, "voxel added");
                std::cout << "[Edit] Added voxel under crosshair\n";
            }
        }
        right_mouse_down = right_pressed;

        bool wire_key = window.IsKeyPressed(GLFW_KEY_F1);
        if (wire_key && !wireframe_key_down) {
            wireframe_enabled = !wireframe_enabled;
            renderer.SetWireframeMode(wireframe_enabled);
            std::cout << "[Render] Wireframe " << (wireframe_enabled ? "ON" : "OFF") << "\n";
        }
        wireframe_key_down = wire_key;

        bool analyze_key = window.IsKeyPressed(GLFW_KEY_F2);
        if (analyze_key && !analyze_key_down) {
            std::vector<Vector3> hits = world.Raycast(camera.GetPosition(), camera.GetForward(), kEditRange);
            if (!hits.empty()) {
                request_analysis(hits, "manual analysis");
                std::cout << "[Analyzer] Manual analysis requested at crosshair (" << hits.size() << " samples)\n";
            } else {
                request_analysis_single(camera.GetPosition(), "manual analysis (camera origin)");
                std::cout << "[Analyzer] Manual analysis requested at camera origin\n";
            }
        }
        analyze_key_down = analyze_key;
#endif

#ifdef RENDERING_ENABLED
        run_analysis_if_needed();
#endif

        camera.ProcessKeyboard(forward, right, up, dt);

        // Render
        renderer.Render(camera, window.GetAspectRatio());

        window.SwapBuffers();

#ifndef RENDERING_ENABLED
        // In stub mode, exit after one frame
        break;
#endif
    }

    std::cout << "Rendered " << renderer.GetRenderedVoxelCount() << " voxels\n";

    // Cleanup
    renderer.Shutdown();
    window.Shutdown();

#ifdef USE_BULLET
    if (physics_integration) {
        physics_integration->ClearDebris();
    }
    if (bullet_engine) {
        bullet_engine->Shutdown();
    }
#endif

    std::cout << "\nDemo complete.\n";
    std::cout << "========================================\n";

    return 0;
}
