# 8-Week Sprint Plan: Voxel Destruction Engine Prototype

**Goal:** Build working destruction demo proving Track 1 (Structural) + Track 3 (Physics) integration

**Timeline:** Weeks 1-8  
**Team Size:** 1-2 developers  
**End Deliverable:** Interactive demo showing realistic building collapse

---

## Overview & Milestones

### Phase 1: Foundation (Weeks 1-2)
**Milestone:** Basic voxel world with rendering and interaction

### Phase 2: Structural Analysis (Weeks 3-4)
**Milestone:** Determine which voxels should fall

### Phase 3: Physics Simulation (Weeks 5-6)
**Milestone:** Make voxels fall realistically

### Phase 4: Integration & Demo (Weeks 7-8)
**Milestone:** Complete destruction pipeline working end-to-end

---

## Week 1: Voxel World Foundation

### Goals
- Set up project structure
- Implement basic voxel storage
- Create simple voxel renderer
- Add camera controls

### Day 1: Project Setup

**Morning (4 hours):**
```
□ Create project structure
  - /src/voxel/      (voxel world)
  - /src/structural/ (Track 1)
  - /src/physics/    (Track 3)
  - /src/rendering/  (visualization)
  - /tests/          (unit tests)

□ Set up build system (CMake or equivalent)
  
□ Add dependencies:
  - GLM or Eigen (math library)
  - GLFW + OpenGL (rendering)
  - GoogleTest (testing)
```

**Afternoon (4 hours):**
```cpp
□ Implement Vector3 math utilities
  
struct Vector3 {
    float x, y, z;
    Vector3 operator+(const Vector3& other);
    Vector3 operator*(float scalar);
    float Distance(const Vector3& other);
    // ... other operators
};

□ Write unit tests for Vector3

□ Implement basic Material struct

struct Material {
    std::string name;
    float density;              // kg/m³
    float compressive_strength; // N/m²
    Color color;                // For rendering
};

□ Create material database with 3 materials:
  - Wood:     density=600,  strength=10e6,  color=brown
  - Brick:    density=1900, strength=30e6,  color=red
  - Concrete: density=2400, strength=40e6,  color=gray
```

**Success Criteria:**
- ✅ Project compiles
- ✅ Unit tests pass
- ✅ Can create Vector3 and Material objects

---

### Day 2: Voxel Storage

**Morning (4 hours):**
```cpp
□ Implement Voxel struct

struct Voxel {
    uint8_t material_id;     // Index into material array
    float current_hp;        // For future damage
    bool is_active;          // Exists or air
};

□ Implement VoxelWorld class (sparse storage)

class VoxelWorld {
private:
    std::unordered_map<Vector3, Voxel> voxels;
    float voxel_size = 0.05f;  // 5cm
    
public:
    bool HasVoxel(Vector3 position);
    Voxel GetVoxel(Vector3 position);
    void SetVoxel(Vector3 position, Voxel voxel);
    void RemoveVoxel(Vector3 position);
    std::vector<Vector3> GetAllVoxelPositions();
};

□ Implement spatial hash for Vector3 (for unordered_map key)

struct Vector3Hash {
    size_t operator()(const Vector3& v) const {
        // Combine x, y, z into hash
        return hash<float>()(v.x) ^ 
               hash<float>()(v.y) << 1 ^ 
               hash<float>()(v.z) << 2;
    }
};
```

**Afternoon (4 hours):**
```cpp
□ Add neighbor finding

std::vector<Vector3> GetNeighbors(Vector3 position, 
                                  Connectivity type = SIX_CONNECTED);

enum Connectivity {
    SIX_CONNECTED,     // Face neighbors only
    TWENTY_SIX         // Face + edge + corner
};

□ Add helper functions

std::vector<Vector3> GetVoxelsInRadius(Vector3 center, float radius);
std::vector<Vector3> Raycast(Vector3 start, Vector3 direction, 
                             float max_distance);

□ Write unit tests

TEST(VoxelWorld, BasicOperations) {
    VoxelWorld world;
    Vector3 pos(0, 0, 0);
    
    world.SetVoxel(pos, Voxel{material_id: 1});
    ASSERT_TRUE(world.HasVoxel(pos));
    
    world.RemoveVoxel(pos);
    ASSERT_FALSE(world.HasVoxel(pos));
}

TEST(VoxelWorld, Neighbors) {
    VoxelWorld world;
    Vector3 center(1, 1, 1);
    world.SetVoxel(center, Voxel{1});
    
    auto neighbors = world.GetNeighbors(center);
    ASSERT_EQ(neighbors.size(), 6);  // 6-connected
}
```

**Success Criteria:**
- ✅ Can add/remove voxels
- ✅ Can query voxels by position
- ✅ Neighbor finding works correctly
- ✅ Unit tests pass

---

### Day 3: Basic Rendering

**Morning (4 hours):**
```cpp
□ Set up OpenGL context (GLFW)

void InitWindow(int width, int height);
void PollEvents();
bool ShouldClose();

□ Implement basic camera

class Camera {
    Vector3 position;
    Vector3 forward, right, up;
    float yaw, pitch;
    
public:
    void ProcessInput(float deltaTime);
    glm::mat4 GetViewMatrix();
    glm::mat4 GetProjectionMatrix(float aspect);
};

□ WASD movement + mouse look
```

**Afternoon (4 hours):**
```cpp
□ Implement voxel renderer (instanced cubes)

class VoxelRenderer {
private:
    GLuint vao, vbo, ebo;
    GLuint shader_program;
    std::vector<glm::mat4> instance_matrices;
    std::vector<glm::vec3> instance_colors;
    
public:
    void Initialize();
    void UpdateVoxels(VoxelWorld& world);
    void Render(Camera& camera);
};

□ Simple vertex shader

#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in mat4 aInstanceMatrix;
layout (location = 5) in vec3 aInstanceColor;

uniform mat4 view;
uniform mat4 projection;

out vec3 Color;

void main() {
    gl_Position = projection * view * aInstanceMatrix * vec4(aPos, 1.0);
    Color = aInstanceColor;
}

□ Simple fragment shader

#version 330 core
in vec3 Color;
out vec4 FragColor;

void main() {
    FragColor = vec4(Color, 1.0);
}
```

**Success Criteria:**
- ✅ Window opens
- ✅ Camera moves with WASD
- ✅ Camera rotates with mouse
- ✅ Can see 3D space (draw axes for reference)

---

### Day 4: Render Voxels

**Morning (4 hours):**
```cpp
□ Create test voxel structures

void CreateTestTower(VoxelWorld& world, int height) {
    for (int y = 0; y < height; y++) {
        world.SetVoxel(Vector3(0, y * 0.05f, 0), 
                      Voxel{material_id: BRICK});
    }
}

void CreateTestWall(VoxelWorld& world, int width, int height) {
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            world.SetVoxel(Vector3(x * 0.05f, y * 0.05f, 0),
                          Voxel{material_id: BRICK});
        }
    }
}

□ Optimize rendering (only update when voxels change)

bool voxels_dirty = true;

void OnVoxelChanged() {
    voxels_dirty = true;
}

void Render() {
    if (voxels_dirty) {
        renderer.UpdateVoxels(world);
        voxels_dirty = false;
    }
    renderer.Render(camera);
}
```

**Afternoon (4 hours):**
```cpp
□ Add mouse picking (click to remove voxels)

Vector3 GetRayFromMouse(int mouse_x, int mouse_y, Camera& camera);

void OnMouseClick(int button, int x, int y) {
    if (button == LEFT_MOUSE) {
        Vector3 ray = GetRayFromMouse(x, y, camera);
        auto hit = world.Raycast(camera.position, ray, 100.0f);
        
        if (!hit.empty()) {
            world.RemoveVoxel(hit[0]);
            voxels_dirty = true;
        }
    }
}

□ Add UI overlay (ImGui or simple text)
  - FPS counter
  - Voxel count
  - Instructions ("Click to remove voxels")
```

**Success Criteria:**
- ✅ Can see voxels rendered as cubes
- ✅ Different materials have different colors
- ✅ Can click on voxels to remove them
- ✅ FPS shown in UI

---

### Day 5: Testing & Polish

**Morning (4 hours):**
```cpp
□ Create more test scenarios

void CreateTestBridge(VoxelWorld& world) {
    // Support pillars
    for (int y = 0; y < 10; y++) {
        world.SetVoxel(Vector3(0, y * 0.05f, 0), Voxel{BRICK});
        world.SetVoxel(Vector3(10 * 0.05f, y * 0.05f, 0), Voxel{BRICK});
    }
    
    // Bridge deck
    for (int x = 0; x < 11; x++) {
        world.SetVoxel(Vector3(x * 0.05f, 10 * 0.05f, 0), Voxel{CONCRETE});
    }
}

void CreateTestBuilding(VoxelWorld& world, int width, int depth, int height) {
    // Hollow building with walls and floor
    for (int y = 0; y < height; y++) {
        // Walls
        for (int x = 0; x < width; x++) {
            world.SetVoxel(Vector3(x * 0.05f, y * 0.05f, 0), Voxel{BRICK});
            world.SetVoxel(Vector3(x * 0.05f, y * 0.05f, depth * 0.05f), 
                          Voxel{BRICK});
        }
        for (int z = 0; z < depth; z++) {
            world.SetVoxel(Vector3(0, y * 0.05f, z * 0.05f), Voxel{BRICK});
            world.SetVoxel(Vector3(width * 0.05f, y * 0.05f, z * 0.05f), 
                          Voxel{BRICK});
        }
    }
}

□ Add scene selector (press 1-5 to load different test scenes)
```

**Afternoon (4 hours):**
```cpp
□ Performance optimization

// Only render visible voxels (frustum culling)
void UpdateVisibleVoxels(Camera& camera) {
    // Simple: Skip voxels outside camera view
}

// Batch rendering improvements
// Profile rendering time

□ Add basic lighting (optional but nice)

// Simple diffuse lighting from sun direction
vec3 lightDir = normalize(vec3(0.5, 1.0, 0.3));
float diff = max(dot(normal, lightDir), 0.0);
vec3 diffuse = diff * Color;

□ Documentation
  - README with build instructions
  - How to run
  - Controls (WASD, mouse, click)
```

**Success Criteria:**
- ✅ Can load different test scenes
- ✅ Rendering is smooth (60+ FPS with 10K voxels)
- ✅ UI is usable
- ✅ Code is documented

---

### Week 1 Deliverable

**What you should have:**

1. **Running application** that:
   - Displays 3D voxel world
   - Allows camera movement (WASD + mouse)
   - Allows voxel destruction (click to remove)
   - Shows FPS and voxel count

2. **Test scenes:**
   - Tower (10m tall)
   - Wall (5m × 3m)
   - Bridge (10m span)
   - Building (5m × 5m × 10m)

3. **Performance:**
   - 60+ FPS with 10,000 voxels
   - Smooth camera movement
   - Responsive voxel removal

4. **Code quality:**
   - Unit tests passing
   - Clean architecture
   - Documented

**Demo video (30 seconds):**
- Show camera flying around test scenes
- Click to remove voxels
- Change between scenes

**Review & Decision:**
- ✅ Foundation solid? → Proceed to Week 2
- ❌ Issues? → Fix before continuing

---

## Week 2: Voxel System Completion

### Goals
- Add surface voxel detection
- Implement spatial hashing for performance
- Add voxel clustering
- Prepare for Track 1 integration

### Day 6: Surface Detection

**Morning (4 hours):**
```cpp
□ Implement surface voxel detection

bool IsSurfaceVoxel(Vector3 position) {
    // A voxel is "surface" if any neighbor is air
    auto neighbors = world.GetNeighbors(position, SIX_CONNECTED);
    
    for (auto& neighbor_pos : neighbors) {
        if (!world.HasVoxel(neighbor_pos)) {
            return true;  // Has air neighbor = surface
        }
    }
    
    return false;  // Completely surrounded = interior
}

std::vector<Vector3> GetSurfaceVoxels() {
    std::vector<Vector3> surface;
    
    for (auto& [pos, voxel] : all_voxels) {
        if (IsSurfaceVoxel(pos)) {
            surface.push_back(pos);
        }
    }
    
    return surface;
}

□ Add visualization toggle (show only surface voxels)

void Render() {
    if (show_surface_only) {
        auto surface = world.GetSurfaceVoxels();
        renderer.UpdateVoxels(surface);
    } else {
        renderer.UpdateVoxels(world.GetAllVoxels());
    }
}
```

**Afternoon (4 hours):**
```cpp
□ Optimize surface detection (cache results)

class VoxelWorld {
private:
    std::unordered_set<Vector3> surface_cache;
    bool surface_cache_dirty = true;
    
public:
    void InvalidateSurfaceCache() {
        surface_cache_dirty = true;
    }
    
    const std::unordered_set<Vector3>& GetSurfaceVoxels() {
        if (surface_cache_dirty) {
            UpdateSurfaceCache();
            surface_cache_dirty = false;
        }
        return surface_cache;
    }
    
    void RemoveVoxel(Vector3 pos) {
        voxels.erase(pos);
        
        // Invalidate surface cache for this voxel and neighbors
        surface_cache.erase(pos);
        for (auto& neighbor : GetNeighbors(pos)) {
            // Neighbor might now be surface
            if (IsSurfaceVoxel(neighbor)) {
                surface_cache.insert(neighbor);
            }
        }
    }
};

□ Write performance tests

TEST(Performance, SurfaceDetection) {
    VoxelWorld world;
    CreateTestBuilding(world, 20, 20, 20);  // 8000 voxels
    
    auto start = Clock::now();
    auto surface = world.GetSurfaceVoxels();
    auto elapsed = GetElapsedMS(start);
    
    ASSERT_LT(elapsed, 100);  // Should be < 100ms
    
    // Surface should be much smaller than total
    ASSERT_LT(surface.size(), world.GetVoxelCount() * 0.5f);
}
```

**Success Criteria:**
- ✅ Can identify surface voxels
- ✅ Surface detection is fast (< 100ms for 10K voxels)
- ✅ Cache invalidation works correctly
- ✅ Can toggle surface visualization

---

### Day 7: Spatial Hashing

**Morning (4 hours):**
```cpp
□ Implement spatial hash grid

class SpatialHash {
private:
    std::unordered_map<GridCell, std::vector<Vector3>> grid;
    float cell_size;
    
    struct GridCell {
        int x, y, z;
        
        bool operator==(const GridCell& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };
    
    struct GridCellHash {
        size_t operator()(const GridCell& cell) const {
            return hash<int>()(cell.x) ^ 
                   (hash<int>()(cell.y) << 1) ^ 
                   (hash<int>()(cell.z) << 2);
        }
    };
    
public:
    SpatialHash(float cell_size) : cell_size(cell_size) {}
    
    GridCell GetCell(Vector3 position) {
        return GridCell{
            (int)(position.x / cell_size),
            (int)(position.y / cell_size),
            (int)(position.z / cell_size)
        };
    }
    
    void Insert(Vector3 position) {
        GridCell cell = GetCell(position);
        grid[cell].push_back(position);
    }
    
    std::vector<Vector3> QueryRadius(Vector3 center, float radius) {
        std::vector<Vector3> results;
        
        // Check cells in radius
        int cell_radius = (int)ceil(radius / cell_size);
        GridCell center_cell = GetCell(center);
        
        for (int dx = -cell_radius; dx <= cell_radius; dx++) {
        for (int dy = -cell_radius; dy <= cell_radius; dy++) {
        for (int dz = -cell_radius; dz <= cell_radius; dz++) {
            GridCell check_cell{
                center_cell.x + dx,
                center_cell.y + dy,
                center_cell.z + dz
            };
            
            if (grid.count(check_cell)) {
                for (auto& pos : grid[check_cell]) {
                    if (Distance(pos, center) <= radius) {
                        results.push_back(pos);
                    }
                }
            }
        }}}
        
        return results;
    }
};
```

**Afternoon (4 hours):**
```cpp
□ Integrate spatial hash into VoxelWorld

class VoxelWorld {
private:
    SpatialHash spatial_hash{1.0f};  // 1m cells
    
public:
    void SetVoxel(Vector3 pos, Voxel voxel) {
        voxels[pos] = voxel;
        spatial_hash.Insert(pos);
    }
    
    std::vector<Vector3> GetNeighborsFast(Vector3 pos) {
        // Query spatial hash instead of checking all voxels
        auto nearby = spatial_hash.QueryRadius(pos, voxel_size * 2);
        
        std::vector<Vector3> neighbors;
        for (auto& candidate : nearby) {
            if (Distance(candidate, pos) <= voxel_size * 1.1f &&
                candidate != pos) {
                neighbors.push_back(candidate);
            }
        }
        
        return neighbors;
    }
};

□ Benchmark performance improvement

TEST(Performance, SpatialHashSpeedup) {
    VoxelWorld world;
    CreateLargeStructure(world, 50, 50, 20);  // 50K voxels
    
    // Old method (check all voxels)
    auto start1 = Clock::now();
    for (int i = 0; i < 1000; i++) {
        auto neighbors = world.GetNeighborsSlow(random_position);
    }
    auto time_slow = GetElapsedMS(start1);
    
    // New method (spatial hash)
    auto start2 = Clock::now();
    for (int i = 0; i < 1000; i++) {
        auto neighbors = world.GetNeighborsFast(random_position);
    }
    auto time_fast = GetElapsedMS(start2);
    
    std::cout << "Speedup: " << (time_slow / time_fast) << "x\n";
    ASSERT_GT(time_slow / time_fast, 10.0f);  // At least 10x faster
}
```

**Success Criteria:**
- ✅ Spatial hash implemented
- ✅ 10x+ speedup on neighbor queries
- ✅ Works correctly with large structures (50K+ voxels)

---

### Day 8: Clustering

**Morning (4 hours):**
```cpp
□ Implement flood-fill clustering

struct VoxelCluster {
    std::vector<Vector3> voxel_positions;
    Vector3 center_of_mass;
    float total_mass;
    BoundingBox bounds;
};

VoxelCluster FloodFillCluster(Vector3 seed, 
                              std::unordered_set<Vector3>& visited) {
    VoxelCluster cluster;
    std::queue<Vector3> queue;
    
    queue.push(seed);
    visited.insert(seed);
    
    while (!queue.empty()) {
        Vector3 current = queue.front();
        queue.pop();
        
        cluster.voxel_positions.push_back(current);
        
        // Add unvisited neighbors
        auto neighbors = world.GetNeighbors(current);
        for (auto& neighbor : neighbors) {
            if (visited.find(neighbor) == visited.end() &&
                world.HasVoxel(neighbor)) {
                queue.push(neighbor);
                visited.insert(neighbor);
            }
        }
    }
    
    // Calculate cluster properties
    cluster.center_of_mass = CalculateCenterOfMass(cluster);
    cluster.total_mass = CalculateTotalMass(cluster);
    cluster.bounds = CalculateBounds(cluster);
    
    return cluster;
}

std::vector<VoxelCluster> FindAllClusters() {
    std::vector<VoxelCluster> clusters;
    std::unordered_set<Vector3> visited;
    
    for (auto& [pos, voxel] : all_voxels) {
        if (visited.find(pos) == visited.end()) {
            VoxelCluster cluster = FloodFillCluster(pos, visited);
            clusters.push_back(cluster);
        }
    }
    
    return clusters;
}
```

**Afternoon (4 hours):**
```cpp
□ Implement cluster property calculations

Vector3 CalculateCenterOfMass(VoxelCluster& cluster) {
    Vector3 com(0, 0, 0);
    float total_mass = 0;
    
    for (auto& pos : cluster.voxel_positions) {
        Voxel voxel = world.GetVoxel(pos);
        Material mat = materials[voxel.material_id];
        
        float voxel_volume = pow(voxel_size, 3);
        float voxel_mass = mat.density * voxel_volume;
        
        com += pos * voxel_mass;
        total_mass += voxel_mass;
    }
    
    return com / total_mass;
}

float CalculateTotalMass(VoxelCluster& cluster) {
    float total = 0;
    
    for (auto& pos : cluster.voxel_positions) {
        Voxel voxel = world.GetVoxel(pos);
        Material mat = materials[voxel.material_id];
        total += mat.density * pow(voxel_size, 3);
    }
    
    return total;
}

BoundingBox CalculateBounds(VoxelCluster& cluster) {
    Vector3 min = cluster.voxel_positions[0];
    Vector3 max = cluster.voxel_positions[0];
    
    for (auto& pos : cluster.voxel_positions) {
        min.x = std::min(min.x, pos.x);
        min.y = std::min(min.y, pos.y);
        min.z = std::min(min.z, pos.z);
        max.x = std::max(max.x, pos.x);
        max.y = std::max(max.y, pos.y);
        max.z = std::max(max.z, pos.z);
    }
    
    return BoundingBox{min, max};
}

□ Add cluster visualization

void RenderClusters(std::vector<VoxelCluster>& clusters) {
    for (int i = 0; i < clusters.size(); i++) {
        Color color = GetClusterColor(i);  // Each cluster different color
        
        for (auto& pos : clusters[i].voxel_positions) {
            DrawVoxel(pos, color);
        }
        
        // Draw center of mass
        DrawSphere(clusters[i].center_of_mass, 0.1f, Color::Yellow);
        
        // Draw bounding box
        DrawBoundingBox(clusters[i].bounds, Color::White);
    }
}
```

**Success Criteria:**
- ✅ Can find all disconnected clusters
- ✅ Cluster properties calculated correctly
- ✅ Can visualize clusters with different colors
- ✅ Center of mass and bounds shown

---

### Day 9: Ground Detection

**Morning (4 hours):**
```cpp
□ Implement ground anchor detection

bool IsGroundAnchor(Vector3 position) {
    // A voxel is grounded if it touches the ground level
    return position.y <= ground_level + voxel_size * 0.5f;
}

std::vector<Vector3> GetGroundAnchors() {
    std::vector<Vector3> anchors;
    
    for (auto& [pos, voxel] : all_voxels) {
        if (IsGroundAnchor(pos)) {
            anchors.push_back(pos);
        }
    }
    
    return anchors;
}

□ Implement ground connectivity check

bool IsConnectedToGround(Vector3 position) {
    // BFS from this position, see if we reach ground
    std::unordered_set<Vector3> visited;
    std::queue<Vector3> queue;
    
    queue.push(position);
    visited.insert(position);
    
    while (!queue.empty()) {
        Vector3 current = queue.front();
        queue.pop();
        
        // Reached ground?
        if (IsGroundAnchor(current)) {
            return true;
        }
        
        // Check neighbors
        auto neighbors = world.GetNeighbors(current);
        for (auto& neighbor : neighbors) {
            if (visited.find(neighbor) == visited.end() &&
                world.HasVoxel(neighbor)) {
                queue.push(neighbor);
                visited.insert(neighbor);
            }
        }
    }
    
    return false;  // No path to ground
}
```

**Afternoon (4 hours):**
```cpp
□ Find floating clusters

std::vector<VoxelCluster> FindFloatingClusters() {
    std::vector<VoxelCluster> all_clusters = FindAllClusters();
    std::vector<VoxelCluster> floating;
    
    for (auto& cluster : all_clusters) {
        // Check if any voxel in cluster touches ground
        bool grounded = false;
        
        for (auto& pos : cluster.voxel_positions) {
            if (IsGroundAnchor(pos)) {
                grounded = true;
                break;
            }
        }
        
        if (!grounded) {
            floating.push_back(cluster);
        }
    }
    
    return floating;
}

□ Test with scenarios

TEST(Clustering, DetectsFloatingStructure) {
    VoxelWorld world;
    
    // Create grounded tower
    CreateTestTower(world, 10);
    
    // Create floating platform above it (not connected)
    for (int x = 5; x < 10; x++) {
        for (int z = 5; z < 10; z++) {
            world.SetVoxel(Vector3(x * 0.05f, 20 * 0.05f, z * 0.05f),
                          Voxel{CONCRETE});
        }
    }
    
    auto floating = FindFloatingClusters();
    
    ASSERT_EQ(floating.size(), 1);  // The platform
    ASSERT_EQ(floating[0].voxel_positions.size(), 25);  // 5×5 platform
}
```

**Success Criteria:**
- ✅ Can detect ground anchors
- ✅ Can determine if voxel connected to ground
- ✅ Can find all floating clusters
- ✅ Unit tests pass

---

### Day 10: Week 2 Polish & Testing

**Morning (4 hours):**
```cpp
□ Create comprehensive test suite

TEST(VoxelSystem, TowerCollapseDetection) {
    VoxelWorld world;
    CreateTestTower(world, 20);
    
    // Remove base
    world.RemoveVoxel(Vector3(0, 0, 0));
    
    // Everything above should be floating
    auto floating = FindFloatingClusters();
    ASSERT_EQ(floating.size(), 1);
    ASSERT_EQ(floating[0].voxel_positions.size(), 19);
}

TEST(VoxelSystem, BridgeCollapseDetection) {
    VoxelWorld world;
    CreateTestBridge(world);
    
    // Remove one support
    for (int y = 0; y < 10; y++) {
        world.RemoveVoxel(Vector3(0, y * 0.05f, 0));
    }
    
    // Bridge deck should still be connected via other support
    auto floating = FindFloatingClusters();
    ASSERT_EQ(floating.size(), 1);  // Only the removed support
}

TEST(VoxelSystem, PartialWallRemoval) {
    VoxelWorld world;
    CreateTestWall(world, 20, 10);
    
    // Remove middle section
    for (int x = 8; x < 12; x++) {
        for (int y = 0; y < 10; y++) {
            world.RemoveVoxel(Vector3(x * 0.05f, y * 0.05f, 0));
        }
    }
    
    // Should have 2 disconnected clusters
    auto clusters = FindAllClusters();
    ASSERT_EQ(clusters.size(), 2);
}

□ Run all tests, fix any failures
```

**Afternoon (4 hours):**
```cpp
□ Performance profiling

void ProfileVoxelSystem() {
    VoxelWorld world;
    CreateLargeStructure(world, 100, 100, 50);  // 500K voxels
    
    auto start = Clock::now();
    auto surface = world.GetSurfaceVoxels();
    auto t1 = GetElapsedMS(start);
    
    auto clusters = FindAllClusters();
    auto t2 = GetElapsedMS(start);
    
    auto floating = FindFloatingClusters();
    auto t3 = GetElapsedMS(start);
    
    std::cout << "Performance Report (500K voxels):\n";
    std::cout << "  Surface detection: " << t1 << "ms\n";
    std::cout << "  Clustering: " << (t2 - t1) << "ms\n";
    std::cout << "  Ground check: " << (t3 - t2) << "ms\n";
    std::cout << "  Total: " << t3 << "ms\n";
    std::cout << "  Surface voxels: " << surface.size() << "\n";
    std::cout << "  Clusters: " << clusters.size() << "\n";
}

□ Optimize any bottlenecks identified

□ Update documentation
  - Document all new classes/functions
  - Add usage examples
  - Update README
```

**Success Criteria:**
- ✅ All unit tests pass
- ✅ Performance acceptable (< 1 second for 100K voxels)
- ✅ Code documented
- ✅ Ready for Track 1 integration

---

### Week 2 Deliverable

**What you should have:**

1. **Voxel System Features:**
   - Surface voxel detection (cached)
   - Spatial hashing (10x+ speedup)
   - Cluster finding (flood-fill)
   - Ground connectivity detection
   - Floating cluster identification

2. **Performance:**
   - Can handle 100K+ voxels
   - Surface detection < 100ms
   - Clustering < 500ms
   - Spatial queries < 1ms

3. **Test Coverage:**
   - 10+ unit tests
   - All passing
   - Edge cases covered

4. **Visualization:**
   - Can toggle surface-only view
   - Can color-code clusters
   - Shows center of mass
   - Shows bounding boxes

**Demo video (1 minute):**
- Show large structure (50K voxels)
- Toggle surface visualization (show interior removed)
- Remove voxels to create separate clusters
- Show clusters colored differently
- Show floating detection

**Review & Decision:**
- ✅ Voxel system solid? → Proceed to Week 3 (Track 1)
- ❌ Performance issues? → Optimize before continuing

---

## Week 3: Structural Analysis - Basic Implementation

### Goals
- Implement displacement-based spring system
- Calculate mass supported by each voxel
- Detect structural failures
- Output failed clusters

### Day 11: Spring System Foundation

**Morning (4 hours):**
```cpp
□ Create SpringNode data structure

struct SpringNode {
    Vector3 position;
    Material material;
    
    // Spring physics
    float displacement = 0;     // How far sagged downward
    float velocity = 0;          // Rate of change
    float mass_supported = 0;    // Total weight above
    
    // Connectivity
    std::vector<SpringNode*> neighbors;
    bool is_ground_anchor = false;
    
    // For debugging
    int id;
};

□ Create StructuralAnalyzer class

class StructuralAnalyzer {
private:
    VoxelWorld* world;
    std::vector<SpringNode*> nodes;
    
    // Parameters (tunable)
    float timestep = 0.01f;
    float damping = 0.9f;
    int max_iterations = 100;
    float convergence_threshold = 0.0001f;
    
public:
    AnalysisResult Analyze(VoxelWorld& voxel_world,
                          std::vector<Vector3> damaged_positions,
                          float time_budget_ms = 500);
    
private:
    void BuildNodeGraph(std::vector<Vector3>& damaged);
    void CalculateMassSupported();
    bool SolveDisplacements();
    std::vector<SpringNode*> DetectFailures();
    std::vector<VoxelCluster> FindClusters(std::vector<SpringNode*>& failed);
};

struct AnalysisResult {
    bool structure_failed;
    std::vector<VoxelCluster> failed_clusters;
    float calculation_time_ms;
    std::string debug_info;
};
```

**Afternoon (4 hours):**
```cpp
□ Implement node graph building

void BuildNodeGraph(std::vector<Vector3>& damaged_positions) {
    nodes.clear();
    
    // Find all surface voxels near damage
    std::unordered_set<Vector3> surface_voxels;
    
    for (auto& pos : damaged_positions) {
        // Check 3×3×3 neighborhood
        for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
        for (int dz = -1; dz <= 1; dz++) {
            Vector3 check_pos = pos + Vector3(dx, dy, dz) * voxel_size;
            
            if (world->HasVoxel(check_pos) && 
                world->IsSurfaceVoxel(check_pos)) {
                surface_voxels.insert(check_pos);
            }
        }}}
    }
    
    // Create nodes
    std::unordered_map<Vector3, SpringNode*> node_map;
    int id = 0;
    
    for (auto& pos : surface_voxels) {
        Voxel voxel = world->GetVoxel(pos);
        Material mat = materials[voxel.material_id];
        
        SpringNode* node = new SpringNode{
            .position = pos,
            .material = mat,
            .is_ground_anchor = IsGroundAnchor(pos),
            .id = id++
        };
        
        nodes.push_back(node);
        node_map[pos] = node;
    }
    
    // Connect neighbors
    for (auto* node : nodes) {
        auto neighbor_positions = world->GetNeighbors(node->position);
        
        for (auto& npos : neighbor_positions) {
            if (node_map.count(npos)) {
                node->neighbors.push_back(node_map[npos]);
            }
        }
    }
    
    std::cout << "Built graph: " << nodes.size() << " nodes\n";
}

□ Write test

TEST(Structural, GraphBuilding) {
    VoxelWorld world;
    CreateTestTower(world, 10);
    
    StructuralAnalyzer analyzer;
    
    // Remove base
    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};
    world.RemoveVoxel(damaged[0]);
    
    analyzer.BuildNodeGraph(damaged);
    
    // Should create nodes for voxels near damage
    ASSERT_GT(analyzer.GetNodeCount(), 0);
}
```

**Success Criteria:**
- ✅ Can build node graph from surface voxels
- ✅ Nodes correctly connected to neighbors
- ✅ Ground anchors identified
- ✅ Unit test passes

---

### Day 12: Mass Calculation

**Morning (4 hours):**
```cpp
□ Implement mass supported calculation (raycast upward)

void CalculateMassSupported() {
    for (auto* node : nodes) {
        if (node->is_ground_anchor) {
            node->mass_supported = 0;  // Ground supports itself
            continue;
        }
        
        // Raycast upward to find all mass above
        float total_mass = 0;
        
        Vector3 ray_origin = node->position;
        Vector3 ray_direction(0, 1, 0);  // Up
        float max_distance = 100.0f;
        
        auto hits = world->Raycast(ray_origin, ray_direction, max_distance);
        
        for (auto& hit_pos : hits) {
            Voxel voxel = world->GetVoxel(hit_pos);
            Material mat = materials[voxel.material_id];
            
            float voxel_volume = pow(voxel_size, 3);
            float voxel_mass = mat.density * voxel_volume;
            
            total_mass += voxel_mass;
        }
        
        node->mass_supported = total_mass;
    }
}

□ Add caching for mass calculation

std::unordered_map<Vector3, float> mass_cache;

float GetCachedMassSupported(Vector3 position) {
    if (mass_cache.count(position)) {
        return mass_cache[position];
    }
    
    float mass = RaycastMassAbove(position);
    mass_cache[position] = mass;
    return mass;
}

void InvalidateMassCache(std::vector<Vector3>& changed_positions) {
    for (auto& pos : changed_positions) {
        mass_cache.erase(pos);
        
        // Also invalidate everything below (their mass-above changed)
        for (float y = pos.y - 100; y < pos.y; y += voxel_size) {
            mass_cache.erase(Vector3(pos.x, y, pos.z));
        }
    }
}
```

**Afternoon (4 hours):**
```cpp
□ Test mass calculation

TEST(Structural, MassCalculation) {
    VoxelWorld world;
    
    // Create 3-voxel tower
    world.SetVoxel(Vector3(0, 0, 0), Voxel{CONCRETE});  // Base
    world.SetVoxel(Vector3(0, 0.05, 0), Voxel{CONCRETE});  // Middle
    world.SetVoxel(Vector3(0, 0.10, 0), Voxel{CONCRETE});  // Top
    
    StructuralAnalyzer analyzer;
    analyzer.BuildNodeGraph({/* no damage */});
    analyzer.CalculateMassSupported();
    
    // Base should support ~0.6 kg (2 voxels above)
    auto* base_node = analyzer.GetNode(Vector3(0, 0, 0));
    float expected_mass = 2 * pow(0.05, 3) * 2400;  // 2 voxels concrete
    
    ASSERT_NEAR(base_node->mass_supported, expected_mass, 0.01);
}

□ Optimize for large structures

// Use spatial hash to speed up raycasting
// Only check voxels in vertical column

std::vector<Vector3> RaycastVerticalOptimized(Vector3 start) {
    std::vector<Vector3> hits;
    
    for (float y = start.y; y < max_height; y += voxel_size) {
        Vector3 check_pos(start.x, y, start.z);
        if (world->HasVoxel(check_pos)) {
            hits.push_back(check_pos);
        }
    }
    
    return hits;
}

□ Profile mass calculation

void ProfileMassCalculation() {
    auto start = Clock::now();
    CalculateMassSupported();
    auto elapsed = GetElapsedMS(start);
    
    std::cout << "Mass calculation: " << elapsed << "ms\n";
    std::cout << "  Nodes processed: " << nodes.size() << "\n";
    std::cout << "  ms per node: " << (elapsed / nodes.size()) << "\n";
}
```

**Success Criteria:**
- ✅ Mass calculated correctly (unit test)
- ✅ Fast enough (< 200ms for 1000 nodes)
- ✅ Caching works
- ✅ Profile shows reasonable performance

---

### Day 13: Displacement Solver

**Morning (4 hours):**
```cpp
□ Implement iterative displacement solver

bool SolveDisplacements() {
    bool converged = false;
    
    for (int iter = 0; iter < max_iterations; iter++) {
        float max_change = 0;
        
        for (auto* node : nodes) {
            if (node->is_ground_anchor) {
                node->displacement = 0;  // Ground doesn't move
                continue;
            }
            
            // Gravitational force (downward)
            float F_gravity = node->mass_supported * 9.81f;
            
            // Spring forces (from neighbors)
            float F_spring = 0;
            
            for (auto* neighbor : node->neighbors) {
                float displacement_diff = neighbor->displacement - node->displacement;
                float spring_force = node->material.spring_constant * displacement_diff;
                F_spring += spring_force;
            }
            
            float F_total = F_gravity + F_spring;
            
            // F = ma, so a = F/m
            float acceleration = F_total / node->mass_supported;
            
            // Update velocity (Euler integration)
            node->velocity += acceleration * timestep;
            
            // Apply damping
            node->velocity *= damping;
            
            // Update displacement
            float old_displacement = node->displacement;
            node->displacement += node->velocity * timestep;
            
            // Clamp to non-negative (can't sag upward)
            node->displacement = std::max(0.0f, node->displacement);
            
            // Track convergence
            max_change = std::max(max_change, 
                                std::abs(node->displacement - old_displacement));
        }
        
        // Check convergence
        if (max_change < convergence_threshold) {
            converged = true;
            std::cout << "Converged in " << iter << " iterations\n";
            break;
        }
    }
    
    return converged;
}
```

**Afternoon (4 hours):**
```cpp
□ Add debugging visualization

void VisualizeDisplacements() {
    float max_displacement = 0;
    for (auto* node : nodes) {
        max_displacement = std::max(max_displacement, node->displacement);
    }
    
    for (auto* node : nodes) {
        // Color based on displacement (blue = 0, red = max)
        float normalized = node->displacement / max_displacement;
        Color color = Lerp(Color::Blue, Color::Red, normalized);
        
        DrawVoxel(node->position, color);
        
        // Draw displacement as line
        Vector3 end = node->position + Vector3(0, -node->displacement, 0);
        DrawLine(node->position, end, Color::Yellow);
    }
}

□ Test convergence

TEST(Structural, ConvergenceTest) {
    VoxelWorld world;
    CreateTestTower(world, 10);
    
    StructuralAnalyzer analyzer;
    
    std::vector<Vector3> damaged = {Vector3(0, 0.05, 0)};  // Remove middle
    world.RemoveVoxel(damaged[0]);
    
    analyzer.BuildNodeGraph(damaged);
    analyzer.CalculateMassSupported();
    bool converged = analyzer.SolveDisplacements();
    
    ASSERT_TRUE(converged);
}

□ Test parameter sensitivity

void TestParameterSensitivity() {
    // Test different timesteps
    for (float dt = 0.001; dt <= 0.1; dt *= 2) {
        analyzer.timestep = dt;
        bool stable = RunSimulation();
        std::cout << "dt=" << dt << " stable=" << stable << "\n";
    }
    
    // Test different damping
    for (float d = 0.5; d <= 0.99; d += 0.1) {
        analyzer.damping = d;
        bool converged = RunSimulation();
        std::cout << "damping=" << d << " converged=" << converged << "\n";
    }
}
```

**Success Criteria:**
- ✅ Solver converges for simple cases
- ✅ Can visualize displacement
- ✅ Parameters tuned for stability
- ✅ Performance acceptable (< 100ms for 1000 nodes)

---

### Day 14: Failure Detection

**Morning (4 hours):**
```cpp
□ Implement failure detection

std::vector<SpringNode*> DetectFailures() {
    std::vector<SpringNode*> failed;
    
    // Criterion 1: Excessive displacement
    const float MAX_DISPLACEMENT = 0.02f;  // 2cm for 5cm voxel
    
    for (auto* node : nodes) {
        if (node->displacement > MAX_DISPLACEMENT) {
            failed.push_back(node);
        }
    }
    
    // Criterion 2: Ground connectivity
    auto floating = FindFloatingNodes();
    failed.insert(failed.end(), floating.begin(), floating.end());
    
    return failed;
}

std::vector<SpringNode*> FindFloatingNodes() {
    std::unordered_set<SpringNode*> grounded;
    std::queue<SpringNode*> queue;
    
    // Start from all ground anchors
    for (auto* node : nodes) {
        if (node->is_ground_anchor) {
            queue.push(node);
            grounded.insert(node);
        }
    }
    
    // Flood-fill to find all connected to ground
    while (!queue.empty()) {
        SpringNode* current = queue.front();
        queue.pop();
        
        for (auto* neighbor : current->neighbors) {
            if (grounded.find(neighbor) == grounded.end()) {
                grounded.insert(neighbor);
                queue.push(neighbor);
            }
        }
    }
    
    // Any node not grounded is floating
    std::vector<SpringNode*> floating;
    for (auto* node : nodes) {
        if (grounded.find(node) == grounded.end()) {
            floating.push_back(node);
        }
    }
    
    return floating;
}
```

**Afternoon (4 hours):**
```cpp
□ Convert failed nodes to voxel clusters

std::vector<VoxelCluster> FindClusters(std::vector<SpringNode*>& failed_nodes) {
    std::unordered_set<SpringNode*> visited;
    std::vector<VoxelCluster> clusters;
    
    for (auto* seed : failed_nodes) {
        if (visited.count(seed)) continue;
        
        // Flood-fill to find connected group
        VoxelCluster cluster;
        std::queue<SpringNode*> queue;
        
        queue.push(seed);
        visited.insert(seed);
        
        while (!queue.empty()) {
            SpringNode* current = queue.front();
            queue.pop();
            
            cluster.voxel_positions.push_back(current->position);
            
            for (auto* neighbor : current->neighbors) {
                if (IsInFailedSet(neighbor, failed_nodes) && 
                    !visited.count(neighbor)) {
                    queue.push(neighbor);
                    visited.insert(neighbor);
                }
            }
        }
        
        // Calculate cluster properties
        cluster.center_of_mass = CalculateCenterOfMass(cluster);
        cluster.total_mass = CalculateTotalMass(cluster);
        cluster.bounds = CalculateBounds(cluster);
        
        clusters.push_back(cluster);
    }
    
    return clusters;
}

□ Test failure detection

TEST(Structural, DetectsTowerCollapse) {
    VoxelWorld world;
    CreateTestTower(world, 10);
    
    StructuralAnalyzer analyzer;
    
    // Remove base
    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};
    world.RemoveVoxel(damaged[0]);
    
    auto result = analyzer.Analyze(world, damaged);
    
    ASSERT_TRUE(result.structure_failed);
    ASSERT_EQ(result.failed_clusters.size(), 1);
    ASSERT_EQ(result.failed_clusters[0].voxel_positions.size(), 9);
}

TEST(Structural, StableStructureNotFailing) {
    VoxelWorld world;
    CreateTestWall(world, 10, 10);
    
    // Remove non-critical voxel
    std::vector<Vector3> damaged = {Vector3(5 * 0.05f, 5 * 0.05f, 0)};
    world.RemoveVoxel(damaged[0]);
    
    StructuralAnalyzer analyzer;
    auto result = analyzer.Analyze(world, damaged);
    
    ASSERT_FALSE(result.structure_failed);  // Should stay up
}
```

**Success Criteria:**
- ✅ Detects tower collapse correctly
- ✅ Doesn't false-positive on stable structures
- ✅ Outputs valid VoxelCluster objects
- ✅ Unit tests pass

---

### Day 15: Week 3 Integration & Testing

**Morning (4 hours):**
```cpp
□ Complete Analyze() method

AnalysisResult Analyze(VoxelWorld& voxel_world,
                       std::vector<Vector3> damaged_positions,
                       float time_budget_ms) {
    world = &voxel_world;
    auto start = Clock::now();
    
    // Step 1: Build node graph
    BuildNodeGraph(damaged_positions);
    
    // Step 2: Calculate mass supported
    CalculateMassSupported();
    
    // Step 3: Solve for displacements
    bool converged = SolveDisplacements();
    
    // Step 4: Detect failures
    auto failed_nodes = DetectFailures();
    
    // Step 5: Find clusters
    auto clusters = FindClusters(failed_nodes);
    
    auto end = Clock::now();
    float elapsed = std::chrono::duration<float, std::milli>(
        end - start).count();
    
    // Build result
    AnalysisResult result;
    result.structure_failed = !clusters.empty();
    result.failed_clusters = clusters;
    result.calculation_time_ms = elapsed;
    result.debug_info = FormatDebugInfo(converged, failed_nodes.size());
    
    // Check time budget
    if (elapsed > time_budget_ms) {
        std::cerr << "WARNING: Analysis took " << elapsed << "ms "
                  << "(budget: " << time_budget_ms << "ms)\n";
    }
    
    // Cleanup
    for (auto* node : nodes) {
        delete node;
    }
    nodes.clear();
    
    return result;
}

□ Add debug info formatting

std::string FormatDebugInfo(bool converged, int failed_count) {
    std::stringstream ss;
    ss << "Nodes: " << nodes.size() << "\n";
    ss << "Converged: " << (converged ? "YES" : "NO") << "\n";
    ss << "Failed nodes: " << failed_count << "\n";
    return ss.str();
}
```

**Afternoon (4 hours):**
```cpp
□ Integration testing with interactive demo

void TestInteractive() {
    VoxelWorld world;
    StructuralAnalyzer analyzer;
    
    // Load test scene
    CreateTestBuilding(world, 10, 10, 5);
    
    while (Running()) {
        // Handle input
        if (MouseClicked()) {
            Vector3 clicked = GetClickedVoxel();
            
            if (world.HasVoxel(clicked)) {
                // Remove voxel
                world.RemoveVoxel(clicked);
                
                // Analyze structure
                auto result = analyzer.Analyze(world, {clicked}, 500);
                
                // Visualize results
                if (result.structure_failed) {
                    std::cout << "COLLAPSE DETECTED!\n";
                    std::cout << result.debug_info;
                    
                    VisualizeClusters(result.failed_clusters);
                } else {
                    std::cout << "Structure stable\n";
                }
            }
        }
        
        Render();
    }
}

□ Create test scenarios document

# Track 1 Test Scenarios

## Scenario 1: Tower Collapse
- Structure: 10-voxel tower
- Damage: Remove base
- Expected: All 9 voxels above collapse
- Status: PASS

## Scenario 2: Bridge with Redundancy
- Structure: Bridge with 2 supports
- Damage: Remove 1 support
- Expected: Bridge stays up (other support holds)
- Status: PASS

## Scenario 3: Wall Partial Damage
- Structure: 10×10 wall
- Damage: Remove 5 random voxels
- Expected: Wall stays up (redundant structure)
- Status: PASS

## Scenario 4: Building Corner Column
- Structure: 5×5×10 building
- Damage: Remove corner column
- Expected: Section above column collapses
- Status: PENDING

□ Performance benchmarking

void BenchmarkAnalyzer() {
    struct Benchmark {
        std::string name;
        int voxel_count;
        float avg_time_ms;
    };
    
    std::vector<Benchmark> benchmarks;
    
    // Small structure
    {
        VoxelWorld world;
        CreateTestTower(world, 10);
        
        float total = 0;
        for (int i = 0; i < 10; i++) {
            auto start = Clock::now();
            analyzer.Analyze(world, {Vector3(0,0,0)});
            total += GetElapsedMS(start);
        }
        
        benchmarks.push_back({"Tower (10 voxels)", 10, total / 10});
    }
    
    // Medium structure
    {
        VoxelWorld world;
        CreateTestBuilding(world, 20, 20, 10);
        
        float total = 0;
        for (int i = 0; i < 10; i++) {
            auto start = Clock::now();
            analyzer.Analyze(world, {Vector3(0.5, 0, 0.5)});
            total += GetElapsedMS(start);
        }
        
        benchmarks.push_back({"Building (4000 voxels)", 4000, total / 10});
    }
    
    // Print results
    std::cout << "Performance Benchmarks:\n";
    for (auto& b : benchmarks) {
        std::cout << b.name << ": " << b.avg_time_ms << "ms\n";
        
        if (b.avg_time_ms > 500) {
            std::cout << "  WARNING: Exceeds 500ms budget!\n";
        }
    }
}
```

**Success Criteria:**
- ✅ Complete Analyze() method works end-to-end
- ✅ Interactive demo functional
- ✅ All test scenarios pass
- ✅ Performance within budget (< 500ms)

---

### Week 3 Deliverable

**What you should have:**

1. **Working Structural Analyzer:**
   - Displacement-based spring system
   - Mass calculation via raycasting
   - Iterative solver with convergence
   - Failure detection (displacement + connectivity)
   - Cluster generation

2. **Performance:**
   - Small structures (< 100 voxels): < 50ms
   - Medium structures (< 5K voxels): < 200ms
   - Large structures (< 50K voxels): < 500ms

3. **Test Coverage:**
   - 5+ integration tests
   - All passing
   - Performance benchmarks documented

4. **Interactive Demo:**
   - Click to remove voxels
   - See structural analysis results
   - Visual feedback on failures
   - Debug info displayed

**Demo video (1 minute):**
- Show tower collapse detection
- Show bridge with redundancy
- Show wall partial damage (stable)
- Show performance stats

**Review & Decision:**
- ✅ Track 1 working? → Proceed to Week 4
- ❌ Issues? → Fix before Track 3

---

## Week 4: Structural Analysis - Refinement

### Goals
- Optimize performance for large structures
- Add parameter tuning system
- Handle edge cases
- Prepare for Track 3 integration

### Day 16: Performance Optimization

**Morning (4 hours):**
```cpp
□ Implement surface-only optimization

void BuildNodeGraphOptimized(std::vector<Vector3>& damaged) {
    // BEFORE: Check all voxels
    // AFTER: Only check surface voxels (huge speedup)
    
    auto surface_voxels = world->GetSurfaceVoxels();
    
    // Filter to only surface voxels near damage
    std::unordered_set<Vector3> relevant_surface;
    
    const float INFLUENCE_RADIUS = 5.0f;  // 5 meters
    
    for (auto& surf_pos : surface_voxels) {
        for (auto& damage_pos : damaged) {
            if (Distance(surf_pos, damage_pos) < INFLUENCE_RADIUS) {
                relevant_surface.insert(surf_pos);
                break;
            }
        }
    }
    
    // Create nodes only for relevant surface voxels
    // ... rest of implementation
    
    std::cout << "Optimized: " << relevant_surface.size() 
              << " nodes (was " << surface_voxels.size() << ")\n";
}

□ Benchmark improvement

TEST(Performance, SurfaceOptimization) {
    VoxelWorld world;
    CreateLargeBuilding(world, 50, 50, 20);  // 50K voxels
    
    StructuralAnalyzer analyzer;
    
    // Without optimization
    auto start1 = Clock::now();
    analyzer.BuildNodeGraphSlow(damaged);
    auto time_slow = GetElapsedMS(start1);
    
    // With optimization
    auto start2 = Clock::now();
    analyzer.BuildNodeGraphOptimized(damaged);
    auto time_fast = GetElapsedMS(start2);
    
    std::cout << "Speedup: " << (time_slow / time_fast) << "x\n";
    ASSERT_GT(time_slow / time_fast, 5.0f);  // At least 5x faster
}
```

**Afternoon (4 hours):**
```cpp
□ Parallel computation for mass calculation

#include <thread>
#include <future>

void CalculateMassSupportedParallel() {
    const int num_threads = std::thread::hardware_concurrency();
    std::vector<std::future<void>> futures;
    
    int nodes_per_thread = nodes.size() / num_threads;
    
    for (int t = 0; t < num_threads; t++) {
        int start = t * nodes_per_thread;
        int end = (t == num_threads - 1) ? nodes.size() 
                                          : (t + 1) * nodes_per_thread;
        
        futures.push_back(std::async(std::launch::async, [=]() {
            for (int i = start; i < end; i++) {
                auto* node = nodes[i];
                
                if (!node->is_ground_anchor) {
                    node->mass_supported = RaycastMassAbove(node->position);
                }
            }
        }));
    }
    
    // Wait for all threads
    for (auto& future : futures) {
        future.wait();
    }
}

□ Early termination optimization

bool SolveDisplacementsOptimized() {
    for (int iter = 0; iter < max_iterations; iter++) {
        float max_change = UpdateDisplacements();
        
        // Early termination: Converged
        if (max_change < convergence_threshold) {
            return true;
        }
        
        // Early termination: Obvious failure (diverging)
        if (max_change > 10.0f) {  // 10 meters = clearly failing
            std::cout << "Early failure detection at iteration " << iter << "\n";
            return false;  // Structure failing
        }
        
        // Early termination: Oscillating (stuck)
        if (iter > 20 && abs(max_change - prev_max_change) < 0.00001f) {
            std::cout << "Oscillation detected, adjusting timestep\n";
            timestep *= 0.5;  // Smaller timestep may help
        }
        
        prev_max_change = max_change;
    }
    
    return false;  // Did not converge
}
```

**Success Criteria:**
- ✅ 5x+ speedup from surface-only optimization
- ✅ 2x+ speedup from parallel mass calculation
- ✅ Early termination catches obvious failures
- ✅ Overall < 500ms for 50K voxel structures

---

### Day 17: Parameter Tuning System

**Morning (4 hours):**
```cpp
□ Create tunable parameters structure

struct StructuralParameters {
    // Per-material spring constants
    std::map<std::string, float> spring_constants = {
        {"wood",     10000.0f},
        {"brick",    50000.0f},
        {"concrete", 100000.0f}
    };
    
    // Per-material max displacements
    std::map<std::string, float> max_displacements = {
        {"wood",     0.020f},  // 2cm
        {"brick",    0.010f},  // 1cm
        {"concrete", 0.005f}   // 0.5cm
    };
    
    // Solver parameters
    float timestep = 0.01f;
    float damping = 0.9f;
    int max_iterations = 100;
    float convergence_threshold = 0.0001f;
    
    // Analysis parameters
    float influence_radius = 5.0f;  // How far from damage to analyze
    
    // Save/load
    void SaveToFile(const std::string& filename);
    void LoadFromFile(const std::string& filename);
};

□ Implement parameter loading

void LoadParameters(const std::string& filename) {
    std::ifstream file(filename);
    
    // Simple INI-style format
    // [Materials]
    // wood_spring_constant = 10000
    // wood_max_displacement = 0.02
    // ...
    
    std::string line;
    while (std::getline(file, line)) {
        // Parse line
        auto tokens = Split(line, '=');
        if (tokens.size() == 2) {
            std::string key = Trim(tokens[0]);
            float value = std::stof(Trim(tokens[1]));
            
            parameters.Set(key, value);
        }
    }
}

□ Add runtime parameter adjustment

void UpdateParametersUI() {
    ImGui::Begin("Structural Parameters");
    
    if (ImGui::TreeNode("Materials")) {
        for (auto& [name, k] : params.spring_constants) {
            ImGui::SliderFloat((name + " spring constant").c_str(), 
                              &k, 1000, 500000);
            
            float& max_disp = params.max_displacements[name];
            ImGui::SliderFloat((name + " max displacement").c_str(),
                              &max_disp, 0.001, 0.05);
        }
        ImGui::TreePop();
    }
    
    if (ImGui::TreeNode("Solver")) {
        ImGui::SliderFloat("Timestep", &params.timestep, 0.001, 0.1);
        ImGui::SliderFloat("Damping", &params.damping, 0.5, 0.99);
        ImGui::SliderInt("Max iterations", &params.max_iterations, 10, 500);
        ImGui::TreePop();
    }
    
    if (ImGui::Button("Save Parameters")) {
        params.SaveToFile("structural_params.ini");
    }
    
    if (ImGui::Button("Load Parameters")) {
        params.LoadFromFile("structural_params.ini");
    }
    
    ImGui::End();
}
```

**Afternoon (4 hours):**
```cpp
□ Parameter sensitivity analysis

void AnalyzeParameterSensitivity() {
    struct TestCase {
        std::string name;
        std::function<void()> setup;
        bool should_collapse;
    };
    
    std::vector<TestCase> test_cases = {
        {"Tower remove base", []() { /* ... */ }, true},
        {"Wall remove middle", []() { /* ... */ }, false},
        {"Bridge remove support", []() { /* ... */ }, true},
        // ... more cases
    };
    
    // Test different spring constant values
    for (float k_multiplier = 0.5; k_multiplier <= 2.0; k_multiplier += 0.5) {
        std::cout << "\n=== Spring constant × " << k_multiplier << " ===\n";
        
        params.spring_constants["concrete"] = 100000 * k_multiplier;
        
        int correct = 0;
        for (auto& test : test_cases) {
            bool result = RunTest(test);
            if (result == test.should_collapse) {
                correct++;
            }
        }
        
        float accuracy = correct / (float)test_cases.size();
        std::cout << "Accuracy: " << (accuracy * 100) << "%\n";
    }
}

□ Auto-tuning system

void AutoTuneParameters() {
    // Genetic algorithm or grid search to find best parameters
    
    std::vector<float> k_values = {50000, 100000, 150000, 200000};
    std::vector<float> disp_values = {0.005, 0.010, 0.015, 0.020};
    
    float best_accuracy = 0;
    StructuralParameters best_params;
    
    for (float k : k_values) {
        for (float disp : disp_values) {
            params.spring_constants["concrete"] = k;
            params.max_displacements["concrete"] = disp;
            
            float accuracy = TestAllScenarios();
            
            if (accuracy > best_accuracy) {
                best_accuracy = accuracy;
                best_params = params;
            }
        }
    }
    
    std::cout << "Best accuracy: " << (best_accuracy * 100) << "%\n";
    params = best_params;
    params.SaveToFile("optimal_params.ini");
}
```

**Success Criteria:**
- ✅ Parameters can be saved/loaded from file
- ✅ UI allows runtime parameter adjustment
- ✅ Sensitivity analysis shows parameter impact
- ✅ Auto-tuning finds reasonable defaults

---

### Day 18: Edge Cases

**Morning (4 hours):**
```cpp
□ Handle very small clusters (< 5 voxels)

std::vector<VoxelCluster> FindClusters(std::vector<SpringNode*>& failed) {
    auto clusters = FloodFillClusters(failed);
    
    // Filter out tiny clusters (probably numerical noise)
    const int MIN_CLUSTER_SIZE = 5;
    
    std::vector<VoxelCluster> filtered;
    for (auto& cluster : clusters) {
        if (cluster.voxel_positions.size() >= MIN_CLUSTER_SIZE) {
            filtered.push_back(cluster);
        } else {
            std::cout << "Ignoring tiny cluster: " 
                      << cluster.voxel_positions.size() << " voxels\n";
        }
    }
    
    return filtered;
}

□ Handle disconnected structures (multiple buildings)

void AnalyzeMultipleStructures(std::vector<Vector3>& damaged) {
    // Find all independent structures first
    auto all_clusters = world->FindAllClusters();
    
    // Only analyze structures near damage
    std::vector<VoxelCluster> relevant;
    
    for (auto& cluster : all_clusters) {
        for (auto& damage_pos : damaged) {
            if (Distance(cluster.center_of_mass, damage_pos) < influence_radius) {
                relevant.push_back(cluster);
                break;
            }
        }
    }
    
    // Analyze each structure independently
    std::vector<VoxelCluster> failed_total;
    
    for (auto& structure : relevant) {
        auto failed = AnalyzeSingleStructure(structure, damaged);
        failed_total.insert(failed_total.end(), failed.begin(), failed.end());
    }
    
    return failed_total;
}

□ Handle numerical instability

bool SolveDisplacementsRobust() {
    // Detect numerical instability
    for (int iter = 0; iter < max_iterations; iter++) {
        UpdateDisplacements();
        
        // Check for NaN or infinite values
        bool has_nan = false;
        for (auto* node : nodes) {
            if (std::isnan(node->displacement) || 
                std::isinf(node->displacement)) {
                has_nan = true;
                break;
            }
        }
        
        if (has_nan) {
            std::cerr << "ERROR: Numerical instability detected!\n";
            std::cerr << "  Iteration: " << iter << "\n";
            std::cerr << "  Reducing timestep and retrying...\n";
            
            // Reset and retry with smaller timestep
            ResetDisplacements();
            timestep *= 0.5;
            continue;
        }
        
        // ... rest of solver
    }
}
```

**Afternoon (4 hours):**
```cpp
□ Edge case tests

TEST(EdgeCases, TinyCluster) {
    VoxelWorld world;
    
    // Single floating voxel
    world.SetVoxel(Vector3(0, 10, 0), Voxel{BRICK});
    
    StructuralAnalyzer analyzer;
    auto result = analyzer.Analyze(world, {}, 500);
    
    // Should be filtered out as too small
    ASSERT_TRUE(result.failed_clusters.empty());
}

TEST(EdgeCases, DisconnectedBuildings) {
    VoxelWorld world;
    
    // Building 1 at (0,0,0)
    CreateTestBuilding(world, 10, 10, 10);
    
    // Building 2 at (100,0,0) - far away
    CreateTestBuilding(world, 10, 10, 10, Vector3(100, 0, 0));
    
    // Damage building 1
    std::vector<Vector3> damaged = {Vector3(0, 0, 0)};
    world.RemoveVoxel(damaged[0]);
    
    StructuralAnalyzer analyzer;
    auto result = analyzer.Analyze(world, damaged, 500);
    
    // Only building 1 should be analyzed (building 2 too far)
    // Check that analysis was fast (didn't check building 2)
    ASSERT_LT(result.calculation_time_ms, 300);
}

TEST(EdgeCases, NumericalStability) {
    VoxelWorld world;
    
    // Create structure with extreme mass difference
    world.SetVoxel(Vector3(0, 0, 0), Voxel{WOOD});  // Light
    
    for (int y = 1; y < 100; y++) {
        world.SetVoxel(Vector3(0, y * 0.05, 0), Voxel{CONCRETE});  // Heavy
    }
    
    StructuralAnalyzer analyzer;
    
    // Should handle without NaN
    ASSERT_NO_THROW({
        auto result = analyzer.Analyze(world, {Vector3(0, 0, 0)}, 500);
        
        // Check no NaN in results
        for (auto& cluster : result.failed_clusters) {
            ASSERT_FALSE(std::isnan(cluster.center_of_mass.x));
            ASSERT_FALSE(std::isnan(cluster.total_mass));
        }
    });
}

□ Document edge cases

# Structural Analysis Edge Cases

## 1. Tiny Clusters
**Issue:** Single voxel floating
**Solution:** Filter clusters < 5 voxels
**Test:** EdgeCases/TinyCluster

## 2. Disconnected Structures  
**Issue:** Multiple buildings on map
**Solution:** Only analyze near damage (influence radius)
**Test:** EdgeCases/DisconnectedBuildings

## 3. Numerical Instability
**Issue:** Extreme mass ratios cause NaN
**Solution:** Detect NaN, reduce timestep, retry
**Test:** EdgeCases/NumericalStability

## 4. Ground Definition
**Issue:** What counts as "ground"?
**Solution:** y <= ground_level + voxel_size/2
**Test:** TBD

## 5. Slow Convergence
**Issue:** Some structures take 100+ iterations
**Solution:** Early termination at 100 iterations
**Test:** Performance/ConvergenceSpeed
```

**Success Criteria:**
- ✅ Tiny clusters filtered
- ✅ Disconnected structures handled
- ✅ Numerical instability caught and recovered
- ✅ All edge case tests pass

---

### Day 19-20: Integration Preparation & Documentation

**Day 19 Morning:**
```cpp
□ Create clean interface for Track 3

class StructuralAnalyzer {
public:
    // Main API for Track 3
    AnalysisResult Analyze(VoxelWorld& world,
                          std::vector<Vector3> damaged_positions,
                          float time_budget_ms = 500);
    
    // For debugging
    void EnableVisualization(bool enable);
    void SetParameters(const StructuralParameters& params);
    
    // Statistics
    struct Statistics {
        int total_analyses;
        float avg_time_ms;
        float max_time_ms;
        int total_failures_detected;
    };
    Statistics GetStatistics();
};

struct AnalysisResult {
    bool structure_failed;
    std::vector<VoxelCluster> failed_clusters;  // Ready for Track 3
    float calculation_time_ms;
    std::string debug_info;
};

struct VoxelCluster {
    std::vector<Vector3> voxel_positions;  // Which voxels fell
    Vector3 center_of_mass;                // For physics body position
    float total_mass;                      // For physics body mass
    BoundingBox bounds;                    // For collision shape
};
```

**Day 19 Afternoon:**
```cpp
□ Write API documentation

/**
 * Track 1: Structural Integrity Analyzer
 * 
 * Purpose: Determines which voxels should fall when structure is damaged
 * 
 * Usage:
 *   StructuralAnalyzer analyzer;
 *   
 *   // When voxels are destroyed
 *   std::vector<Vector3> damaged = {pos1, pos2, ...};
 *   auto result = analyzer.Analyze(world, damaged);
 *   
 *   if (result.structure_failed) {
 *       for (auto& cluster : result.failed_clusters) {
 *           // Pass to Track 3 (Physics)
 *           CreateRigidBody(cluster);
 *       }
 *   }
 * 
 * Performance: < 500ms for typical structures (< 50K voxels)
 * 
 * Parameters: See structural_params.ini for tunable values
 */

□ Create examples

// Example 1: Simple tower collapse
void Example_TowerCollapse() {
    VoxelWorld world;
    StructuralAnalyzer analyzer;
    
    // Create tower
    for (int y = 0; y < 10; y++) {
        world.SetVoxel(Vector3(0, y * 0.05, 0), Voxel{BRICK});
    }
    
    // Remove base
    world.RemoveVoxel(Vector3(0, 0, 0));
    
    // Analyze
    auto result = analyzer.Analyze(world, {Vector3(0, 0, 0)});
    
    // Result: 1 cluster with 9 voxels
    assert(result.failed_clusters.size() == 1);
    assert(result.failed_clusters[0].voxel_positions.size() == 9);
}

// Example 2: Building with redundancy
void Example_RedundantStructure() {
    VoxelWorld world;
    StructuralAnalyzer analyzer;
    
    // Create building with 4 corner columns
    // ... (construction code)
    
    // Remove 1 column
    // ... (removal code)
    
    // Analyze
    auto result = analyzer.Analyze(world, damaged);
    
    // Result: Structure stays up (3 columns sufficient)
    assert(!result.structure_failed);
}
```

**Day 20 - Full Documentation:**
```cpp
□ Write README for Track 1

# Track 1: Structural Integrity Analysis

## Overview
Determines which voxels should fall when a structure is damaged.

## Algorithm
Displacement-based spring system:
1. Build graph of surface voxels
2. Calculate mass supported by each voxel
3. Iteratively solve spring equations
4. Detect failures (excessive displacement or no ground connection)
5. Find connected clusters of failed voxels

## Performance
- Small structures (< 1K voxels): < 50ms
- Medium structures (< 10K voxels): < 200ms
- Large structures (< 50K voxels): < 500ms

## Files
- `structural_analyzer.h/cpp` - Main analyzer
- `spring_node.h` - Node data structure
- `structural_parameters.h` - Tunable parameters
- `structural_params.ini` - Default parameters

## Dependencies
- VoxelWorld (Week 1-2)
- Math utilities (Vector3, etc.)

## Testing
Run: `./tests --gtest_filter=Structural*`

## Integration with Track 3
See `examples/track1_to_track3.cpp`

□ Create integration example

// examples/track1_to_track3.cpp

void IntegrateTrack1And3() {
    VoxelWorld world;
    StructuralAnalyzer structural_analyzer;
    // PhysicsSystem physics_system;  // Track 3 (Week 5-6)
    
    // When player destroys voxels
    std::vector<Vector3> damaged = GetDamagedVoxels();
    
    // Track 1: Analyze structure
    auto structural_result = structural_analyzer.Analyze(world, damaged, 500);
    
    if (structural_result.structure_failed) {
        // Track 3: Create physics objects (Week 5-6)
        for (auto& cluster : structural_result.failed_clusters) {
            // TO BE IMPLEMENTED IN WEEK 5:
            // PxRigidDynamic* body = CreateRigidBody(cluster);
            // physics_system.AddDebris(body);
            
            // For now, just visualize
            VisualizeCluster(cluster, Color::Red);
        }
    }
}
```

**Success Criteria:**
- ✅ Clean API documented
- ✅ Examples provided
- ✅ README written
- ✅ Integration points identified
- ✅ Ready for Track 3 (Week 5)

---

### Week 4 Deliverable

**What you should have:**

1. **Optimized Structural Analyzer:**
   - Surface-only optimization (5x speedup)
   - Parallel mass calculation (2x speedup)
   - Early termination for obvious cases
   - Total: < 500ms for 50K voxels

2. **Parameter System:**
   - Tunable parameters (spring constants, displacements, solver settings)
   - Save/load from file
   - Runtime adjustment UI
   - Sensitivity analysis tools

3. **Edge Case Handling:**
   - Tiny cluster filtering
   - Disconnected structure support
   - Numerical stability checks
   - All edge case tests passing

4. **Documentation:**
   - Complete API documentation
   - Usage examples
   - Integration guide for Track 3
   - Parameter tuning guide

5. **Statistics:**
   - 15+ unit tests
   - 5+ integration tests
   - Performance benchmarks documented
   - Code coverage > 80%

**Demo video (2 minutes):**
- Show parameter adjustment UI
- Show before/after optimization (performance)
- Show complex structure (building with columns)
- Show edge cases handled correctly
- Show statistics and debug info

**Review & Decision:**
- ✅ Track 1 production-ready? → Proceed to Week 5 (Track 3)
- ✅ Performance acceptable? → Continue
- ❌ Issues remaining? → Address before Week 5

---

## Weeks 5-6: Physics Integration (Track 3)

### Week 5, Day 21: PhysX Setup

**Morning (4 hours):**
```cpp
□ Install PhysX SDK
  - Download from: https://github.com/NVIDIAGameWorks/PhysX
  - Or use package manager (vcpkg, conan)
  
□ Add to project
  - CMakeLists.txt:
    find_package(PhysX REQUIRED)
    target_link_libraries(YourProject PhysX::PhysX)

□ Initialize PhysX

#include <PxPhysicsAPI.h>
using namespace physx;

class PhysicsSystem {
private:
    PxFoundation* foundation;
    PxPhysics* physics;
    PxScene* scene;
    PxCooking* cooking;
    PxDefaultCpuDispatcher* dispatcher;
    PxDefaultErrorCallback error_callback;
    PxDefaultAllocator allocator;
    
public:
    void Initialize() {
        // 1. Foundation
        foundation = PxCreateFoundation(PX_PHYSICS_VERSION,
                                       allocator,
                                       error_callback);
        
        // 2. Physics
        PxTolerancesScale scale;
        physics = PxCreatePhysics(PX_PHYSICS_VERSION,
                                 *foundation,
                                 scale);
        
        // 3. Cooking (for convex meshes)
        cooking = PxCreateCooking(PX_PHYSICS_VERSION,
                                 *foundation,
                                 PxCookingParams(scale));
        
        // 4. Scene
        PxSceneDesc sceneDesc(scale);
        sceneDesc.gravity = PxVec3(0, -9.81f, 0);
        
        dispatcher = PxDefaultCpuDispatcherCreate(4);  // 4 threads
        sceneDesc.cpuDispatcher = dispatcher;
        sceneDesc.filterShader = PxDefaultSimulationFilterShader;
        
        scene = physics->createScene(sceneDesc);
    }
    
    void Shutdown() {
        scene->release();
        dispatcher->release();
        cooking->release();
        physics->release();
        foundation->release();
    }
};

□ Test basic PhysX

TEST(Physics, Initialization) {
    PhysicsSystem physics;
    physics.Initialize();
    
    // Create simple box
    PxRigidDynamic* box = CreateBox(Vector3(0, 10, 0), 1.0f);
    ASSERT_NE(box, nullptr);
    
    physics.Shutdown();
}
```

**Afternoon (4 hours):**
```cpp
□ Create ground plane

void CreateGroundCollision() {
    PxRigidStatic* ground = physics->createRigidStatic(
        PxTransform(PxVec3(0, 0, 0))
    );
    
    PxMaterial* ground_mat = physics->createMaterial(0.9f, 0.8f, 0.1f);
    
    PxShape* plane = physics->createShape(
        PxPlaneGeometry(),
        *ground_mat
    );
    
    ground->attachShape(*plane);
    scene->addActor(*ground);
}

□ Create simple rigid body

PxRigidDynamic* CreateBox(Vector3 position, float size) {
    PxTransform transform(ToPxVec3(position));
    
    PxRigidDynamic* body = physics->createRigidDynamic(transform);
    
    PxMaterial* material = physics->createMaterial(0.5f, 0.5f, 0.3f);
    
    PxShape* shape = physics->createShape(
        PxBoxGeometry(size/2, size/2, size/2),
        *material
    );
    
    body->attachShape(*shape);
    PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
    
    scene->addActor(*body);
    
    return body;
}

□ Run simulation

void Simulate(float duration) {
    const float timestep = 1.0f / 60.0f;
    int steps = (int)(duration / timestep);
    
    for (int i = 0; i < steps; i++) {
        scene->simulate(timestep);
        scene->fetchResults(true);
        
        // Render current state
        RenderPhysics();
    }
}

□ Test falling box

TEST(Physics, FallingBox) {
    PhysicsSystem physics;
    physics.Initialize();
    physics.CreateGroundCollision();
    
    PxRigidDynamic* box = physics.CreateBox(Vector3(0, 10, 0), 1.0f);
    
    // Simulate for 2 seconds
    physics.Simulate(2.0f);
    
    // Box should be on ground
    PxVec3 final_pos = box->getGlobalPose().p;
    ASSERT_LT(final_pos.y, 1.0f);  // Near ground
    
    physics.Shutdown();
}
```

**Success Criteria:**
- ✅ PhysX compiles and links
- ✅ Can create rigid bodies
- ✅ Can simulate falling
- ✅ Ground collision works

---

(Continue with remaining days 22-30...)

Due to length constraints, I'll create the complete document as a file.
