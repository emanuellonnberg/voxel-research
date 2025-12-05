# 8-Week Sprint Plan: Weeks 5-8 (Physics Integration & Final Demo)

**Continuation of 8-Week Sprint Plan**  
**Focus:** Track 3 (Physics) Integration with Bullet Physics Library  
**Prerequisites:** Weeks 1-4 completed (Voxel system + Track 1 structural analysis)

---

## Overview: Weeks 5-8

### What We're Building

**Week 5: Bullet Physics Foundation**
- Set up Bullet Physics library
- Create basic rigid body system
- Implement voxel-to-collision-shape conversion
- Test simple falling debris

**Week 6: Debris Simulation System**
- Implement debris lifecycle (active → settling → static)
- Add material-specific physics properties
- Implement settling detection
- Convert settled debris to rubble

**Week 7: Complete Integration**
- Connect Track 1 (Structural) → Track 3 (Physics)
- Implement full destruction pipeline
- Add visual effects (particles, dust)
- Optimize performance

**Week 8: Polish & Final Demo**
- Final optimization pass
- Add audio integration
- Create demo scenarios
- Record showcase video
- Documentation & handoff

---

## Week 5: Bullet Physics Foundation

### Goals
- Set up Bullet Physics library
- Understand Bullet API differences from PhysX
- Create basic rigid body simulation
- Convert VoxelCluster to Bullet collision shapes

### Day 21: Bullet Setup & Initialization

**Morning (4 hours):**

```cpp
□ Install Bullet Physics

// Option 1: vcpkg
vcpkg install bullet3

// Option 2: Build from source
git clone https://github.com/bulletphysics/bullet3.git
cd bullet3
mkdir build && cd build
cmake .. -DBUILD_SHARED_LIBS=ON
make -j8

// Option 3: Package manager (Ubuntu/Debian)
sudo apt-get install libbullet-dev

□ Add to CMakeLists.txt

find_package(Bullet REQUIRED)

target_link_libraries(YourProject
    BulletDynamics
    BulletCollision
    LinearMath
)

include_directories(${BULLET_INCLUDE_DIRS})

□ Test compilation

#include <btBulletDynamicsCommon.h>

TEST(Bullet, LinksCorrectly) {
    btVector3 test(1, 2, 3);
    ASSERT_FLOAT_EQ(test.x(), 1.0f);
    ASSERT_FLOAT_EQ(test.y(), 2.0f);
    ASSERT_FLOAT_EQ(test.z(), 3.0f);
}
```

**Afternoon (4 hours):**

```cpp
□ Create BulletPhysicsSystem class

#include <btBulletDynamicsCommon.h>

class BulletPhysicsSystem {
private:
    // Core Bullet components
    btDefaultCollisionConfiguration* collision_config;
    btCollisionDispatcher* dispatcher;
    btBroadphaseInterface* broadphase;
    btSequentialImpulseConstraintSolver* solver;
    btDiscreteDynamicsWorld* dynamics_world;
    
    // Ground plane
    btRigidBody* ground_body;
    
public:
    void Initialize();
    void Shutdown();
    void Step(float deltaTime);
    
    btDiscreteDynamicsWorld* GetWorld() { return dynamics_world; }
};

void BulletPhysicsSystem::Initialize() {
    // 1. Collision configuration
    collision_config = new btDefaultCollisionConfiguration();
    
    // 2. Dispatcher
    dispatcher = new btCollisionDispatcher(collision_config);
    
    // 3. Broadphase (for collision detection optimization)
    broadphase = new btDbvtBroadphase();
    
    // 4. Constraint solver
    solver = new btSequentialImpulseConstraintSolver();
    
    // 5. Dynamics world (the main physics simulation)
    dynamics_world = new btDiscreteDynamicsWorld(
        dispatcher,
        broadphase,
        solver,
        collision_config
    );
    
    // Set gravity
    dynamics_world->setGravity(btVector3(0, -9.81f, 0));
    
    std::cout << "Bullet Physics initialized\n";
}

void BulletPhysicsSystem::Shutdown() {
    // Delete in reverse order
    delete dynamics_world;
    delete solver;
    delete broadphase;
    delete dispatcher;
    delete collision_config;
}

void BulletPhysicsSystem::Step(float deltaTime) {
    // Step simulation
    // 2nd parameter: max substeps (for stability)
    // 3rd parameter: fixed timestep
    dynamics_world->stepSimulation(deltaTime, 10, 1.0f/60.0f);
}

□ Write initialization test

TEST(Bullet, Initialization) {
    BulletPhysicsSystem physics;
    physics.Initialize();
    
    ASSERT_NE(physics.GetWorld(), nullptr);
    
    // Check gravity
    btVector3 gravity = physics.GetWorld()->getGravity();
    ASSERT_FLOAT_EQ(gravity.y(), -9.81f);
    
    physics.Shutdown();
}

□ Create utility conversion functions

// Convert between our Vector3 and Bullet's btVector3
inline btVector3 ToBtVector3(const Vector3& v) {
    return btVector3(v.x, v.y, v.z);
}

inline Vector3 ToVector3(const btVector3& v) {
    return Vector3(v.x(), v.y(), v.z());
}

// Convert transform
inline btTransform ToBtTransform(const Vector3& position, 
                                 const Quaternion& rotation) {
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(ToBtVector3(position));
    transform.setRotation(btQuaternion(rotation.x, rotation.y, 
                                       rotation.z, rotation.w));
    return transform;
}
```

**Success Criteria:**
- ✅ Bullet compiles and links correctly
- ✅ Can create dynamics world
- ✅ Gravity is set correctly
- ✅ Conversion utilities work

---

### Day 22: Ground Collision & Simple Bodies

**Morning (4 hours):**

```cpp
□ Create ground plane

void BulletPhysicsSystem::CreateGroundPlane() {
    // Create ground shape (infinite plane at y=0)
    btCollisionShape* ground_shape = new btStaticPlaneShape(
        btVector3(0, 1, 0),  // Normal (pointing up)
        0                     // Plane constant (y = 0)
    );
    
    // Ground is at origin
    btTransform ground_transform;
    ground_transform.setIdentity();
    ground_transform.setOrigin(btVector3(0, 0, 0));
    
    // Create motion state
    btDefaultMotionState* ground_motion = 
        new btDefaultMotionState(ground_transform);
    
    // Rigid body construction info
    btRigidBody::btRigidBodyConstructionInfo ground_info(
        0,              // mass = 0 (static object)
        ground_motion,
        ground_shape,
        btVector3(0, 0, 0)  // local inertia (unused for static)
    );
    
    // Create rigid body
    ground_body = new btRigidBody(ground_info);
    
    // Set friction and restitution (bounciness)
    ground_body->setFriction(0.9f);      // High friction
    ground_body->setRestitution(0.1f);   // Low bounce
    
    // Add to world
    dynamics_world->addRigidBody(ground_body);
    
    std::cout << "Ground plane created\n";
}

□ Create simple box rigid body

btRigidBody* CreateBox(Vector3 position, float size, float mass) {
    // 1. Create collision shape
    btCollisionShape* box_shape = new btBoxShape(
        btVector3(size/2, size/2, size/2)  // Half-extents
    );
    
    // 2. Create transform
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(ToBtVector3(position));
    
    // 3. Create motion state
    btDefaultMotionState* motion_state = 
        new btDefaultMotionState(transform);
    
    // 4. Calculate inertia
    btVector3 local_inertia(0, 0, 0);
    if (mass != 0.0f) {
        box_shape->calculateLocalInertia(mass, local_inertia);
    }
    
    // 5. Create rigid body
    btRigidBody::btRigidBodyConstructionInfo rb_info(
        mass,
        motion_state,
        box_shape,
        local_inertia
    );
    
    btRigidBody* body = new btRigidBody(rb_info);
    
    // Set properties
    body->setFriction(0.6f);
    body->setRestitution(0.3f);
    
    return body;
}
```

**Afternoon (4 hours):**

```cpp
□ Test falling box simulation

TEST(Bullet, FallingBox) {
    BulletPhysicsSystem physics;
    physics.Initialize();
    physics.CreateGroundPlane();
    
    // Create box at height
    btRigidBody* box = CreateBox(Vector3(0, 10, 0), 1.0f, 10.0f);
    physics.GetWorld()->addRigidBody(box);
    
    // Simulate for 2 seconds (120 steps at 60 Hz)
    for (int i = 0; i < 120; i++) {
        physics.Step(1.0f / 60.0f);
    }
    
    // Get final position
    btTransform transform;
    box->getMotionState()->getWorldTransform(transform);
    btVector3 final_pos = transform.getOrigin();
    
    // Should be on ground (y ≈ 0.5 for 1m box)
    ASSERT_LT(final_pos.y(), 1.0f);
    ASSERT_GT(final_pos.y(), 0.4f);
    
    // Clean up
    physics.GetWorld()->removeRigidBody(box);
    delete box->getMotionState();
    delete box->getCollisionShape();
    delete box;
    
    physics.Shutdown();
}

□ Add visual debugging

class BulletDebugDrawer : public btIDebugDraw {
private:
    int debug_mode;
    
public:
    BulletDebugDrawer() : debug_mode(DBG_DrawWireframe) {}
    
    virtual void drawLine(const btVector3& from, 
                         const btVector3& to, 
                         const btVector3& color) override {
        // Draw line in your renderer
        DrawLine(ToVector3(from), ToVector3(to), 
                Color(color.x(), color.y(), color.z()));
    }
    
    virtual void drawContactPoint(const btVector3& point,
                                 const btVector3& normal,
                                 btScalar distance,
                                 int lifeTime,
                                 const btVector3& color) override {
        // Draw contact point
        DrawSphere(ToVector3(point), 0.1f, 
                  Color(color.x(), color.y(), color.z()));
    }
    
    virtual void reportErrorWarning(const char* warning) override {
        std::cerr << "Bullet Warning: " << warning << "\n";
    }
    
    virtual void draw3dText(const btVector3& location,
                          const char* text) override {
        // Optional: Draw text at location
    }
    
    virtual void setDebugMode(int mode) override {
        debug_mode = mode;
    }
    
    virtual int getDebugMode() const override {
        return debug_mode;
    }
};

void BulletPhysicsSystem::EnableDebugDrawing() {
    BulletDebugDrawer* debugDrawer = new BulletDebugDrawer();
    dynamics_world->setDebugDrawer(debugDrawer);
}

void BulletPhysicsSystem::RenderDebug() {
    if (dynamics_world->getDebugDrawer()) {
        dynamics_world->debugDrawWorld();
    }
}

□ Create interactive test

void InteractivePhysicsTest() {
    BulletPhysicsSystem physics;
    physics.Initialize();
    physics.CreateGroundPlane();
    physics.EnableDebugDrawing();
    
    std::vector<btRigidBody*> boxes;
    
    while (Running()) {
        // Spawn box on click
        if (MouseClicked()) {
            Vector3 spawn_pos = camera.position + camera.forward * 5.0f;
            btRigidBody* box = CreateBox(spawn_pos, 1.0f, 10.0f);
            physics.GetWorld()->addRigidBody(box);
            boxes.push_back(box);
        }
        
        // Step physics
        physics.Step(GetDeltaTime());
        
        // Render
        RenderScene();
        physics.RenderDebug();
        
        HandleInput();
    }
    
    physics.Shutdown();
}
```

**Success Criteria:**
- ✅ Ground plane works (objects don't fall through)
- ✅ Box falls and lands on ground
- ✅ Debug visualization shows collision shapes
- ✅ Can spawn multiple boxes interactively

---

### Day 23: Voxel Cluster Conversion

**Morning (4 hours):**

```cpp
□ Create convex hull from voxel cluster

btConvexHullShape* CreateConvexHullFromCluster(VoxelCluster& cluster) {
    btConvexHullShape* shape = new btConvexHullShape();
    
    // Add all voxel corner points to convex hull
    for (auto& voxel_pos : cluster.voxel_positions) {
        // Each voxel has 8 corners
        float vs = voxel_size;
        
        for (int i = 0; i < 8; i++) {
            float dx = (i & 1) ? vs : 0;
            float dy = (i & 2) ? vs : 0;
            float dz = (i & 4) ? vs : 0;
            
            Vector3 corner = voxel_pos + Vector3(dx, dy, dz);
            shape->addPoint(ToBtVector3(corner), false);
        }
    }
    
    // Recalculate local AABB (important!)
    shape->recalcLocalAabb();
    
    // Optimize the hull
    shape->optimizeConvexHull();
    
    return shape;
}

□ Test convex hull creation

TEST(BulletIntegration, ConvexHullFromCluster) {
    VoxelWorld world;
    
    // Create small 3-voxel tower
    VoxelCluster cluster;
    cluster.voxel_positions = {
        Vector3(0, 0, 0),
        Vector3(0, 0.05, 0),
        Vector3(0, 0.10, 0)
    };
    cluster.center_of_mass = Vector3(0, 0.05, 0);
    cluster.total_mass = 0.9f;  // 3 voxels * 0.3kg each
    
    // Create convex hull
    btConvexHullShape* hull = CreateConvexHullFromCluster(cluster);
    
    ASSERT_NE(hull, nullptr);
    ASSERT_GT(hull->getNumPoints(), 0);
    
    delete hull;
}
```

**Afternoon (4 hours):**

```cpp
□ Create rigid body from voxel cluster

btRigidBody* CreateRigidBodyFromCluster(VoxelCluster& cluster,
                                        Material& material) {
    // 1. Create collision shape
    btConvexHullShape* shape = CreateConvexHullFromCluster(cluster);
    
    // 2. Create transform (at center of mass)
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(ToBtVector3(cluster.center_of_mass));
    
    // 3. Create motion state
    btDefaultMotionState* motion_state = 
        new btDefaultMotionState(transform);
    
    // 4. Calculate inertia
    btVector3 local_inertia(0, 0, 0);
    float mass = cluster.total_mass;
    
    if (mass > 0) {
        shape->calculateLocalInertia(mass, local_inertia);
    }
    
    // 5. Create rigid body
    btRigidBody::btRigidBodyConstructionInfo rb_info(
        mass,
        motion_state,
        shape,
        local_inertia
    );
    
    btRigidBody* body = new btRigidBody(rb_info);
    
    // 6. Set material properties
    body->setFriction(GetFriction(material));
    body->setRestitution(GetRestitution(material));
    
    // 7. Initial velocity (falling)
    body->setLinearVelocity(btVector3(0, -1.0f, 0));
    
    return body;
}

float GetFriction(Material& material) {
    if (material.name == "wood")     return 0.6f;
    if (material.name == "brick")    return 0.8f;
    if (material.name == "concrete") return 0.9f;
    if (material.name == "metal")    return 0.3f;
    if (material.name == "glass")    return 0.4f;
    return 0.5f;  // Default
}

float GetRestitution(Material& material) {
    if (material.name == "wood")     return 0.3f;
    if (material.name == "brick")    return 0.15f;
    if (material.name == "concrete") return 0.1f;
    if (material.name == "metal")    return 0.7f;
    if (material.name == "glass")    return 0.5f;
    return 0.2f;  // Default
}

□ Test cluster-to-rigid-body conversion

TEST(BulletIntegration, ClusterToRigidBody) {
    BulletPhysicsSystem physics;
    physics.Initialize();
    physics.CreateGroundPlane();
    
    // Create test cluster
    VoxelCluster cluster = CreateTestCluster();
    Material material = materials["concrete"];
    
    // Convert to rigid body
    btRigidBody* body = CreateRigidBodyFromCluster(cluster, material);
    
    ASSERT_NE(body, nullptr);
    ASSERT_FLOAT_EQ(body->getMass(), cluster.total_mass);
    
    // Add to world and simulate
    physics.GetWorld()->addRigidBody(body);
    
    for (int i = 0; i < 60; i++) {
        physics.Step(1.0f / 60.0f);
    }
    
    // Should have fallen
    btTransform transform;
    body->getMotionState()->getWorldTransform(transform);
    btVector3 pos = transform.getOrigin();
    
    ASSERT_LT(pos.y(), cluster.center_of_mass.y);
    
    physics.Shutdown();
}

□ Handle large clusters (simplification)

btConvexHullShape* CreateSimplifiedHull(VoxelCluster& cluster) {
    // For clusters with many voxels, use simplified approach
    
    if (cluster.voxel_positions.size() > 100) {
        // Use bounding box instead of full convex hull
        BoundingBox bounds = cluster.bounds;
        Vector3 size = bounds.max - bounds.min;
        
        btBoxShape* box = new btBoxShape(
            btVector3(size.x/2, size.y/2, size.z/2)
        );
        
        // Note: Can't directly return btBoxShape as btConvexHullShape
        // So we create a simple convex hull from box corners
        btConvexHullShape* shape = new btConvexHullShape();
        
        for (int i = 0; i < 8; i++) {
            float x = (i & 1) ? bounds.max.x : bounds.min.x;
            float y = (i & 2) ? bounds.max.y : bounds.min.y;
            float z = (i & 4) ? bounds.max.z : bounds.min.z;
            
            shape->addPoint(btVector3(x, y, z), false);
        }
        
        shape->recalcLocalAabb();
        return shape;
    }
    
    // Normal convex hull for smaller clusters
    return CreateConvexHullFromCluster(cluster);
}
```

**Success Criteria:**
- ✅ Can create convex hull from voxel cluster
- ✅ Can create rigid body with correct mass
- ✅ Material properties applied correctly
- ✅ Simplified hull works for large clusters

---

### Day 24: Debris Manager

**Morning (4 hours):**

```cpp
□ Create DebrisObject class

enum DebrisState {
    ACTIVE,    // Simulating
    SETTLING,  // Slowing down
    SETTLED,   // Stopped moving
    REMOVED    // Cleaned up
};

class DebrisObject {
public:
    btRigidBody* rigid_body;
    VoxelCluster original_cluster;
    Material material;
    
    DebrisState state;
    float time_below_threshold;
    
    int id;
    
    DebrisObject(btRigidBody* body, 
                VoxelCluster& cluster,
                Material& mat)
        : rigid_body(body)
        , original_cluster(cluster)
        , material(mat)
        , state(ACTIVE)
        , time_below_threshold(0)
    {
        static int next_id = 0;
        id = next_id++;
    }
    
    void Update(float deltaTime);
    bool IsSettled() const;
    Vector3 GetPosition() const;
    void Cleanup(btDiscreteDynamicsWorld* world);
};

void DebrisObject::Update(float deltaTime) {
    if (state == REMOVED) return;
    
    // Get velocities
    btVector3 linear_vel = rigid_body->getLinearVelocity();
    btVector3 angular_vel = rigid_body->getAngularVelocity();
    
    float linear_speed = linear_vel.length();
    float angular_speed = angular_vel.length();
    
    // Thresholds
    const float LINEAR_THRESHOLD = 0.1f;   // 0.1 m/s
    const float ANGULAR_THRESHOLD = 0.2f;  // 0.2 rad/s
    
    if (linear_speed < LINEAR_THRESHOLD && 
        angular_speed < ANGULAR_THRESHOLD) {
        
        time_below_threshold += deltaTime;
        
        if (time_below_threshold > 1.0f) {  // 1 second
            state = SETTLED;
        } else {
            state = SETTLING;
        }
    } else {
        time_below_threshold = 0;
        state = ACTIVE;
    }
}

bool DebrisObject::IsSettled() const {
    return state == SETTLED;
}

Vector3 DebrisObject::GetPosition() const {
    btTransform transform;
    rigid_body->getMotionState()->getWorldTransform(transform);
    return ToVector3(transform.getOrigin());
}

void DebrisObject::Cleanup(btDiscreteDynamicsWorld* world) {
    if (rigid_body) {
        world->removeRigidBody(rigid_body);
        
        delete rigid_body->getMotionState();
        delete rigid_body->getCollisionShape();
        delete rigid_body;
        
        rigid_body = nullptr;
    }
    
    state = REMOVED;
}
```

**Afternoon (4 hours):**

```cpp
□ Create DebrisManager class

class DebrisManager {
private:
    BulletPhysicsSystem* physics;
    std::vector<DebrisObject*> active_debris;
    
    const int MAX_DEBRIS = 50;
    
public:
    DebrisManager(BulletPhysicsSystem* phys) : physics(phys) {}
    
    void AddDebris(VoxelCluster& cluster, Material& material);
    void Update(float deltaTime);
    void CleanupSettled();
    
    int GetActiveCount() const { return active_debris.size(); }
    std::vector<DebrisObject*>& GetDebris() { return active_debris; }
};

void DebrisManager::AddDebris(VoxelCluster& cluster, Material& material) {
    // Enforce debris limit
    if (active_debris.size() >= MAX_DEBRIS) {
        // Remove oldest debris (force settle)
        auto oldest = active_debris.front();
        std::cout << "Debris limit reached, removing oldest (#" 
                  << oldest->id << ")\n";
        
        oldest->Cleanup(physics->GetWorld());
        delete oldest;
        active_debris.erase(active_debris.begin());
    }
    
    // Create rigid body
    btRigidBody* body = CreateRigidBodyFromCluster(cluster, material);
    
    // Add to physics world
    physics->GetWorld()->addRigidBody(body);
    
    // Create debris object
    DebrisObject* debris = new DebrisObject(body, cluster, material);
    active_debris.push_back(debris);
    
    std::cout << "Added debris #" << debris->id 
              << " (" << cluster.voxel_positions.size() << " voxels, "
              << cluster.total_mass << " kg)\n";
}

void DebrisManager::Update(float deltaTime) {
    for (auto* debris : active_debris) {
        debris->Update(deltaTime);
    }
}

void DebrisManager::CleanupSettled() {
    auto it = active_debris.begin();
    
    while (it != active_debris.end()) {
        DebrisObject* debris = *it;
        
        if (debris->IsSettled()) {
            std::cout << "Debris #" << debris->id << " settled at "
                      << debris->GetPosition() << "\n";
            
            // Convert to rubble (Track 2 integration)
            // rubble_system->AddRubble(debris->GetPosition(), height);
            
            // Cleanup
            debris->Cleanup(physics->GetWorld());
            delete debris;
            
            it = active_debris.erase(it);
        } else {
            ++it;
        }
    }
}

□ Test debris manager

TEST(DebrisManager, BasicLifecycle) {
    BulletPhysicsSystem physics;
    physics.Initialize();
    physics.CreateGroundPlane();
    
    DebrisManager debris_manager(&physics);
    
    // Create test cluster
    VoxelCluster cluster = CreateTestCluster();
    Material material = materials["concrete"];
    
    // Add debris
    debris_manager.AddDebris(cluster, material);
    ASSERT_EQ(debris_manager.GetActiveCount(), 1);
    
    // Simulate until settled
    for (int i = 0; i < 300; i++) {  // 5 seconds
        physics.Step(1.0f / 60.0f);
        debris_manager.Update(1.0f / 60.0f);
        debris_manager.CleanupSettled();
        
        if (debris_manager.GetActiveCount() == 0) {
            std::cout << "Debris settled after " << i << " frames\n";
            break;
        }
    }
    
    ASSERT_EQ(debris_manager.GetActiveCount(), 0);
    
    physics.Shutdown();
}

TEST(DebrisManager, DebrisLimit) {
    BulletPhysicsSystem physics;
    physics.Initialize();
    physics.CreateGroundPlane();
    
    DebrisManager debris_manager(&physics);
    
    // Try to add 60 debris (limit is 50)
    for (int i = 0; i < 60; i++) {
        VoxelCluster cluster = CreateTestCluster();
        Material material = materials["concrete"];
        
        debris_manager.AddDebris(cluster, material);
    }
    
    // Should cap at 50
    ASSERT_EQ(debris_manager.GetActiveCount(), 50);
    
    physics.Shutdown();
}
```

**Success Criteria:**
- ✅ DebrisObject tracks state correctly
- ✅ DebrisManager enforces debris limit
- ✅ Settling detection works
- ✅ Cleanup doesn't crash

---

### Day 25: Material-Specific Behaviors

**Morning (4 hours):**

```cpp
□ Implement fracture pattern generation

std::vector<VoxelCluster> FractureCluster(VoxelCluster& cluster,
                                         Material& material) {
    std::vector<VoxelCluster> fragments;
    
    if (material.name == "wood") {
        // Wood: Splinters (split along grain)
        fragments = SplitIntoSplinters(cluster);
        
    } else if (material.name == "concrete") {
        // Concrete: Irregular chunks (Voronoi-like)
        fragments = SplitIntoChunks(cluster, 5, 10);
        
    } else if (material.name == "brick") {
        // Brick: Masonry units (regular pieces)
        fragments = SplitIntoMasonryUnits(cluster);
        
    } else if (material.name == "glass") {
        // Glass: Many small shards
        fragments = ShatterIntoShards(cluster, 20);
        
    } else {
        // Default: Simple split
        fragments = SplitIntoChunks(cluster, 3, 5);
    }
    
    return fragments;
}

std::vector<VoxelCluster> SplitIntoSplinters(VoxelCluster& cluster) {
    // Wood breaks into long thin pieces
    std::vector<VoxelCluster> splinters;
    
    // Find longest axis
    Vector3 size = cluster.bounds.max - cluster.bounds.min;
    int longest_axis = (size.x > size.y && size.x > size.z) ? 0 :
                       (size.y > size.z) ? 1 : 2;
    
    // Split perpendicular to longest axis
    int num_splinters = RandomRange(3, 6);
    
    for (int i = 0; i < num_splinters; i++) {
        VoxelCluster splinter;
        
        // Each splinter gets a slice of voxels
        for (auto& pos : cluster.voxel_positions) {
            float coord = (longest_axis == 0) ? pos.x :
                         (longest_axis == 1) ? pos.y : pos.z;
            
            float min = cluster.bounds.min[longest_axis];
            float max = cluster.bounds.max[longest_axis];
            float step = (max - min) / num_splinters;
            
            if (coord >= min + i * step && 
                coord < min + (i + 1) * step) {
                splinter.voxel_positions.push_back(pos);
            }
        }
        
        if (!splinter.voxel_positions.empty()) {
            splinter.center_of_mass = CalculateCenterOfMass(splinter);
            splinter.total_mass = CalculateTotalMass(splinter);
            splinter.bounds = CalculateBounds(splinter);
            splinters.push_back(splinter);
        }
    }
    
    return splinters;
}

std::vector<VoxelCluster> SplitIntoChunks(VoxelCluster& cluster,
                                         int min_chunks,
                                         int max_chunks) {
    // Split recursively along random axes
    std::vector<VoxelCluster> chunks = {cluster};
    
    int target_count = RandomRange(min_chunks, max_chunks);
    
    while (chunks.size() < target_count) {
        // Find largest chunk
        auto largest_it = std::max_element(chunks.begin(), chunks.end(),
            [](VoxelCluster& a, VoxelCluster& b) {
                return a.voxel_positions.size() < b.voxel_positions.size();
            });
        
        if (largest_it->voxel_positions.size() < 5) break;  // Too small
        
        // Split it
        auto split_result = SplitClusterInHalf(*largest_it);
        
        // Replace with two halves
        chunks.erase(largest_it);
        chunks.push_back(split_result.first);
        chunks.push_back(split_result.second);
    }
    
    return chunks;
}
```

**Afternoon (4 hours):**

```cpp
□ Add material-specific initial velocities

void SetInitialVelocity(btRigidBody* body, Material& material) {
    if (material.name == "glass") {
        // Glass shards fly outward from impact
        Vector3 direction = RandomDirection();
        btVector3 velocity = ToBtVector3(direction * 3.0f);  // Fast
        body->setLinearVelocity(velocity);
        
        // High spin
        btVector3 angular = ToBtVector3(RandomDirection() * 10.0f);
        body->setAngularVelocity(angular);
        
    } else if (material.name == "wood") {
        // Wood splinters tumble
        btVector3 velocity(0, -1.0f, 0);  // Mostly down
        body->setLinearVelocity(velocity);
        
        // Medium spin
        btVector3 angular = ToBtVector3(RandomDirection() * 5.0f);
        body->setAngularVelocity(angular);
        
    } else {
        // Default: Fall with slight randomness
        btVector3 velocity(
            RandomRange(-0.5f, 0.5f),
            -1.0f,
            RandomRange(-0.5f, 0.5f)
        );
        body->setLinearVelocity(velocity);
    }
}

□ Update CreateRigidBodyFromCluster to use fracture

std::vector<btRigidBody*> CreateDebrisFromCluster(
    VoxelCluster& cluster,
    Material& material,
    btDiscreteDynamicsWorld* world)
{
    std::vector<btRigidBody*> bodies;
    
    // Fracture cluster based on material
    auto fragments = FractureCluster(cluster, material);
    
    // Create rigid body for each fragment
    for (auto& fragment : fragments) {
        btRigidBody* body = CreateRigidBodyFromCluster(fragment, material);
        
        // Set initial velocity
        SetInitialVelocity(body, material);
        
        // Add to world
        world->addRigidBody(body);
        
        bodies.push_back(body);
    }
    
    return bodies;
}

□ Test material-specific behaviors

TEST(MaterialBehaviors, WoodSplinters) {
    VoxelCluster cluster = CreateTestCluster();
    Material wood = materials["wood"];
    
    auto splinters = FractureCluster(cluster, wood);
    
    // Should create 3-6 splinters
    ASSERT_GE(splinters.size(), 3);
    ASSERT_LE(splinters.size(), 6);
    
    // Each splinter should be elongated
    for (auto& splinter : splinters) {
        Vector3 size = splinter.bounds.max - splinter.bounds.min;
        
        // At least one dimension should be 3x longer than another
        float max_dim = std::max({size.x, size.y, size.z});
        float min_dim = std::min({size.x, size.y, size.z});
        
        ASSERT_GT(max_dim / min_dim, 2.0f);
    }
}

TEST(MaterialBehaviors, ConcreteChunks) {
    VoxelCluster cluster = CreateTestCluster();
    Material concrete = materials["concrete"];
    
    auto chunks = FractureCluster(cluster, concrete);
    
    // Should create 5-10 irregular chunks
    ASSERT_GE(chunks.size(), 5);
    ASSERT_LE(chunks.size(), 10);
}
```

**Success Criteria:**
- ✅ Wood creates splinters (elongated pieces)
- ✅ Concrete creates irregular chunks
- ✅ Glass creates many small shards with high velocity
- ✅ Initial velocities appropriate for materials

---

### Week 5 Deliverable

**What you should have:**

1. **Working Bullet Physics Integration:**
   - Dynamics world initialized
   - Ground plane collision
   - Rigid body creation from voxel clusters
   - Material properties applied

2. **Debris Management System:**
   - DebrisObject lifecycle tracking
   - Settling detection
   - Automatic cleanup
   - Debris count limiting (max 50)

3. **Material-Specific Behaviors:**
   - Fracture patterns (wood, concrete, brick, glass)
   - Material-specific initial velocities
   - Friction and restitution per material

4. **Performance:**
   - Can handle 50 active debris objects
   - Maintains 30+ FPS during simulation
   - Settling detection within 3 seconds

**Demo video (2 minutes):**
- Show simple box falling and bouncing
- Show voxel cluster converted to rigid body
- Show different materials (wood splinters vs concrete chunks)
- Show debris settling and cleanup
- Show performance stats

**Review & Decision:**
- ✅ Bullet integration working? → Proceed to Week 6
- ✅ Debris simulation realistic? → Continue
- ❌ Performance issues? → Optimize before Week 6

---

## Week 6: Complete Debris Simulation

### Goals
- Improve settling detection
- Add rubble generation (Track 2 integration)
- Implement particle effects
- Optimize debris simulation

### Day 26: Advanced Settling Detection

**Morning (4 hours):**

```cpp
□ Implement sleep-based settling

// Bullet has built-in sleeping, let's use it

void btRigidBody::setActivationState(int newState);
// ACTIVE_TAG = 1
// ISLAND_SLEEPING = 2
// WANTS_DEACTIVATION = 3
// DISABLE_DEACTIVATION = 4
// DISABLE_SIMULATION = 5

void ConfigureBodyForSettling(btRigidBody* body) {
    // Set sleep thresholds
    body->setSleepingThresholds(
        0.1f,   // Linear velocity threshold
        0.2f    // Angular velocity threshold
    );
    
    // Allow sleeping
    body->setActivationState(ACTIVE_TAG);
    
    // Set damping to help settle faster
    body->setDamping(
        0.3f,   // Linear damping
        0.3f    // Angular damping
    );
}

bool IsBodySleeping(btRigidBody* body) {
    return body->getActivationState() == ISLAND_SLEEPING;
}

□ Update DebrisObject to use sleep detection

void DebrisObject::Update(float deltaTime) {
    if (state == REMOVED) return;
    
    // Use Bullet's sleep detection
    if (IsBodySleeping(rigid_body)) {
        if (state != SETTLING) {
            time_below_threshold = 0;
            state = SETTLING;
        }
        
        time_below_threshold += deltaTime;
        
        if (time_below_threshold > 0.5f) {  // 0.5 seconds of sleep
            state = SETTLED;
        }
    } else {
        time_below_threshold = 0;
        state = ACTIVE;
    }
}
```

**Afternoon (4 hours):**

```cpp
□ Implement ground collision detection

class CollisionCallback : public btCollisionWorld::ContactResultCallback {
public:
    bool hit_ground;
    
    CollisionCallback() : hit_ground(false) {}
    
    btScalar addSingleResult(
        btManifoldPoint& cp,
        const btCollisionObjectWrapper* obj0, int partId0, int index0,
        const btCollisionObjectWrapper* obj1, int partId1, int index1) override 
    {
        // Check if one of the objects is ground
        if (obj0->getCollisionObject()->getUserIndex() == GROUND_ID ||
            obj1->getCollisionObject()->getUserIndex() == GROUND_ID) {
            hit_ground = true;
        }
        
        return 0;  // Continue checking other contacts
    }
};

bool IsDebrisOnGround(btRigidBody* body, btDiscreteDynamicsWorld* world) {
    CollisionCallback callback;
    world->contactTest(body, callback);
    return callback.hit_ground;
}

□ Add stacking detection

bool IsDebrisStable(DebrisObject* debris, 
                   std::vector<DebrisObject*>& all_debris) {
    // Check if debris is resting on something stable
    
    Vector3 pos = debris->GetPosition();
    
    // Check ground
    if (pos.y < 1.0f) return true;  // On ground
    
    // Check if resting on settled debris
    for (auto* other : all_debris) {
        if (other == debris) continue;
        if (!other->IsSettled()) continue;
        
        Vector3 other_pos = other->GetPosition();
        
        // Is this debris directly above other?
        float horizontal_dist = Distance2D(pos, other_pos);
        
        if (horizontal_dist < 0.5f && pos.y > other_pos.y) {
            return true;  // Resting on settled debris
        }
    }
    
    return false;
}

□ Test improved settling

TEST(Settling, StackedDebris) {
    BulletPhysicsSystem physics;
    physics.Initialize();
    physics.CreateGroundPlane();
    
    DebrisManager debris_manager(&physics);
    
    // Drop 3 boxes (they should stack)
    for (int i = 0; i < 3; i++) {
        VoxelCluster cluster = CreateTestCluster();
        cluster.center_of_mass.y = 5.0f + i * 2.0f;
        
        Material material = materials["concrete"];
        debris_manager.AddDebris(cluster, material);
    }
    
    // Simulate
    int settled_count = 0;
    for (int frame = 0; frame < 600; frame++) {  // 10 seconds max
        physics.Step(1.0f / 60.0f);
        debris_manager.Update(1.0f / 60.0f);
        
        settled_count = 0;
        for (auto* debris : debris_manager.GetDebris()) {
            if (debris->IsSettled()) settled_count++;
        }
        
        if (settled_count == 3) break;
    }
    
    ASSERT_EQ(settled_count, 3);
    
    physics.Shutdown();
}
```

**Success Criteria:**
- ✅ Sleep-based settling more reliable
- ✅ Ground collision detected correctly
- ✅ Stacked debris settles properly
- ✅ Settling within 3 seconds for most cases

---

### Day 27: Rubble Generation (Track 2 Integration)

**Morning (4 hours):**

```cpp
□ Create RubbleSystem interface

struct RubbleObject {
    Vector3 position;
    float height;
    float radius;
    Material material;
};

class RubbleSystem {
private:
    std::vector<RubbleObject> rubble_pieces;
    
public:
    void AddRubble(Vector3 position, float height, Material& material);
    void RemoveRubble(Vector3 position);
    std::vector<RubbleObject> GetRubbleNearby(Vector3 position, float radius);
    
    // For Track 2 (cover system)
    float GetCoverHeight(Vector3 position);
};

void RubbleSystem::AddRubble(Vector3 position, 
                             float height,
                             Material& material) {
    RubbleObject rubble;
    rubble.position = position;
    rubble.height = height;
    rubble.radius = 0.5f;  // Default 0.5m radius
    rubble.material = material;
    
    rubble_pieces.push_back(rubble);
    
    std::cout << "Rubble added at " << position 
              << " (height: " << height << "m)\n";
}

float RubbleSystem::GetCoverHeight(Vector3 position) {
    float total_height = 0;
    int count = 0;
    
    for (auto& rubble : rubble_pieces) {
        if (Distance2D(rubble.position, position) < rubble.radius) {
            total_height += rubble.height;
            count++;
        }
    }
    
    if (count == 0) return 0;
    return total_height / count;
}

□ Integrate rubble generation with debris settling

void DebrisManager::CleanupSettled(RubbleSystem* rubble_system) {
    auto it = active_debris.begin();
    
    while (it != active_debris.end()) {
        DebrisObject* debris = *it;
        
        if (debris->IsSettled()) {
            Vector3 final_pos = debris->GetPosition();
            
            // Calculate rubble height from debris bounds
            float height = debris->original_cluster.bounds.max.y - 
                          debris->original_cluster.bounds.min.y;
            
            // Add to rubble system
            if (rubble_system) {
                rubble_system->AddRubble(final_pos, height, debris->material);
            }
            
            // Cleanup debris
            debris->Cleanup(physics->GetWorld());
            delete debris;
            
            it = active_debris.erase(it);
        } else {
            ++it;
        }
    }
}
```

**Afternoon (4 hours):**

```cpp
□ Render rubble as simple shapes

void RenderRubble(RubbleSystem& rubble_system) {
    for (auto& rubble : rubble_system.GetAllRubble()) {
        // Render as a low pile
        
        // Material color
        Color color = GetMaterialColor(rubble.material);
        
        // Draw as flattened cylinder or pile of cubes
        if (rubble.height < 0.3f) {
            // Flat pile
            DrawFlatPile(rubble.position, rubble.radius, rubble.height, color);
        } else {
            // Medium pile
            DrawRubblePile(rubble.position, rubble.radius, rubble.height, color);
        }
    }
}

void DrawRubblePile(Vector3 position, float radius, float height, Color color) {
    // Draw multiple small cubes to represent rubble pile
    int cube_count = (int)(radius * height * 20);  // Density
    
    for (int i = 0; i < cube_count; i++) {
        // Random position within cylinder
        float angle = RandomFloat(0, 2 * PI);
        float r = RandomFloat(0, radius);
        float h = RandomFloat(0, height);
        
        Vector3 cube_pos(
            position.x + r * cos(angle),
            position.y + h,
            position.z + r * sin(angle)
        );
        
        float cube_size = 0.05f;  // 5cm cubes
        DrawCube(cube_pos, cube_size, color);
    }
}

□ Test rubble generation

TEST(Rubble, GenerationFromSettled) {
    BulletPhysicsSystem physics;
    physics.Initialize();
    physics.CreateGroundPlane();
    
    DebrisManager debris_manager(&physics);
    RubbleSystem rubble_system;
    
    // Add debris
    VoxelCluster cluster = CreateTestCluster();
    Material material = materials["concrete"];
    debris_manager.AddDebris(cluster, material);
    
    // Simulate until settled
    for (int i = 0; i < 300; i++) {
        physics.Step(1.0f / 60.0f);
        debris_manager.Update(1.0f / 60.0f);
        debris_manager.CleanupSettled(&rubble_system);
    }
    
    // Should have created rubble
    ASSERT_GT(rubble_system.GetRubbleCount(), 0);
    
    // Rubble should provide some cover
    Vector3 debris_pos = cluster.center_of_mass;
    float cover_height = rubble_system.GetCoverHeight(debris_pos);
    ASSERT_GT(cover_height, 0);
    
    physics.Shutdown();
}
```

**Success Criteria:**
- ✅ Rubble generated when debris settles
- ✅ Rubble height calculated correctly
- ✅ Rubble provides cover (Track 2 integration point)
- ✅ Rubble rendered visually

---

### Day 28: Particle Effects

**Morning (4 hours):**

```cpp
□ Create simple particle system

struct Particle {
    Vector3 position;
    Vector3 velocity;
    Color color;
    float size;
    float lifetime;
    float age;
};

class ParticleSystem {
private:
    std::vector<Particle> particles;
    
public:
    void Spawn(Vector3 position, Vector3 velocity, Color color, 
              float size, float lifetime);
    void Update(float deltaTime);
    void Render();
};

void ParticleSystem::Spawn(Vector3 position, Vector3 velocity, 
                          Color color, float size, float lifetime) {
    Particle p;
    p.position = position;
    p.velocity = velocity;
    p.color = color;
    p.size = size;
    p.lifetime = lifetime;
    p.age = 0;
    
    particles.push_back(p);
}

void ParticleSystem::Update(float deltaTime) {
    auto it = particles.begin();
    
    while (it != particles.end()) {
        it->age += deltaTime;
        
        if (it->age >= it->lifetime) {
            it = particles.erase(it);
        } else {
            // Update position
            it->position += it->velocity * deltaTime;
            
            // Apply gravity
            it->velocity.y -= 9.81f * deltaTime;
            
            // Fade out
            float alpha = 1.0f - (it->age / it->lifetime);
            it->color.a = alpha;
            
            ++it;
        }
    }
}

void ParticleSystem::Render() {
    for (auto& p : particles) {
        DrawBillboard(p.position, p.size, p.color);
    }
}

□ Add debris impact particles

void OnDebrisImpact(DebrisObject* debris, Vector3 impact_point) {
    Material& mat = debris->material;
    
    // Spawn particles based on material
    if (mat.name == "concrete") {
        // Dust cloud
        for (int i = 0; i < 20; i++) {
            Vector3 vel(
                RandomRange(-1, 1),
                RandomRange(0, 2),
                RandomRange(-1, 1)
            );
            
            particle_system.Spawn(
                impact_point,
                vel,
                Color(0.7f, 0.7f, 0.6f, 0.5f),  // Gray dust
                0.2f,   // Size
                2.0f    // Lifetime
            );
        }
        
        // Rock chips
        for (int i = 0; i < 10; i++) {
            Vector3 vel(
                RandomRange(-2, 2),
                RandomRange(1, 3),
                RandomRange(-2, 2)
            );
            
            particle_system.Spawn(
                impact_point,
                vel,
                Color(0.5f, 0.5f, 0.5f, 1.0f),  // Rock color
                0.05f,
                1.0f
            );
        }
    }
    // ... other materials
}
```

**Afternoon (4 hours):**

```cpp
□ Detect debris impacts using collision callback

class ImpactDetectionCallback : public btCollisionWorld::ContactResultCallback {
public:
    std::vector<Vector3> impact_points;
    float min_impulse;
    
    ImpactDetectionCallback(float min_imp = 10.0f) 
        : min_impulse(min_imp) {}
    
    btScalar addSingleResult(
        btManifoldPoint& cp,
        const btCollisionObjectWrapper* obj0, int partId0, int index0,
        const btCollisionObjectWrapper* obj1, int partId1, int index1) override 
    {
        // Get impact impulse
        float impulse = cp.getAppliedImpulse();
        
        if (impulse > min_impulse) {
            // Significant impact, record it
            Vector3 point = ToVector3(cp.getPositionWorldOnA());
            impact_points.push_back(point);
        }
        
        return 0;
    }
};

void DebrisManager::DetectImpacts() {
    for (auto* debris : active_debris) {
        ImpactDetectionCallback callback(10.0f);  // 10 N·s threshold
        physics->GetWorld()->contactTest(debris->rigid_body, callback);
        
        for (auto& impact_point : callback.impact_points) {
            OnDebrisImpact(debris, impact_point);
        }
    }
}

□ Add continuous dust during collapse

void SpawnCollapseAmbience(Vector3 epicenter, float radius) {
    // Spawn dust cloud over area
    
    for (int i = 0; i < 50; i++) {
        Vector3 offset(
            RandomRange(-radius, radius),
            RandomRange(0, radius),
            RandomRange(-radius, radius)
        );
        
        Vector3 pos = epicenter + offset;
        
        Vector3 vel(
            RandomRange(-0.5f, 0.5f),
            RandomRange(0, 1),
            RandomRange(-0.5f, 0.5f)
        );
        
        particle_system.Spawn(
            pos,
            vel,
            Color(0.6f, 0.6f, 0.5f, 0.3f),
            0.5f,   // Large dust clouds
            5.0f    // Long lifetime
        );
    }
}

□ Test particles

void TestParticleEffects() {
    BulletPhysicsSystem physics;
    physics.Initialize();
    physics.CreateGroundPlane();
    
    ParticleSystem particle_system;
    DebrisManager debris_manager(&physics);
    
    // Add debris
    VoxelCluster cluster = CreateTestCluster();
    cluster.center_of_mass.y = 10.0f;  // High drop
    Material material = materials["concrete"];
    
    debris_manager.AddDebris(cluster, material);
    
    // Simulate
    for (int i = 0; i < 180; i++) {  // 3 seconds
        physics.Step(1.0f / 60.0f);
        debris_manager.Update(1.0f / 60.0f);
        debris_manager.DetectImpacts();
        
        particle_system.Update(1.0f / 60.0f);
        
        // Render
        RenderScene();
        particle_system.Render();
    }
    
    physics.Shutdown();
}
```

**Success Criteria:**
- ✅ Particles spawn on impact
- ✅ Dust clouds for concrete
- ✅ Material-specific particle colors
- ✅ Particles fade out over time

---

### Day 29-30: Optimization & Polish

**Day 29 Morning:**

```cpp
□ Collision filtering (debris doesn't collide with debris)

void SetupCollisionGroups() {
    // Define collision groups
    enum CollisionGroup {
        COL_GROUND  = 1 << 0,  // 0x0001
        COL_DEBRIS  = 1 << 1,  // 0x0002
        COL_UNITS   = 1 << 2   // 0x0004
    };
    
    // Ground collides with everything
    ground_body->getBroadphaseHandle()->m_collisionFilterGroup = COL_GROUND;
    ground_body->getBroadphaseHandle()->m_collisionFilterMask = 
        COL_GROUND | COL_DEBRIS | COL_UNITS;
    
    // Debris collides with ground and units, NOT other debris
    for (auto* debris : debris_manager.GetDebris()) {
        debris->rigid_body->getBroadphaseHandle()->m_collisionFilterGroup = 
            COL_DEBRIS;
        debris->rigid_body->getBroadphaseHandle()->m_collisionFilterMask = 
            COL_GROUND | COL_UNITS;  // NOT COL_DEBRIS
    }
}

□ Profile physics performance

void ProfilePhysics() {
    BulletPhysicsSystem physics;
    physics.Initialize();
    physics.CreateGroundPlane();
    
    DebrisManager debris_manager(&physics);
    
    // Create 50 debris (max)
    for (int i = 0; i < 50; i++) {
        VoxelCluster cluster = CreateTestCluster();
        Material material = materials["concrete"];
        debris_manager.AddDebris(cluster, material);
    }
    
    // Profile 300 frames (5 seconds)
    std::vector<float> frame_times;
    
    for (int i = 0; i < 300; i++) {
        auto start = Clock::now();
        
        physics.Step(1.0f / 60.0f);
        debris_manager.Update(1.0f / 60.0f);
        
        auto end = Clock::now();
        float elapsed = GetElapsedMS(start, end);
        frame_times.push_back(elapsed);
    }
    
    // Print statistics
    float avg = Average(frame_times);
    float max_time = Max(frame_times);
    float min_time = Min(frame_times);
    
    std::cout << "Physics Performance (50 debris):\n";
    std::cout << "  Average: " << avg << "ms (" << (1000.0f/avg) << " FPS)\n";
    std::cout << "  Max: " << max_time << "ms\n";
    std::cout << "  Min: " << min_time << "ms\n";
    
    ASSERT_LT(avg, 33.0f);  // Should be < 33ms (30 FPS)
    
    physics.Shutdown();
}
```

**Day 29 Afternoon:**

```cpp
□ Add LOD for distant debris

void DebrisManager::UpdateLOD(Vector3 camera_position) {
    for (auto* debris : active_debris) {
        Vector3 debris_pos = debris->GetPosition();
        float distance = Distance(debris_pos, camera_position);
        
        if (distance > 30.0f) {
            // Far: Disable collision (visual only)
            debris->rigid_body->setCollisionFlags(
                debris->rigid_body->getCollisionFlags() |
                btCollisionObject::CF_NO_CONTACT_RESPONSE
            );
        } else {
            // Close: Enable collision
            debris->rigid_body->setCollisionFlags(
                debris->rigid_body->getCollisionFlags() &
                ~btCollisionObject::CF_NO_CONTACT_RESPONSE
            );
        }
    }
}

□ Batch debris cleanup

void DebrisManager::CleanupOldDebris(float max_age) {
    auto it = active_debris.begin();
    
    int cleaned = 0;
    
    while (it != active_debris.end()) {
        DebrisObject* debris = *it;
        
        // Remove debris older than max_age
        float age = GetTime() - debris->spawn_time;
        
        if (age > max_age) {
            debris->Cleanup(physics->GetWorld());
            delete debris;
            it = active_debris.erase(it);
            cleaned++;
        } else {
            ++it;
        }
    }
    
    if (cleaned > 0) {
        std::cout << "Cleaned up " << cleaned << " old debris\n";
    }
}
```

**Day 30 - Documentation:**

```cpp
□ Create Track 3 API documentation

/**
 * Track 3: Physics Integration with Bullet
 * 
 * Purpose: Simulates realistic debris physics after structural collapse
 * 
 * Usage:
 *   BulletPhysicsSystem physics;
 *   physics.Initialize();
 *   
 *   DebrisManager debris_manager(&physics);
 *   
 *   // When structure fails (from Track 1)
 *   for (auto& cluster : failed_clusters) {
 *       debris_manager.AddDebris(cluster, material);
 *   }
 *   
 *   // Each frame
 *   physics.Step(deltaTime);
 *   debris_manager.Update(deltaTime);
 *   debris_manager.CleanupSettled(&rubble_system);
 * 
 * Performance: 30+ FPS with 50 active debris objects
 */

□ Write integration examples

// examples/track1_track3_integration.cpp

void FullDestructionPipeline() {
    // Systems
    VoxelWorld world;
    StructuralAnalyzer structural;
    BulletPhysicsSystem physics;
    DebrisManager debris_manager(&physics);
    RubbleSystem rubble_system;
    
    physics.Initialize();
    physics.CreateGroundPlane();
    
    // When voxels destroyed
    std::vector<Vector3> damaged = GetDamagedVoxels();
    
    // Track 1: Analyze structure
    auto result = structural.Analyze(world, damaged, 500);
    
    if (result.structure_failed) {
        // Track 3: Create physics debris
        for (auto& cluster : result.failed_clusters) {
            Material mat = GetMaterialForCluster(cluster);
            debris_manager.AddDebris(cluster, mat);
        }
        
        // Simulate collapse
        for (int i = 0; i < 180; i++) {  // 3 seconds
            physics.Step(1.0f / 60.0f);
            debris_manager.Update(1.0f / 60.0f);
            debris_manager.CleanupSettled(&rubble_system);
            
            RenderFrame();
        }
    }
}
```

**Success Criteria:**
- ✅ Collision filtering improves performance
- ✅ LOD system working
- ✅ Performance acceptable (30+ FPS with 50 debris)
- ✅ Documentation complete

---

### Week 6 Deliverable

**What you should have:**

1. **Complete Debris Simulation:**
   - Reliable settling detection (sleep-based)
   - Stacking detection
   - Ground collision detection
   - Automatic cleanup

2. **Rubble Integration:**
   - Rubble generated from settled debris
   - Rubble provides cover (Track 2 ready)
   - Rubble rendered visually

3. **Visual Effects:**
   - Particle system
   - Impact particles (dust, chips)
   - Material-specific effects
   - Collapse ambience

4. **Performance Optimizations:**
   - Collision filtering
   - LOD system
   - Debris limits enforced
   - 30+ FPS with 50 debris

5. **Documentation:**
   - Complete API docs
   - Integration examples
   - Usage guide

**Demo video (3 minutes):**
- Show complete destruction pipeline (Track 1 → Track 3)
- Show debris falling and settling
- Show rubble generation
- Show particle effects
- Show multiple materials
- Show performance metrics

**Review & Decision:**
- ✅ Track 3 complete? → Proceed to Week 7 (Integration)
- ✅ Physics realistic? → Continue
- ❌ Issues? → Address before Week 7

---

## Week 7: Complete Integration & Visual Polish

### Goals
- Connect all systems (Track 1 + Track 3 + Rubble)
- Add turn-based timing control
- Implement visual feedback systems
- Create test scenarios

### Day 31: Turn-Based Timing

**Morning (4 hours):**

```cpp
□ Create TurnManager

class TurnManager {
public:
    enum TurnPhase {
        PLAYER_ACTION,      // Player moving/shooting
        STRUCTURAL_ANALYSIS,// Track 1 running
        PHYSICS_SIMULATION, // Track 3 running
        AI_TURN            // Enemy turn
    };
    
private:
    TurnPhase current_phase;
    float phase_timer;
    
public:
    void StartTurnTransition();
    void Update(float deltaTime);
    TurnPhase GetPhase() const { return current_phase; }
};

void TurnManager::StartTurnTransition() {
    current_phase = STRUCTURAL_ANALYSIS;
    phase_timer = 0;
}

void TurnManager::Update(float deltaTime) {
    phase_timer += deltaTime;
    
    switch (current_phase) {
        case STRUCTURAL_ANALYSIS:
            // Waiting for structural analysis to complete
            if (structural_complete) {
                if (structure_failed) {
                    current_phase = PHYSICS_SIMULATION;
                } else {
                    current_phase = AI_TURN;
                }
                phase_timer = 0;
            }
            break;
            
        case PHYSICS_SIMULATION:
            // Waiting for all debris to settle
            if (all_debris_settled) {
                current_phase = AI_TURN;
                phase_timer = 0;
            }
            break;
            
        case AI_TURN:
            // AI taking its turn
            // ...
            break;
    }
}

□ Implement fast-forward physics simulation

void SimulateCollapseFastForward(float game_time_duration) {
    const float TIMESTEP = 1.0f / 60.0f;
    int steps = (int)(game_time_duration / TIMESTEP);
    
    std::cout << "Simulating " << game_time_duration << "s collapse ("
              << steps << " steps)\n";
    
    auto start = Clock::now();
    
    for (int i = 0; i < steps; i++) {
        physics.Step(TIMESTEP);
        debris_manager.Update(TIMESTEP);
        debris_manager.CleanupSettled(&rubble_system);
        
        // Optional: Record keyframes for playback
        if (i % 2 == 0) {  // Every other frame
            RecordKeyframe();
        }
        
        // Early exit if all settled
        if (debris_manager.GetActiveCount() == 0) {
            std::cout << "All debris settled at step " << i << "\n";
            break;
        }
    }
    
    auto end = Clock::now();
    float real_time = GetElapsedSeconds(start, end);
    
    std::cout << "Simulation complete: " << game_time_duration 
              << "s game time in " << real_time << "s real time\n";
}
```

**Afternoon (4 hours):**

```cpp
□ Create keyframe recording system

struct TransformKeyframe {
    float time;
    std::map<int, btTransform> debris_transforms;
};

class KeyframeRecorder {
private:
    std::vector<TransformKeyframe> keyframes;
    
public:
    void StartRecording();
    void RecordFrame(float time, std::vector<DebrisObject*>& debris);
    void PlaybackAnimation();
};

void KeyframeRecorder::RecordFrame(float time, 
                                   std::vector<DebrisObject*>& debris) {
    TransformKeyframe keyframe;
    keyframe.time = time;
    
    for (auto* d : debris) {
        btTransform transform;
        d->rigid_body->getMotionState()->getWorldTransform(transform);
        keyframe.debris_transforms[d->id] = transform;
    }
    
    keyframes.push_back(keyframe);
}

void KeyframeRecorder::PlaybackAnimation() {
    // Play back recorded keyframes at 60 FPS
    
    for (size_t i = 0; i < keyframes.size(); i++) {
        auto& keyframe = keyframes[i];
        
        // Update visual transforms
        for (auto& [id, transform] : keyframe.debris_transforms) {
            UpdateDebrisVisual(id, transform);
        }
        
        // Render
        RenderFrame();
        
        // Wait for next frame
        Sleep(1.0f / 60.0f);
    }
}

□ Test turn-based timing

TEST(TurnBased, FastForwardSimulation) {
    BulletPhysicsSystem physics;
    physics.Initialize();
    physics.CreateGroundPlane();
    
    DebrisManager debris_manager(&physics);
    KeyframeRecorder recorder;
    
    // Add debris
    for (int i = 0; i < 10; i++) {
        VoxelCluster cluster = CreateTestCluster();
        Material material = materials["concrete"];
        debris_manager.AddDebris(cluster, material);
    }
    
    // Fast-forward simulate 3 seconds
    auto start = Clock::now();
    
    recorder.StartRecording();
    
    for (int frame = 0; frame < 180; frame++) {
        float time = frame / 60.0f;
        
        physics.Step(1.0f / 60.0f);
        debris_manager.Update(1.0f / 60.0f);
        
        recorder.RecordFrame(time, debris_manager.GetDebris());
    }
    
    auto end = Clock::now();
    float elapsed = GetElapsedSeconds(start, end);
    
    std::cout << "Simulated 3s in " << elapsed << "s\n";
    
    // Should be faster than real-time (no rendering)
    ASSERT_LT(elapsed, 2.0f);
    
    physics.Shutdown();
}
```

**Success Criteria:**
- ✅ Turn manager tracks phases
- ✅ Can fast-forward physics simulation
- ✅ Keyframe recording works
- ✅ Simulation runs faster than real-time

---

### Day 32: Visual Feedback Systems

**Morning (4 hours):**

```cpp
□ Add damage visualization

class VoxelVisualizer {
public:
    void HighlightDamagedVoxels(std::vector<Vector3>& positions);
    void ShowFailedClusters(std::vector<VoxelCluster>& clusters);
    void AnimateCracking(Vector3 position, float intensity);
};

void VoxelVisualizer::ShowFailedClusters(
    std::vector<VoxelCluster>& clusters) 
{
    for (size_t i = 0; i < clusters.size(); i++) {
        Color cluster_color = GetClusterColor(i);
        
        for (auto& pos : clusters[i].voxel_positions) {
            // Flash red before falling
            DrawVoxel(pos, Color::Lerp(cluster_color, Color::Red, 0.5f));
        }
        
        // Draw center of mass
        DrawSphere(clusters[i].center_of_mass, 0.2f, Color::Yellow);
        
        // Draw bounding box
        DrawBoundingBox(clusters[i].bounds, Color::White);
    }
}

□ Add UI overlay

class DestructionUI {
public:
    void ShowStructuralAnalysis(AnalysisResult& result);
    void ShowPhysicsStats(DebrisManager& debris);
    void ShowWarning(const std::string& message);
};

void DestructionUI::ShowStructuralAnalysis(AnalysisResult& result) {
    ImGui::Begin("Structural Analysis");
    
    ImGui::Text("Status: %s", result.structure_failed ? "FAILED" : "STABLE");
    ImGui::Text("Analysis Time: %.1f ms", result.calculation_time_ms);
    
    if (result.structure_failed) {
        ImGui::Text("Failed Clusters: %d", result.failed_clusters.size());
        
        for (size_t i = 0; i < result.failed_clusters.size(); i++) {
            auto& cluster = result.failed_clusters[i];
            ImGui::Text("  Cluster %d: %d voxels, %.1f kg",
                       i,
                       cluster.voxel_positions.size(),
                       cluster.total_mass);
        }
    }
    
    ImGui::Text("\n%s", result.debug_info.c_str());
    
    ImGui::End();
}

void DestructionUI::ShowPhysicsStats(DebrisManager& debris) {
    ImGui::Begin("Physics");
    
    ImGui::Text("Active Debris: %d / %d", 
               debris.GetActiveCount(), 
               50);  // MAX_DEBRIS
    
    ImGui::ProgressBar(debris.GetActiveCount() / 50.0f);
    
    // Count by state
    int active = 0, settling = 0, settled = 0;
    for (auto* d : debris.GetDebris()) {
        switch (d->state) {
            case ACTIVE: active++; break;
            case SETTLING: settling++; break;
            case SETTLED: settled++; break;
        }
    }
    
    ImGui::Text("  Active: %d", active);
    ImGui::Text("  Settling: %d", settling);
    ImGui::Text("  Settled: %d", settled);
    
    ImGui::End();
}
```

**Afternoon (4 hours):**

```cpp
□ Add camera shake on impact

class CameraEffects {
private:
    Vector3 shake_offset;
    float shake_intensity;
    float shake_duration;
    
public:
    void TriggerShake(float intensity, float duration);
    void Update(float deltaTime);
    Vector3 GetShakeOffset() const { return shake_offset; }
};

void CameraEffects::TriggerShake(float intensity, float duration) {
    shake_intensity = intensity;
    shake_duration = duration;
}

void CameraEffects::Update(float deltaTime) {
    if (shake_duration > 0) {
        // Random shake
        shake_offset = Vector3(
            RandomRange(-1, 1),
            RandomRange(-1, 1),
            RandomRange(-1, 1)
        ) * shake_intensity;
        
        shake_duration -= deltaTime;
        
        // Fade out
        shake_intensity *= 0.9f;
    } else {
        shake_offset = Vector3(0, 0, 0);
    }
}

□ Add slow-motion for dramatic effect

class TimeScale {
private:
    float current_scale = 1.0f;
    float target_scale = 1.0f;
    
public:
    void SetSlowMotion(float scale = 0.3f, float duration = 2.0f);
    void Update(float deltaTime);
    float GetScale() const { return current_scale; }
};

void TimeScale::SetSlowMotion(float scale, float duration) {
    target_scale = scale;
    
    // Smooth transition
    // ...
}

void MainLoop() {
    CameraEffects camera_fx;
    TimeScale time_scale;
    
    while (Running()) {
        float dt = GetDeltaTime() * time_scale.GetScale();
        
        // Update systems
        physics.Step(dt);
        debris_manager.Update(dt);
        camera_fx.Update(dt);
        time_scale.Update(dt);
        
        // Apply camera shake
        camera.position += camera_fx.GetShakeOffset();
        
        // Render
        RenderFrame();
    }
}

□ Test visual feedback

void DemoVisualFeedback() {
    // Create test scenario
    // ...
    
    // Show visual progression:
    // 1. Highlight damaged voxels (yellow)
    visualizer.HighlightDamagedVoxels(damaged_positions);
    Sleep(0.5f);
    
    // 2. Flash failing clusters (red)
    visualizer.ShowFailedClusters(failed_clusters);
    Sleep(0.5f);
    
    // 3. Trigger slow-motion
    time_scale.SetSlowMotion(0.3f, 2.0f);
    
    // 4. Start collapse with camera shake
    camera_fx.TriggerShake(0.5f, 2.0f);
    
    // 5. Show physics stats
    ui.ShowPhysicsStats(debris_manager);
}
```

**Success Criteria:**
- ✅ Visual highlighting of failed clusters
- ✅ UI shows analysis results
- ✅ Camera shake on impacts
- ✅ Slow-motion effects work

---

### Day 33-34: Test Scenarios & Polish

**Day 33 Morning:**

```cpp
□ Create comprehensive test scenarios

struct TestScenario {
    std::string name;
    std::function<void(VoxelWorld&)> setup;
    std::function<std::vector<Vector3>()> damage;
    bool should_collapse;
    int expected_debris_count;
};

std::vector<TestScenario> CreateTestScenarios() {
    std::vector<TestScenario> scenarios;
    
    // Scenario 1: Simple tower
    scenarios.push_back({
        "Tower Collapse",
        [](VoxelWorld& world) {
            CreateTestTower(world, 20);
        },
        []() {
            return std::vector<Vector3>{Vector3(0, 0, 0)};  // Remove base
        },
        true,  // Should collapse
        1      // 1 debris cluster
    });
    
    // Scenario 2: Building with columns
    scenarios.push_back({
        "Building Corner Column",
        [](VoxelWorld& world) {
            CreateTestBuilding(world, 10, 10, 5);
        },
        []() {
            std::vector<Vector3> damaged;
            // Remove corner column
            for (int y = 0; y < 5; y++) {
                damaged.push_back(Vector3(0, y * 0.05, 0));
            }
            return damaged;
        },
        true,
        2  // Corner section + maybe part of roof
    });
    
    // Scenario 3: Bridge
    scenarios.push_back({
        "Bridge Partial Collapse",
        [](VoxelWorld& world) {
            CreateTestBridge(world, 20);
        },
        []() {
            std::vector<Vector3> damaged;
            // Remove one support
            for (int y = 0; y < 10; y++) {
                damaged.push_back(Vector3(0, y * 0.05, 0));
            }
            return damaged;
        },
        true,
        1  // Half of bridge deck
    });
    
    // Scenario 4: Redundant wall
    scenarios.push_back({
        "Wall Stays Up",
        [](VoxelWorld& world) {
            CreateTestWall(world, 20, 10);
        },
        []() {
            std::vector<Vector3> damaged;
            // Remove a few random voxels
            for (int i = 0; i < 5; i++) {
                damaged.push_back(Vector3(
                    RandomRange(0, 20) * 0.05,
                    RandomRange(0, 10) * 0.05,
                    0
                ));
            }
            return damaged;
        },
        false,  // Should NOT collapse
        0
    });
    
    return scenarios;
}

□ Run all test scenarios

void RunAllScenarios() {
    auto scenarios = CreateTestScenarios();
    
    int passed = 0;
    int failed = 0;
    
    for (auto& scenario : scenarios) {
        std::cout << "\n=== Testing: " << scenario.name << " ===\n";
        
        // Setup
        VoxelWorld world;
        BulletPhysicsSystem physics;
        DebrisManager debris_manager(&physics);
        StructuralAnalyzer structural;
        
        physics.Initialize();
        physics.CreateGroundPlane();
        
        scenario.setup(world);
        
        // Damage
        auto damaged = scenario.damage();
        for (auto& pos : damaged) {
            world.RemoveVoxel(pos);
        }
        
        // Analyze
        auto result = structural.Analyze(world, damaged, 500);
        
        // Check expectation
        bool correct = (result.structure_failed == scenario.should_collapse);
        
        if (correct && result.structure_failed) {
            // Also check debris count
            correct = (result.failed_clusters.size() == 
                      scenario.expected_debris_count);
        }
        
        if (correct) {
            std::cout << "✓ PASSED\n";
            passed++;
        } else {
            std::cout << "✗ FAILED\n";
            std::cout << "  Expected collapse: " << scenario.should_collapse 
                      << "\n";
            std::cout << "  Got: " << result.structure_failed << "\n";
            failed++;
        }
        
        physics.Shutdown();
    }
    
    std::cout << "\n=== Results ===\n";
    std::cout << "Passed: " << passed << "\n";
    std::cout << "Failed: " << failed << "\n";
}
```

**Day 33 Afternoon:**

```cpp
□ Add save/load for scenarios

void SaveScenario(const std::string& filename, VoxelWorld& world) {
    std::ofstream file(filename, std::ios::binary);
    
    // Save voxel count
    int count = world.GetVoxelCount();
    file.write((char*)&count, sizeof(int));
    
    // Save each voxel
    for (auto& [pos, voxel] : world.GetAllVoxels()) {
        file.write((char*)&pos, sizeof(Vector3));
        file.write((char*)&voxel, sizeof(Voxel));
    }
    
    file.close();
}

VoxelWorld LoadScenario(const std::string& filename) {
    VoxelWorld world;
    std::ifstream file(filename, std::ios::binary);
    
    // Load voxel count
    int count;
    file.read((char*)&count, sizeof(int));
    
    // Load each voxel
    for (int i = 0; i < count; i++) {
        Vector3 pos;
        Voxel voxel;
        
        file.read((char*)&pos, sizeof(Vector3));
        file.read((char*)&voxel, sizeof(Voxel));
        
        world.SetVoxel(pos, voxel);
    }
    
    file.close();
    return world;
}

□ Create scenario browser

void ScenarioBrowserUI() {
    ImGui::Begin("Scenarios");
    
    static int selected = 0;
    auto scenarios = GetAvailableScenarios();
    
    // List scenarios
    if (ImGui::BeginListBox("##scenarios")) {
        for (size_t i = 0; i < scenarios.size(); i++) {
            bool is_selected = (selected == i);
            if (ImGui::Selectable(scenarios[i].name.c_str(), 
                                 is_selected)) {
                selected = i;
            }
        }
        ImGui::EndListBox();
    }
    
    // Load button
    if (ImGui::Button("Load Scenario")) {
        LoadAndRunScenario(scenarios[selected]);
    }
    
    ImGui::End();
}
```

**Day 34 - Final Polish:**

```cpp
□ Performance profiling

void GeneratePerformanceReport() {
    std::ofstream report("performance_report.txt");
    
    struct TestCase {
        std::string name;
        int voxel_count;
        int debris_count;
    };
    
    std::vector<TestCase> cases = {
        {"Small (1K voxels, 5 debris)", 1000, 5},
        {"Medium (10K voxels, 20 debris)", 10000, 20},
        {"Large (50K voxels, 50 debris)", 50000, 50}
    };
    
    for (auto& test : cases) {
        report << "\n=== " << test.name << " ===\n";
        
        // Create scenario
        VoxelWorld world;
        CreateLargeStructure(world, test.voxel_count);
        
        // Track 1 timing
        auto start1 = Clock::now();
        auto structural_result = structural.Analyze(world, damaged, 500);
        float t1 = GetElapsedMS(start1);
        
        report << "Track 1 (Structural): " << t1 << "ms\n";
        
        // Track 3 timing
        BulletPhysicsSystem physics;
        physics.Initialize();
        DebrisManager debris_manager(&physics);
        
        auto start3 = Clock::now();
        
        // Simulate 180 frames
        for (int i = 0; i < 180; i++) {
            physics.Step(1.0f / 60.0f);
            debris_manager.Update(1.0f / 60.0f);
        }
        
        float t3 = GetElapsedMS(start3);
        
        report << "Track 3 (Physics 3s): " << t3 << "ms\n";
        report << "  FPS equivalent: " << (180.0f / (t3/1000.0f)) << "\n";
        
        physics.Shutdown();
    }
    
    report.close();
}

□ Create final demo

void FinalDemo() {
    // Best showcase of all features
    
    // 1. Load impressive scenario
    VoxelWorld world = LoadScenario("demo_building.vxl");
    
    // 2. Show intact building
    ShowCamera Flythrough(5.0f);
    
    // 3. Apply damage
    ui.ShowMessage("Destroying load-bearing column...");
    ApplyDamage(corner_column);
    
    // 4. Track 1: Analysis
    ui.ShowMessage("Analyzing structural integrity...");
    auto result = structural.Analyze(world, damaged, 500);
    ui.ShowStructuralAnalysis(result);
    Sleep(1.0f);
    
    // 5. Track 3: Collapse
    ui.ShowMessage("Simulating collapse...");
    visualizer.ShowFailedClusters(result.failed_clusters);
    
    time_scale.SetSlowMotion(0.3f, 3.0f);
    camera_fx.TriggerShake(0.8f, 3.0f);
    
    // Simulate and record
    for (int i = 0; i < 180; i++) {
        physics.Step(1.0f / 60.0f);
        debris_manager.Update(1.0f / 60.0f);
        particle_system.Update(1.0f / 60.0f);
        
        RenderFrame();
    }
    
    // 6. Show rubble
    ui.ShowMessage("Collapse complete. Rubble generated.");
    rubble_system.RenderAll();
    
    // 7. Show statistics
    ShowFinalStats();
}
```

**Success Criteria:**
- ✅ All test scenarios pass
- ✅ Can save/load scenarios
- ✅ Performance report generated
- ✅ Final demo impressive

---

### Week 7 Deliverable

**What you should have:**

1. **Complete Integration:**
   - Track 1 (Structural) → Track 3 (Physics) pipeline
   - Turn-based timing control
   - Keyframe recording/playback
   - Rubble generation

2. **Visual Polish:**
   - Damage visualization
   - UI overlays (analysis, physics stats)
   - Camera effects (shake, slow-mo)
   - Particle effects

3. **Test Suite:**
   - 4+ comprehensive scenarios
   - Save/load system
   - Scenario browser
   - All scenarios passing

4. **Performance:**
   - Track 1: < 500ms
   - Track 3: 30+ FPS with 50 debris
   - Total: < 5 seconds for full collapse

**Demo video (5 minutes):**
- Show intact structure
- Show damage application
- Show structural analysis (UI overlay)
- Show dramatic collapse (slow-mo, shake)
- Show debris simulation
- Show rubble generation
- Show multiple scenarios
- Show performance stats

**Review & Decision:**
- ✅ Integration complete? → Proceed to Week 8 (Final)
- ✅ Visuals impressive? → Continue
- ❌ Issues? → Address before Week 8

---

## Week 8: Final Demo & Documentation

### Goals
- Record showcase video
- Write complete documentation
- Create usage guide
- Plan next steps

### Day 35-36: Showcase Video Production

**Day 35:**

```cpp
□ Script for showcase video

# Voxel Destruction Engine - Showcase Video Script

## Intro (30 seconds)
- Title card: "Voxel Destruction Engine"
- Show impressive building
- Voiceover: "A complete voxel-based tactical destruction system"

## Feature 1: Structural Analysis (60 seconds)
- Show intact building
- Apply damage to load-bearing column
- Show structural analysis overlay
- Highlight failed clusters in red
- Show analysis time: "Analysis: 180ms"

## Feature 2: Physics Simulation (90 seconds)
- Show collapse in slow-motion
- Camera shake effect
- Multiple camera angles
- Particle effects (dust, debris)
- Show debris settling
- Show final rubble pile

## Feature 3: Materials (60 seconds)
- Show wood splinters
- Show concrete chunks
- Show brick masonry units
- Show glass shatter
- Compare behaviors side-by-side

## Feature 4: Performance (30 seconds)
- Show performance stats overlay
- 50 active debris objects
- 35 FPS during simulation
- Track 1: 200ms
- Track 3: 2.8s total

## Feature 5: Integration (60 seconds)
- Show complete gameplay loop
- Player destroys voxels
- Structural analysis
- Physics simulation
- Rubble provides cover
- Turn-based timing

## Outro (30 seconds)
- Summary statistics
- "Built with Bullet Physics"
- "8 weeks development"
- GitHub link / contact info

Total: ~6 minutes

□ Record footage

void RecordShowcaseFootage() {
    // Enable recording
    screen_recorder.Start("showcase_footage.mp4", 1920, 1080, 60);
    
    // Scene 1: Building reveal
    camera.SetPosition(Vector3(20, 10, 20));
    camera.LookAt(building_center);
    
    for (int i = 0; i < 300; i++) {  // 5 seconds
        RenderFrame();
        screen_recorder.CaptureFrame();
    }
    
    // Scene 2: Damage application
    ui.ShowMessage("Applying explosive damage...");
    HighlightDamageArea(corner_column);
    
    // ... etc
    
    screen_recorder.Stop();
}
```

**Day 36:**

```cpp
□ Edit video

- Use video editing software (DaVinci Resolve, Premiere)
- Add title cards
- Add statistics overlays
- Add slow-motion effects
- Add background music
- Export final video

□ Create animated GIFs for documentation

void ExportGIF(const std::string& name, int start_frame, int end_frame) {
    // Export specific sequence as GIF
    // Use FFmpeg or similar
    
    std::string command = "ffmpeg -i showcase_footage.mp4 "
                         "-ss " + std::to_string(start_frame / 60.0f) + " "
                         "-t " + std::to_string((end_frame - start_frame) / 60.0f) + " "
                         "-vf 'scale=800:-1' "
                         name + ".gif";
    
    system(command.c_str());
}

// Export key moments as GIFs
ExportGIF("collapse", 180, 360);
ExportGIF("wood_splinters", 420, 480);
ExportGIF("glass_shatter", 540, 600);
```

**Success Criteria:**
- ✅ 6-minute showcase video complete
- ✅ Professional quality (1080p60)
- ✅ All features demonstrated
- ✅ Performance stats shown
- ✅ GIFs exported for docs

---

### Day 37-38: Complete Documentation

**Day 37 - Technical Documentation:**

```markdown
□ Create README.md

# Voxel Destruction Engine

A complete voxel-based tactical destruction system with structural integrity analysis and realistic physics simulation.

## Features

- **Structural Integrity Analysis**: Displacement-based spring system determines which voxels should collapse
- **Physics Simulation**: Bullet Physics integration for realistic debris behavior
- **Material System**: Wood, brick, concrete, glass with unique fracture patterns
- **Debris Management**: Automatic settling detection and cleanup
- **Rubble Generation**: Settled debris becomes tactical cover
- **Performance**: < 500ms structural analysis, 30+ FPS physics with 50 debris

## Quick Start

```bash
# Build
mkdir build && cd build
cmake ..
make -j8

# Run demo
./voxel_destruction_demo
```

## Architecture

```
VoxelWorld (Weeks 1-2)
    ↓
Track 1: Structural Analysis (Weeks 3-4)
    ↓
Track 3: Physics Simulation (Weeks 5-6)
    ↓
Rubble System (Week 6)
```

## Dependencies

- Bullet Physics 3.x
- OpenGL 4.x
- GLFW 3.x
- GLM or Eigen
- GoogleTest (for tests)

## Documentation

- [API Reference](docs/api.md)
- [Usage Guide](docs/usage.md)
- [Performance Guide](docs/performance.md)
- [Integration Guide](docs/integration.md)

## Performance

| Scenario | Voxels | Debris | Track 1 | Track 3 | Total |
|----------|--------|--------|---------|---------|-------|
| Small    | 1K     | 5      | 50ms    | 0.8s    | 0.9s  |
| Medium   | 10K    | 20     | 200ms   | 1.5s    | 1.7s  |
| Large    | 50K    | 50     | 480ms   | 2.8s    | 3.3s  |

## License

[Your License Here]

□ Create API documentation

# API Reference

## Core Classes

### VoxelWorld

```cpp
class VoxelWorld {
public:
    bool HasVoxel(Vector3 position);
    Voxel GetVoxel(Vector3 position);
    void SetVoxel(Vector3 position, Voxel voxel);
    void RemoveVoxel(Vector3 position);
    
    std::vector<Vector3> GetNeighbors(Vector3 position);
    std::vector<Vector3> GetSurfaceVoxels();
};
```

### StructuralAnalyzer (Track 1)

```cpp
struct AnalysisResult {
    bool structure_failed;
    std::vector<VoxelCluster> failed_clusters;
    float calculation_time_ms;
};

class StructuralAnalyzer {
public:
    AnalysisResult Analyze(
        VoxelWorld& world,
        std::vector<Vector3> damaged_positions,
        float time_budget_ms = 500
    );
};
```

### BulletPhysicsSystem (Track 3)

```cpp
class BulletPhysicsSystem {
public:
    void Initialize();
    void Shutdown();
    void Step(float deltaTime);
    void CreateGroundPlane();
    
    btDiscreteDynamicsWorld* GetWorld();
};
```

### DebrisManager

```cpp
class DebrisManager {
public:
    void AddDebris(VoxelCluster& cluster, Material& material);
    void Update(float deltaTime);
    void CleanupSettled(RubbleSystem* rubble_system = nullptr);
    
    int GetActiveCount() const;
};
```

## Usage Examples

### Basic Destruction

```cpp
VoxelWorld world;
StructuralAnalyzer structural;
BulletPhysicsSystem physics;
DebrisManager debris_manager(&physics);

// When voxels destroyed
std::vector<Vector3> damaged = GetDamagedVoxels();

// Analyze structure
auto result = structural.Analyze(world, damaged);

if (result.structure_failed) {
    // Create physics debris
    for (auto& cluster : result.failed_clusters) {
        Material mat = GetMaterialForCluster(cluster);
        debris_manager.AddDebris(cluster, mat);
    }
    
    // Simulate collapse
    for (int i = 0; i < 180; i++) {
        physics.Step(1.0f / 60.0f);
        debris_manager.Update(1.0f / 60.0f);
    }
}
```

□ Create usage guide

# Usage Guide

## Getting Started

### 1. Create Voxel World

```cpp
VoxelWorld world;

// Create simple tower
for (int y = 0; y < 10; y++) {
    world.SetVoxel(Vector3(0, y * 0.05, 0), Voxel{BRICK});
}
```

### 2. Initialize Physics

```cpp
BulletPhysicsSystem physics;
physics.Initialize();
physics.CreateGroundPlane();

DebrisManager debris_manager(&physics);
```

### 3. Handle Destruction

```cpp
void OnVoxelDestroyed(Vector3 position) {
    world.RemoveVoxel(position);
    
    // Analyze structure
    StructuralAnalyzer structural;
    auto result = structural.Analyze(world, {position}, 500);
    
    if (result.structure_failed) {
        // Create debris
        for (auto& cluster : result.failed_clusters) {
            Material mat = materials[BRICK];
            debris_manager.AddDebris(cluster, mat);
        }
    }
}
```

### 4. Simulate Each Frame

```cpp
void Update(float deltaTime) {
    physics.Step(deltaTime);
    debris_manager.Update(deltaTime);
    debris_manager.CleanupSettled(&rubble_system);
}
```

## Advanced Topics

### Custom Materials

### Performance Tuning

### Integration with Gameplay
```

**Day 38 - User Documentation:**

```markdown
□ Create troubleshooting guide

# Troubleshooting Guide

## Common Issues

### Issue: Physics simulation is too slow

**Symptoms**: FPS drops below 30 with many debris

**Solutions**:
1. Reduce MAX_DEBRIS limit (default 50)
2. Enable collision filtering (debris vs debris)
3. Increase settling thresholds (faster cleanup)
4. Use LOD system for distant debris

### Issue: Structural analysis takes too long

**Symptoms**: Analysis exceeds 500ms budget

**Solutions**:
1. Reduce max_iterations parameter
2. Enable surface-only optimization
3. Increase influence_radius threshold
4. Use parallel computation

### Issue: Debris doesn't settle

**Symptoms**: Debris keeps moving indefinitely

**Solutions**:
1. Check sleep thresholds (too strict?)
2. Increase damping values
3. Verify ground collision working
4. Check for numerical instability

□ Create FAQ

# Frequently Asked Questions

## Q: Why Bullet instead of PhysX?

A: Bullet is lighter-weight, open-source, and easier to integrate for first version. PhysX offers more features (Blast fracture) but adds complexity.

## Q: Can I use this in my game?

A: Yes! [License details here]

## Q: What are the performance requirements?

A: Minimum: 4-core CPU, 8GB RAM, GPU with OpenGL 4.x
Recommended: 8-core CPU, 16GB RAM, dedicated GPU

## Q: How do I add new materials?

A: See docs/materials.md for guide

## Q: Can debris damage units?

A: Yes! Check docs/gameplay_integration.md

□ Create video tutorials outline

# Video Tutorial Series Outline

## Tutorial 1: Setup & First Collapse (10 min)
- Install dependencies
- Build project
- Run first demo
- Destroy simple tower

## Tutorial 2: Understanding Structural Analysis (15 min)
- How springs work
- Mass calculation
- Failure detection
- Parameter tuning

## Tutorial 3: Physics Integration (15 min)
- Bullet basics
- Convex hull creation
- Debris lifecycle
- Settling detection

## Tutorial 4: Adding Materials (10 min)
- Define new material
- Set properties
- Test fracture patterns

## Tutorial 5: Game Integration (20 min)
- Turn-based timing
- Cover system integration
- Performance optimization
- Best practices
```

**Success Criteria:**
- ✅ Complete README.md
- ✅ Full API documentation
- ✅ Usage guide with examples
- ✅ Troubleshooting guide
- ✅ FAQ
- ✅ Video tutorial outlines

---

### Day 39: Code Cleanup & Comments

**Morning (4 hours):**

```cpp
□ Add comprehensive code comments

/**
 * BulletPhysicsSystem
 * 
 * Manages Bullet Physics dynamics world and provides high-level interface
 * for debris simulation.
 * 
 * Thread-Safety: NOT thread-safe. All calls must be from main thread.
 * 
 * Performance: Step() takes ~5-15ms with 50 active rigid bodies.
 * 
 * Example:
 *   BulletPhysicsSystem physics;
 *   physics.Initialize();
 *   physics.CreateGroundPlane();
 *   
 *   // Each frame
 *   physics.Step(deltaTime);
 */
class BulletPhysicsSystem {
    // ...
};

/**
 * Creates convex hull collision shape from voxel cluster.
 * 
 * @param cluster VoxelCluster to convert
 * @return Bullet convex hull shape (caller must delete)
 * 
 * Performance: ~1-5ms for clusters with < 100 voxels
 * 
 * Note: For clusters > 100 voxels, consider CreateSimplifiedHull()
 */
btConvexHullShape* CreateConvexHullFromCluster(VoxelCluster& cluster);

□ Code cleanup pass

// Remove debug code
// Remove commented-out code
// Fix indentation
// Consistent naming
// Remove magic numbers (use constants)

const float VOXEL_SIZE = 0.05f;  // 5cm voxels
const int MAX_DEBRIS = 50;
const float SETTLING_LINEAR_THRESHOLD = 0.1f;  // m/s
const float SETTLING_ANGULAR_THRESHOLD = 0.2f; // rad/s
const float SETTLING_TIME_REQUIRED = 1.0f;     // seconds

□ Run static analysis

// Use clang-tidy or similar
clang-tidy src/*.cpp -- -Iinclude

// Fix warnings
// Check for:
// - Memory leaks
// - Uninitialized variables
// - Dead code
// - Performance issues
```

**Afternoon (4 hours):**

```cpp
□ Add assertions for debugging

void DebrisManager::AddDebris(VoxelCluster& cluster, Material& material) {
    assert(cluster.voxel_positions.size() > 0 && 
           "Cluster must have at least one voxel");
    assert(cluster.total_mass > 0 && 
           "Cluster must have positive mass");
    assert(physics != nullptr && 
           "Physics system must be initialized");
    
    // ... rest of function
}

□ Add logging system

enum LogLevel {
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARNING,
    LOG_ERROR
};

class Logger {
public:
    static void Log(LogLevel level, const std::string& message);
};

// Usage
Logger::Log(LOG_INFO, "Physics initialized");
Logger::Log(LOG_WARNING, "Debris limit reached, removing oldest");
Logger::Log(LOG_ERROR, "Failed to create convex hull");

□ Final testing pass

// Run all unit tests
./tests --gtest_filter=*

// Run integration tests
./integration_tests

// Run performance benchmarks
./benchmarks

// Memory leak check (valgrind)
valgrind --leak-check=full ./voxel_destruction_demo
```

**Success Criteria:**
- ✅ All code commented
- ✅ No warnings from static analysis
- ✅ All tests passing
- ✅ No memory leaks
- ✅ Code follows style guide

---

### Day 40: Next Steps & Handoff

**Morning (4 hours):**

```cpp
□ Create roadmap for future work

# Future Work Roadmap

## Version 2: Enhanced Features (Months 3-4)

### Track 1 Improvements
- [ ] Max-flow algorithm (more accurate than springs)
- [ ] Buckling detection for tall columns
- [ ] Progressive material weakening
- [ ] Arch and cantilever support

### Track 3 Improvements
- [ ] Compound collision shapes (better accuracy)
- [ ] Material-specific fragmentation algorithms
- [ ] Better debris stacking detection
- [ ] Debris-debris collision (optional)

### Track 2 Integration (Cover System)
- [ ] Dynamic cover detection from rubble
- [ ] Cover degradation over time
- [ ] Line-of-sight integration
- [ ] Flanking mechanics

## Version 3: Gameplay Features (Months 5-6)

### Track 4 (Turn-Based Optimization)
- [ ] Multi-threaded structural analysis
- [ ] Aggressive caching systems
- [ ] LOD for distant structures
- [ ] Budget management system

### Track 5 (Balance & Polish)
- [ ] AI destruction tactics
- [ ] Munitions scarcity
- [ ] Material damage curves
- [ ] Playtesting methodology

### Audio
- [ ] Impact sounds (material-specific)
- [ ] Collapse ambience
- [ ] Debris settling sounds

### VFX
- [ ] Improved particle effects
- [ ] Smoke/dust clouds
- [ ] Impact decals
- [ ] Lighting effects

## Version 4: Advanced Features (Months 7+)

### PhysX Migration
- [ ] Replace Bullet with PhysX
- [ ] PhysX Blast integration
- [ ] GPU acceleration
- [ ] Advanced fracture patterns

### Networked Multiplayer
- [ ] Deterministic physics
- [ ] State synchronization
- [ ] Debris culling for network

### Editor Tools
- [ ] Structure designer
- [ ] Material editor
- [ ] Test scenario creator

□ Document known limitations

# Known Limitations

## Structural Analysis (Track 1)

1. **No buckling detection**
   - Tall thin columns don't realistically fail
   - Workaround: Manually set max_displacement lower

2. **Spring system approximation**
   - Not as accurate as FEM or max-flow
   - Good enough for gameplay, not engineering

3. **Surface-only analysis**
   - Interior voxels ignored (performance)
   - Large solid blocks may have issues

## Physics (Track 3)

1. **Simplified fracture**
   - No PhysX Blast (manual patterns)
   - Less realistic than dedicated system

2. **Debris limit (50)**
   - Performance constraint
   - May not handle massive collapses

3. **No debris-debris collision**
   - Intentionally disabled (performance)
   - Debris phases through each other

4. **Ground plane only**
   - Complex terrain not supported yet
   - Chunked collision system needed

## General

1. **Single-threaded**
   - No multi-threading yet
   - Future optimization opportunity

2. **No save/load for mid-collapse**
   - Can only save stable states
   - Active physics not serializable

3. **Limited material variety**
   - Only 5 materials implemented
   - Easy to add more
```

**Afternoon (4 hours):**

```cpp
□ Create contributor guide

# Contributing Guide

## Code Style

- Indent: 4 spaces
- Braces: Same line
- Naming: snake_case for functions, PascalCase for classes
- Comments: Doxygen style

## Pull Request Process

1. Fork repository
2. Create feature branch
3. Write tests
4. Ensure all tests pass
5. Update documentation
6. Submit PR with description

## Testing Requirements

- Unit tests for new functions
- Integration tests for new features
- Performance benchmarks if relevant
- All existing tests must pass

## Performance Guidelines

- Target: 60 FPS with 10K voxels
- Profile before optimizing
- Document performance characteristics
- Avoid premature optimization

□ Package final deliverables

project/
├── README.md
├── LICENSE
├── CMakeLists.txt
├── src/
│   ├── voxel/
│   ├── structural/
│   ├── physics/
│   └── main.cpp
├── include/
├── tests/
├── docs/
│   ├── api.md
│   ├── usage.md
│   ├── performance.md
│   ├── troubleshooting.md
│   └── roadmap.md
├── examples/
│   ├── simple_tower.cpp
│   ├── building_collapse.cpp
│   └── material_comparison.cpp
├── scenarios/
│   ├── tower.vxl
│   ├── building.vxl
│   └── bridge.vxl
├── media/
│   ├── showcase_video.mp4
│   ├── collapse.gif
│   └── screenshots/
└── benchmarks/

□ Create release notes

# Release Notes - v1.0.0

## Features

- Complete voxel destruction pipeline
- Structural integrity analysis (Track 1)
- Physics simulation with Bullet (Track 3)
- Material system (wood, brick, concrete, glass, metal)
- Debris lifecycle management
- Rubble generation
- Particle effects
- Turn-based timing support

## Performance

- Structural analysis: < 500ms for 50K voxels
- Physics simulation: 30+ FPS with 50 debris
- Memory usage: < 500 MB typical

## Known Issues

- No debris-debris collision
- Ground plane only (no complex terrain)
- Single-threaded

## Breaking Changes

N/A (initial release)

## Upgrade Guide

N/A (initial release)

□ Create final presentation

# Voxel Destruction Engine - Final Presentation

## Slide 1: Overview
- 8-week sprint completed
- Track 1 (Structural) + Track 3 (Physics) implemented
- Working destruction demo

## Slide 2: Architecture
[Diagram showing data flow]

## Slide 3: Track 1 - Structural Analysis
- Displacement-based springs
- < 500ms for typical structures
- Correctly identifies failures

## Slide 4: Track 3 - Physics Simulation
- Bullet Physics integration
- Material-specific behaviors
- Debris lifecycle management

## Slide 5: Demo
[Embedded video or live demo]

## Slide 6: Performance
[Charts and metrics]

## Slide 7: Next Steps
[Roadmap for v2.0]

## Slide 8: Q&A
```

**Success Criteria:**
- ✅ Roadmap documented
- ✅ Known limitations listed
- ✅ Contributor guide complete
- ✅ Final deliverables packaged
- ✅ Release notes written
- ✅ Presentation ready

---

### Week 8 Deliverable

**What you should have:**

1. **Complete Documentation:**
   - README.md with quick start
   - Full API reference
   - Usage guide with examples
   - Troubleshooting guide
   - FAQ
   - Video tutorial outlines

2. **Showcase Materials:**
   - 6-minute showcase video (1080p60)
   - Multiple GIFs for documentation
   - Screenshots of key features
   - Performance benchmarks

3. **Clean Codebase:**
   - All code commented
   - No warnings
   - All tests passing
   - No memory leaks
   - Consistent style

4. **Future Planning:**
   - Roadmap for v2-v4
   - Known limitations documented
   - Contributor guide
   - Release notes

5. **Deliverable Package:**
   - Organized project structure
   - Examples and scenarios
   - Complete build system
   - Ready for handoff

**Final Presentation (30 minutes):**
- Overview (5 min)
- Live demo (10 min)
- Technical deep-dive (10 min)
- Q&A (5 min)

---

## Sprint Summary

### What Was Accomplished

**Weeks 1-2: Foundation**
- Complete voxel world system
- Rendering and interaction
- Spatial hashing (10x speedup)
- Clustering algorithms

**Weeks 3-4: Track 1**
- Structural integrity analysis
- Displacement-based springs
- Failure detection
- Parameter tuning system

**Weeks 5-6: Track 3**
- Bullet Physics integration
- Debris simulation
- Material-specific behaviors
- Rubble generation

**Week 7: Integration**
- Complete destruction pipeline
- Turn-based timing
- Visual feedback systems
- Test scenarios

**Week 8: Polish**
- Showcase video
- Complete documentation
- Code cleanup
- Future roadmap

### Key Metrics

**Performance Achieved:**
- Track 1: 180ms (target: < 500ms) ✓
- Track 3: 35 FPS with 50 debris (target: 30+) ✓
- Total collapse: 2.8s (target: < 5s) ✓

**Code Quality:**
- 50+ unit tests
- 10+ integration tests
- 0 memory leaks
- 0 compiler warnings

**Documentation:**
- 100+ pages of docs
- 6-minute showcase video
- 5 tutorial outlines
- Complete API reference

### Next Steps

**Immediate (Week 9):**
- Address any feedback
- Fix remaining bugs
- Polish demo

**Short-term (Months 3-4):**
- Implement Track 2 (Cover System)
- Optimize Track 1 (max-flow algorithm)
- Enhanced fracture patterns

**Long-term (Months 5+):**
- Track 4 (Optimization)
- Track 5 (Balance & AI)
- PhysX migration
- Multiplayer support

---

**END OF 8-WEEK SPRINT PLAN (WEEKS 5-8)**

