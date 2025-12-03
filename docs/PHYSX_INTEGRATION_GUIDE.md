# PhysX Integration Guide

**Day 19: Integration guide for connecting StructuralAnalyzer with NVIDIA PhysX**

## Table of Contents

1. [Overview](#overview)
2. [Integration Architecture](#integration-architecture)
3. [Basic Integration](#basic-integration)
4. [Advanced Patterns](#advanced-patterns)
5. [Performance Optimization](#performance-optimization)
6. [Troubleshooting](#troubleshooting)

## Overview

The `StructuralAnalyzer` determines **what** should collapse, while PhysX simulates **how** it falls. This guide shows how to connect them.

### Integration Flow

```
1. Apply damage to VoxelWorld
2. StructuralAnalyzer.Analyze() → Find failed clusters
3. Convert failed clusters to PhysX rigid bodies
4. Simulate physics with PhysX
5. Update VoxelWorld based on physics results
```

### Key Concepts

- **Cluster**: Group of failed voxels that should fall together
- **Rigid Body**: PhysX representation of a falling cluster
- **Collision Shape**: PhysX convex hull or compound shape for cluster
- **Material**: PhysX material with friction/restitution from voxel materials

## Integration Architecture

### High-Level Architecture

```
┌─────────────────┐
│   VoxelWorld    │
│  (Game State)   │
└────────┬────────┘
         │ Damage event
         ▼
┌─────────────────────┐
│ StructuralAnalyzer  │
│  (Determine what    │
│   should collapse)  │
└────────┬────────────┘
         │ Failed clusters
         ▼
┌─────────────────────┐
│  ClusterConverter   │
│  (Voxels → PhysX)   │
└────────┬────────────┘
         │ Rigid bodies
         ▼
┌─────────────────────┐
│   PhysX Scene       │
│  (Simulate physics) │
└────────┬────────────┘
         │ Physics results
         ▼
┌─────────────────────┐
│  VoxelWorld Update  │
│  (Apply results)    │
└─────────────────────┘
```

### Component Responsibilities

**StructuralAnalyzer:**
- Determines structural failures
- Groups failed voxels into clusters
- Provides mass, center of mass, bounding box

**ClusterConverter:**
- Converts voxel clusters to PhysX shapes
- Creates appropriate collision geometry
- Sets material properties

**PhysX Scene:**
- Simulates rigid body dynamics
- Handles collisions
- Applies forces and constraints

**Integration Layer:**
- Manages lifecycle (create/destroy bodies)
- Synchronizes voxel world with physics
- Handles cleanup after simulation

## Basic Integration

### Step 1: Setup PhysX (Assumed)

```cpp
// Assumed you have PhysX initialized
physx::PxPhysics* gPhysics;
physx::PxScene* gScene;
physx::PxMaterial* gDefaultMaterial;
```

### Step 2: Create Integration Class

```cpp
#include "StructuralAnalyzer.h"
#include "VoxelWorld.h"
#include <PxPhysicsAPI.h>

class VoxelPhysicsIntegration {
public:
    VoxelPhysicsIntegration(
        VoxelWorld& world,
        physx::PxPhysics* physics,
        physx::PxScene* scene
    ) : voxel_world(world)
      , px_physics(physics)
      , px_scene(scene)
    {
        analyzer.LoadParameters("config/optimal_params.ini");
    }

    void ApplyDamage(const std::vector<Vector3>& damaged_positions) {
        // 1. Analyze structural integrity
        auto result = analyzer.Analyze(voxel_world, damaged_positions);

        if (!result.structure_failed) {
            return;  // Structure is stable
        }

        // 2. Convert failed clusters to physics bodies
        for (const auto& cluster : result.failed_clusters) {
            CreateRigidBodyFromCluster(cluster);

            // 3. Remove voxels from world
            for (const auto& pos : cluster.voxel_positions) {
                voxel_world.RemoveVoxel(pos);
            }
        }

        // 4. Clear analyzer cache since world changed
        analyzer.ClearMassCache();
    }

private:
    VoxelWorld& voxel_world;
    physx::PxPhysics* px_physics;
    physx::PxScene* px_scene;
    StructuralAnalyzer analyzer;
    std::vector<physx::PxRigidDynamic*> active_bodies;

    void CreateRigidBodyFromCluster(const VoxelCluster& cluster);
};
```

### Step 3: Convert Cluster to PhysX Body

```cpp
void VoxelPhysicsIntegration::CreateRigidBodyFromCluster(
    const VoxelCluster& cluster)
{
    if (cluster.voxel_positions.empty()) {
        return;
    }

    // Create transform at cluster center of mass
    physx::PxTransform transform(
        physx::PxVec3(
            cluster.center_of_mass.x,
            cluster.center_of_mass.y,
            cluster.center_of_mass.z
        )
    );

    // Create rigid dynamic body
    physx::PxRigidDynamic* body = px_physics->createRigidDynamic(transform);

    // Add collision shape (simplified: box shape)
    Vector3 extents = cluster.bounds_max - cluster.bounds_min;
    physx::PxBoxGeometry box(
        extents.x * 0.5f,
        extents.y * 0.5f,
        extents.z * 0.5f
    );

    physx::PxShape* shape = px_physics->createShape(
        box,
        *gDefaultMaterial  // Use appropriate material
    );

    body->attachShape(*shape);
    shape->release();

    // Set mass (from cluster)
    physx::PxRigidBodyExt::setMassAndUpdateInertia(*body, cluster.total_mass);

    // Add to scene
    px_scene->addActor(*body);
    active_bodies.push_back(body);

    std::cout << "Created rigid body: "
              << cluster.voxel_positions.size() << " voxels, "
              << cluster.total_mass << " kg\n";
}
```

### Step 4: Use in Game Loop

```cpp
// In your game loop
void Update(float delta_time) {
    // Handle damage events
    if (explosion_occurred) {
        std::vector<Vector3> damaged = GetDamagedVoxels(explosion_pos);
        physics_integration.ApplyDamage(damaged);
    }

    // Step PhysX
    px_scene->simulate(delta_time);
    px_scene->fetchResults(true);

    // Update visuals, cleanup finished bodies, etc.
}
```

## Advanced Patterns

### Pattern 1: Accurate Collision Shapes

Instead of simple boxes, create convex hulls from voxel positions.

```cpp
physx::PxConvexMesh* CreateConvexHullFromCluster(
    const VoxelCluster& cluster)
{
    // Convert voxel positions to PxVec3
    std::vector<physx::PxVec3> vertices;
    vertices.reserve(cluster.voxel_positions.size());

    for (const auto& pos : cluster.voxel_positions) {
        vertices.push_back(physx::PxVec3(pos.x, pos.y, pos.z));
    }

    // Create convex mesh description
    physx::PxConvexMeshDesc convexDesc;
    convexDesc.points.count = static_cast<physx::PxU32>(vertices.size());
    convexDesc.points.stride = sizeof(physx::PxVec3);
    convexDesc.points.data = vertices.data();
    convexDesc.flags = physx::PxConvexFlag::eCOMPUTE_CONVEX;

    // Cook the convex mesh
    physx::PxDefaultMemoryOutputStream buf;
    physx::PxConvexMeshCookingResult::Enum result;

    if (!PxCookConvexMesh(convexDesc, buf, &result)) {
        std::cerr << "Convex mesh cooking failed!\n";
        return nullptr;
    }

    physx::PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
    return px_physics->createConvexMesh(input);
}

void CreateRigidBodyWithConvexHull(const VoxelCluster& cluster) {
    physx::PxConvexMesh* convexMesh = CreateConvexHullFromCluster(cluster);

    if (!convexMesh) {
        // Fallback to box
        CreateRigidBodyFromCluster(cluster);
        return;
    }

    // Create body
    physx::PxTransform transform(
        physx::PxVec3(
            cluster.center_of_mass.x,
            cluster.center_of_mass.y,
            cluster.center_of_mass.z
        )
    );

    physx::PxRigidDynamic* body = px_physics->createRigidDynamic(transform);

    // Attach convex shape
    physx::PxConvexMeshGeometry geom(convexMesh);
    physx::PxShape* shape = px_physics->createShape(
        geom,
        *gDefaultMaterial
    );

    body->attachShape(*shape);
    shape->release();

    // Set mass
    physx::PxRigidBodyExt::setMassAndUpdateInertia(*body, cluster.total_mass);

    // Add to scene
    px_scene->addActor(*body);

    convexMesh->release();
}
```

### Pattern 2: Material-Based Physics Properties

Map voxel materials to PhysX materials.

```cpp
class MaterialPhysicsMapper {
private:
    std::unordered_map<uint8_t, physx::PxMaterial*> material_map;

public:
    MaterialPhysicsMapper(physx::PxPhysics* physics) {
        // Create PhysX materials for each voxel material
        // Concrete: High friction, low restitution
        material_map[MaterialDatabase::CONCRETE] =
            physics->createMaterial(0.8f, 0.8f, 0.1f);

        // Wood: Medium friction, low restitution
        material_map[MaterialDatabase::WOOD] =
            physics->createMaterial(0.6f, 0.6f, 0.2f);

        // Steel: Low friction, low restitution
        material_map[MaterialDatabase::STEEL] =
            physics->createMaterial(0.3f, 0.3f, 0.05f);

        // Brick: High friction, medium restitution
        material_map[MaterialDatabase::BRICK] =
            physics->createMaterial(0.7f, 0.7f, 0.3f);
    }

    physx::PxMaterial* GetMaterial(uint8_t voxel_material_id) {
        auto it = material_map.find(voxel_material_id);
        if (it != material_map.end()) {
            return it->second;
        }
        return material_map[MaterialDatabase::CONCRETE];  // Default
    }

    ~MaterialPhysicsMapper() {
        for (auto& pair : material_map) {
            pair.second->release();
        }
    }
};

void CreateRigidBodyWithMaterial(
    const VoxelCluster& cluster,
    MaterialPhysicsMapper& material_mapper)
{
    // Determine dominant material in cluster
    uint8_t dominant_material = cluster.voxel_positions.empty() ?
        MaterialDatabase::CONCRETE :
        voxel_world.GetVoxel(cluster.voxel_positions[0]).material_id;

    physx::PxMaterial* px_material = material_mapper.GetMaterial(dominant_material);

    // Create body with appropriate material...
    // (rest of implementation)
}
```

### Pattern 3: Progressive Simulation

Simulate clusters in stages to spread computation.

```cpp
class ProgressivePhysicsSimulation {
private:
    struct PendingCluster {
        VoxelCluster cluster;
        float spawn_delay;
    };

    std::queue<PendingCluster> pending_clusters;
    float time_accumulator = 0.0f;

public:
    void QueueCluster(const VoxelCluster& cluster, float delay = 0.0f) {
        pending_clusters.push({cluster, delay});
    }

    void Update(float delta_time) {
        time_accumulator += delta_time;

        // Spawn clusters that are ready
        while (!pending_clusters.empty()) {
            auto& pending = pending_clusters.front();

            if (pending.spawn_delay <= 0.0f) {
                CreateRigidBodyFromCluster(pending.cluster);
                pending_clusters.pop();
            } else {
                pending.spawn_delay -= delta_time;
                break;  // Wait for next frame
            }
        }

        // Simulate physics
        px_scene->simulate(delta_time);
        px_scene->fetchResults(true);
    }
};
```

### Pattern 4: Cluster Merging

Merge small adjacent clusters to reduce physics body count.

```cpp
std::vector<VoxelCluster> MergeSmallClusters(
    const std::vector<VoxelCluster>& clusters,
    size_t min_voxels_per_cluster = 5,
    float merge_distance = 1.0f)
{
    std::vector<VoxelCluster> merged;

    for (const auto& cluster : clusters) {
        if (cluster.voxel_positions.size() >= min_voxels_per_cluster) {
            merged.push_back(cluster);
            continue;
        }

        // Try to merge with nearby cluster
        bool merged_with_existing = false;

        for (auto& existing : merged) {
            float dist = cluster.center_of_mass.Distance(existing.center_of_mass);

            if (dist < merge_distance) {
                // Merge clusters
                existing.voxel_positions.insert(
                    existing.voxel_positions.end(),
                    cluster.voxel_positions.begin(),
                    cluster.voxel_positions.end()
                );

                // Recalculate center of mass
                Vector3 sum = Vector3::Zero();
                for (const auto& pos : existing.voxel_positions) {
                    sum = sum + pos;
                }
                existing.center_of_mass = sum * (1.0f / existing.voxel_positions.size());

                existing.total_mass += cluster.total_mass;
                merged_with_existing = true;
                break;
            }
        }

        if (!merged_with_existing) {
            merged.push_back(cluster);
        }
    }

    std::cout << "Merged " << clusters.size() << " clusters into "
              << merged.size() << " clusters\n";

    return merged;
}
```

## Performance Optimization

### Optimization 1: LOD for Collision Shapes

Use simpler shapes for distant or small clusters.

```cpp
enum class CollisionLOD {
    HIGH,    // Convex hull
    MEDIUM,  // Compound boxes
    LOW      // Single box
};

CollisionLOD DetermineCollisionLOD(
    const VoxelCluster& cluster,
    const Vector3& camera_pos)
{
    float dist = cluster.center_of_mass.Distance(camera_pos);
    size_t voxel_count = cluster.voxel_positions.size();

    if (dist < 10.0f && voxel_count > 20) {
        return CollisionLOD::HIGH;
    } else if (dist < 50.0f && voxel_count > 5) {
        return CollisionLOD::MEDIUM;
    } else {
        return CollisionLOD::LOW;
    }
}

void CreateRigidBodyWithLOD(
    const VoxelCluster& cluster,
    const Vector3& camera_pos)
{
    CollisionLOD lod = DetermineCollisionLOD(cluster, camera_pos);

    switch (lod) {
        case CollisionLOD::HIGH:
            CreateRigidBodyWithConvexHull(cluster);
            break;
        case CollisionLOD::MEDIUM:
            CreateRigidBodyWithCompound(cluster);
            break;
        case CollisionLOD::LOW:
            CreateRigidBodyFromCluster(cluster);  // Simple box
            break;
    }
}
```

### Optimization 2: Body Pooling

Reuse PhysX bodies instead of creating/destroying.

```cpp
class RigidBodyPool {
private:
    std::vector<physx::PxRigidDynamic*> available_bodies;
    std::vector<physx::PxRigidDynamic*> active_bodies;

public:
    physx::PxRigidDynamic* Acquire() {
        if (!available_bodies.empty()) {
            auto body = available_bodies.back();
            available_bodies.pop_back();
            active_bodies.push_back(body);
            return body;
        }

        // Create new body
        physx::PxRigidDynamic* body = px_physics->createRigidDynamic(
            physx::PxTransform(physx::PxIdentity)
        );
        active_bodies.push_back(body);
        return body;
    }

    void Release(physx::PxRigidDynamic* body) {
        // Remove from active
        auto it = std::find(active_bodies.begin(), active_bodies.end(), body);
        if (it != active_bodies.end()) {
            active_bodies.erase(it);
        }

        // Reset and return to pool
        body->setGlobalPose(physx::PxTransform(physx::PxIdentity));
        body->setLinearVelocity(physx::PxVec3(0, 0, 0));
        body->setAngularVelocity(physx::PxVec3(0, 0, 0));
        available_bodies.push_back(body);
    }
};
```

### Optimization 3: Async Analysis + Physics

Run structural analysis and physics in parallel.

```cpp
class AsyncIntegration {
private:
    std::future<AnalysisResult> pending_analysis;
    std::queue<AnalysisResult> pending_spawns;

public:
    void StartAnalysis(
        VoxelWorld& world,
        const std::vector<Vector3>& damaged)
    {
        pending_analysis = std::async(std::launch::async, [&world, damaged]() {
            StructuralAnalyzer analyzer;
            analyzer.LoadParameters("config/optimal_params.ini");
            return analyzer.Analyze(world, damaged);
        });
    }

    void Update(float delta_time) {
        // Check if analysis is complete
        if (pending_analysis.valid() &&
            pending_analysis.wait_for(std::chrono::seconds(0)) ==
            std::future_status::ready)
        {
            auto result = pending_analysis.get();
            if (result.structure_failed) {
                pending_spawns.push(result);
            }
        }

        // Spawn physics bodies from completed analyses
        if (!pending_spawns.empty()) {
            auto result = pending_spawns.front();
            pending_spawns.pop();

            for (const auto& cluster : result.failed_clusters) {
                CreateRigidBodyFromCluster(cluster);
            }
        }

        // Simulate physics (runs in parallel with analysis)
        px_scene->simulate(delta_time);
        px_scene->fetchResults(true);
    }
};
```

## Troubleshooting

### Issue: Physics Bodies Explode

**Cause:** Center of mass offset or extreme initial velocities

**Solution:**
```cpp
// Ensure COM is correct
body->setCMassLocalPose(physx::PxTransform(physx::PxIdentity));

// Start with zero velocity
body->setLinearVelocity(physx::PxVec3(0, 0, 0));
body->setAngularVelocity(physx::PxVec3(0, 0, 0));

// Use smaller timestep
px_scene->setSimulationTimestep(1.0f / 120.0f);
```

### Issue: Bodies Fall Through Ground

**Cause:** Missing or incorrect ground collision

**Solution:**
```cpp
// Ensure ground plane exists
physx::PxRigidStatic* groundPlane = px_physics->createRigidStatic(
    physx::PxTransform(physx::PxVec3(0, 0, 0))
);

physx::PxShape* groundShape = px_physics->createShape(
    physx::PxPlaneGeometry(),
    *gDefaultMaterial
);

groundPlane->attachShape(*groundShape);
groundShape->release();
px_scene->addActor(*groundPlane);
```

### Issue: Too Many Bodies Hurt Performance

**Solution:**
1. Merge small clusters
2. Use simpler collision shapes
3. Implement body pooling
4. Sleep inactive bodies
5. Remove bodies that fall too far

```cpp
void CleanupDistantBodies(const Vector3& world_center, float max_distance) {
    for (auto it = active_bodies.begin(); it != active_bodies.end();) {
        physx::PxTransform pose = (*it)->getGlobalPose();
        Vector3 pos(pose.p.x, pose.p.y, pose.p.z);

        if (pos.Distance(world_center) > max_distance) {
            px_scene->removeActor(**it);
            (*it)->release();
            it = active_bodies.erase(it);
        } else {
            ++it;
        }
    }
}
```

## Complete Example

Here's a complete integration example:

```cpp
// VoxelPhysicsSystem.h
class VoxelPhysicsSystem {
public:
    VoxelPhysicsSystem(
        VoxelWorld& world,
        physx::PxPhysics* physics,
        physx::PxScene* scene
    );

    void ApplyExplosion(Vector3 position, float radius, float force);
    void Update(float delta_time);
    void Cleanup();

private:
    VoxelWorld& voxel_world;
    physx::PxPhysics* px_physics;
    physx::PxScene* px_scene;
    StructuralAnalyzer analyzer;
    MaterialPhysicsMapper material_mapper;
    RigidBodyPool body_pool;
    std::vector<physx::PxRigidDynamic*> active_bodies;

    void CreatePhysicsFromCluster(const VoxelCluster& cluster);
};

// VoxelPhysicsSystem.cpp
void VoxelPhysicsSystem::ApplyExplosion(
    Vector3 position,
    float radius,
    float force)
{
    // 1. Damage voxels in explosion radius
    auto nearby = voxel_world.GetVoxelsInRadius(position, radius);
    std::vector<Vector3> damaged;

    for (const auto& pos : nearby) {
        float dist = pos.Distance(position);
        if (dist < radius) {
            voxel_world.RemoveVoxel(pos);
            damaged.push_back(pos);
        }
    }

    // 2. Analyze structural integrity
    auto result = analyzer.Analyze(voxel_world, damaged);

    if (!result.structure_failed) {
        return;
    }

    // 3. Create physics bodies
    for (const auto& cluster : result.failed_clusters) {
        CreatePhysicsFromCluster(cluster);

        // Remove voxels
        for (const auto& pos : cluster.voxel_positions) {
            voxel_world.RemoveVoxel(pos);
        }
    }

    // 4. Apply explosion force
    for (auto* body : active_bodies) {
        physx::PxVec3 body_pos = body->getGlobalPose().p;
        Vector3 dir = Vector3(body_pos.x, body_pos.y, body_pos.z) - position;
        float dist = dir.Length();

        if (dist < radius) {
            dir = dir * (1.0f / dist);  // Normalize
            float force_magnitude = force * (1.0f - dist / radius);

            body->addForce(
                physx::PxVec3(dir.x, dir.y, dir.z) * force_magnitude,
                physx::PxForceMode::eIMPULSE
            );
        }
    }

    analyzer.ClearMassCache();
}
```

## See Also

- [API Documentation](STRUCTURAL_ANALYZER_API.md) - Complete API reference
- [Usage Guide](USAGE_GUIDE.md) - Practical usage examples
- [Parameter Tuning Guide](PARAMETER_TUNING_GUIDE.md) - Optimize parameters
- [PhysX Documentation](https://nvidia-omniverse.github.io/PhysX/physx/5.1.1/) - Official PhysX docs
