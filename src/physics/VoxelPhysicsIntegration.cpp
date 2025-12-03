#include "VoxelPhysicsIntegration.h"
#include <iostream>
#include <algorithm>

VoxelPhysicsIntegration::VoxelPhysicsIntegration(IPhysicsEngine* engine, VoxelWorld* world)
    : physics_engine(engine)
    , voxel_world(world)
    , voxel_density(1000.0f)  // Water density (1000 kg/m³)
    , friction(0.5f)
    , restitution(0.3f)
{
    if (!physics_engine) {
        throw std::runtime_error("VoxelPhysicsIntegration: physics_engine cannot be null");
    }

    std::cout << "[VoxelPhysicsIntegration] Initialized with "
              << physics_engine->GetEngineName() << "\n";
}

VoxelPhysicsIntegration::~VoxelPhysicsIntegration() {
    ClearDebris();
    std::cout << "[VoxelPhysicsIntegration] Shutdown complete\n";
}

// ===== Debris Management =====

int VoxelPhysicsIntegration::SpawnDebris(const std::vector<VoxelCluster>& clusters) {
    int spawned_count = 0;

    for (const auto& cluster : clusters) {
        // Skip if already spawned
        if (IsClusterSpawned(cluster.cluster_id)) {
            continue;
        }

        // Skip empty clusters
        if (cluster.voxel_positions.empty()) {
            continue;
        }

        // Calculate physics properties
        Vector3 center_of_mass = CalculateCenterOfMass(cluster);
        float mass = CalculateMass(cluster);

        // Create collision shape (convex hull from voxels)
        CollisionShapeHandle shape = CreateConvexHullFromVoxels(cluster);
        if (!shape) {
            // Fallback to bounding box if convex hull fails
            shape = CreateBoundingBoxFromVoxels(cluster);
        }

        if (!shape) {
            std::cerr << "[VoxelPhysicsIntegration] Failed to create shape for cluster "
                      << cluster.cluster_id << "\n";
            continue;
        }

        // Create rigid body
        RigidBodyDesc desc;
        desc.transform.position = center_of_mass;
        desc.transform.rotation = Quaternion::Identity();
        desc.mass = mass;
        desc.linear_damping = 0.1f;   // Slight air resistance
        desc.angular_damping = 0.1f;  // Slight rotational damping
        desc.friction = friction;
        desc.restitution = restitution;
        desc.kinematic = false;  // Dynamic body

        PhysicsBodyHandle body = physics_engine->CreateRigidBody(desc);
        if (!body) {
            std::cerr << "[VoxelPhysicsIntegration] Failed to create body for cluster "
                      << cluster.cluster_id << "\n";
            physics_engine->DestroyShape(shape);
            continue;
        }

        // Attach shape to body
        physics_engine->AttachShape(body, shape);

        // Track debris
        size_t debris_index = debris_bodies.size();
        debris_bodies.emplace_back(body, shape, cluster.cluster_id, cluster.voxel_positions.size());
        cluster_to_debris[cluster.cluster_id] = debris_index;

        spawned_count++;
    }

    if (spawned_count > 0) {
        std::cout << "[VoxelPhysicsIntegration] Spawned " << spawned_count
                  << " debris bodies (total: " << debris_bodies.size() << ")\n";
    }

    return spawned_count;
}

void VoxelPhysicsIntegration::Step(float deltaTime) {
    if (!physics_engine) {
        return;
    }

    // Step physics simulation
    physics_engine->Step(deltaTime);

    // Optional: Sync debris positions back to voxel world
    // (If you want to visualize debris movement in the voxel grid)
    // For now, we just let physics engine handle the simulation
}

void VoxelPhysicsIntegration::ClearDebris() {
    if (!physics_engine) {
        return;
    }

    // Destroy all debris bodies and shapes
    for (auto& debris : debris_bodies) {
        physics_engine->DestroyRigidBody(debris.body);
        physics_engine->DestroyShape(debris.shape);
    }

    debris_bodies.clear();
    cluster_to_debris.clear();

    std::cout << "[VoxelPhysicsIntegration] Cleared all debris\n";
}

int VoxelPhysicsIntegration::RemoveOutOfBoundsDebris(float min_y) {
    if (!physics_engine) {
        return 0;
    }

    int removed_count = 0;
    std::vector<size_t> indices_to_remove;

    // Find debris below threshold
    for (size_t i = 0; i < debris_bodies.size(); i++) {
        Transform t = physics_engine->GetBodyTransform(debris_bodies[i].body);
        if (t.position.y < min_y) {
            indices_to_remove.push_back(i);
        }
    }

    // Remove in reverse order to maintain valid indices
    for (auto it = indices_to_remove.rbegin(); it != indices_to_remove.rend(); ++it) {
        size_t index = *it;
        auto& debris = debris_bodies[index];

        // Destroy physics objects
        physics_engine->DestroyRigidBody(debris.body);
        physics_engine->DestroyShape(debris.shape);

        // Remove from tracking
        cluster_to_debris.erase(debris.cluster_id);

        // Remove from vector (swap with last element for O(1) removal)
        if (index < debris_bodies.size() - 1) {
            debris_bodies[index] = debris_bodies.back();
            // Update tracking for swapped element
            cluster_to_debris[debris_bodies[index].cluster_id] = index;
        }
        debris_bodies.pop_back();

        removed_count++;
    }

    if (removed_count > 0) {
        std::cout << "[VoxelPhysicsIntegration] Removed " << removed_count
                  << " out-of-bounds debris (remaining: " << debris_bodies.size() << ")\n";
    }

    return removed_count;
}

// ===== Configuration =====

void VoxelPhysicsIntegration::SetVoxelDensity(float density) {
    voxel_density = density;
}

void VoxelPhysicsIntegration::SetDebrisMaterial(float friction_val, float restitution_val) {
    friction = friction_val;
    restitution = restitution_val;
}

// ===== Queries =====

int VoxelPhysicsIntegration::GetDebrisCount() const {
    return static_cast<int>(debris_bodies.size());
}

bool VoxelPhysicsIntegration::IsClusterSpawned(uint32_t cluster_id) const {
    return cluster_to_debris.find(cluster_id) != cluster_to_debris.end();
}

// ===== Helper Functions =====

Vector3 VoxelPhysicsIntegration::CalculateCenterOfMass(const VoxelCluster& cluster) {
    if (cluster.voxel_positions.empty()) {
        return Vector3::Zero();
    }

    Vector3 sum = Vector3::Zero();
    for (const auto& position : cluster.voxel_positions) {
        sum = sum + position;
    }

    return sum / static_cast<float>(cluster.voxel_positions.size());
}

float VoxelPhysicsIntegration::CalculateMass(const VoxelCluster& cluster) {
    // Assume each voxel is 1m³ cube
    float voxel_volume = 1.0f;  // m³
    float total_volume = static_cast<float>(cluster.voxel_positions.size()) * voxel_volume;
    float mass = total_volume * voxel_density;

    // Clamp mass to reasonable range (prevent physics instabilities)
    const float min_mass = 0.1f;   // kg
    const float max_mass = 10000.0f;  // kg
    return std::max(min_mass, std::min(mass, max_mass));
}

CollisionShapeHandle VoxelPhysicsIntegration::CreateConvexHullFromVoxels(const VoxelCluster& cluster) {
    if (cluster.voxel_positions.empty()) {
        return nullptr;
    }

    // Convert voxel positions to vertices for convex hull
    ConvexHullShapeDesc desc;
    desc.vertices.reserve(cluster.voxel_positions.size());

    // Calculate center of mass for local coordinates
    Vector3 center = CalculateCenterOfMass(cluster);

    // Add voxel corners to convex hull (simplified: use voxel centers)
    for (const auto& position : cluster.voxel_positions) {
        // Use voxel position relative to center of mass
        Vector3 local_pos = position - center;
        desc.vertices.push_back(local_pos);

        // For better hull, could add all 8 corners of each voxel cube
        // But for now, just use center to keep it simple
    }

    // Remove duplicates (if any)
    std::sort(desc.vertices.begin(), desc.vertices.end(),
              [](const Vector3& a, const Vector3& b) {
                  if (a.x != b.x) return a.x < b.x;
                  if (a.y != b.y) return a.y < b.y;
                  return a.z < b.z;
              });
    auto last = std::unique(desc.vertices.begin(), desc.vertices.end(),
                           [](const Vector3& a, const Vector3& b) {
                               return (a - b).Length() < 0.01f;
                           });
    desc.vertices.erase(last, desc.vertices.end());

    // Need at least 4 vertices for a convex hull
    if (desc.vertices.size() < 4) {
        return nullptr;
    }

    return physics_engine->CreateConvexHullShape(desc);
}

CollisionShapeHandle VoxelPhysicsIntegration::CreateBoundingBoxFromVoxels(const VoxelCluster& cluster) {
    if (cluster.voxel_positions.empty()) {
        return nullptr;
    }

    // Calculate axis-aligned bounding box
    Vector3 min_pos = cluster.voxel_positions[0];
    Vector3 max_pos = cluster.voxel_positions[0];

    for (const auto& position : cluster.voxel_positions) {
        min_pos.x = std::min(min_pos.x, position.x);
        min_pos.y = std::min(min_pos.y, position.y);
        min_pos.z = std::min(min_pos.z, position.z);

        max_pos.x = std::max(max_pos.x, position.x);
        max_pos.y = std::max(max_pos.y, position.y);
        max_pos.z = std::max(max_pos.z, position.z);
    }

    // Calculate half extents (box is centered at origin in local space)
    Vector3 half_extents = (max_pos - min_pos) * 0.5f;

    // Ensure minimum size (prevent degenerate boxes)
    half_extents.x = std::max(half_extents.x, 0.1f);
    half_extents.y = std::max(half_extents.y, 0.1f);
    half_extents.z = std::max(half_extents.z, 0.1f);

    BoxShapeDesc desc(half_extents);
    return physics_engine->CreateBoxShape(desc);
}
