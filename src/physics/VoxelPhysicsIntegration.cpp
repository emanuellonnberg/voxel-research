#include "VoxelPhysicsIntegration.h"
#include <iostream>
#include <algorithm>

VoxelPhysicsIntegration::VoxelPhysicsIntegration(IPhysicsEngine* engine, VoxelWorld* world)
    : physics_engine(engine)
    , voxel_world(world)
    , voxel_density(1000.0f)  // Water density (1000 kg/m³)
    , friction(0.5f)
    , restitution(0.3f)
    , fragmentation_enabled(true)       // Week 5 Day 25: Enable by default
    , material_velocities_enabled(true) // Week 5 Day 25: Enable by default
    , settling_linear_threshold(0.1f)   // Week 5 Day 26: 0.1 m/s
    , settling_angular_threshold(0.2f)  // Week 5 Day 26: 0.2 rad/s
    , settling_time_threshold(0.5f)     // Week 5 Day 26: 0.5 seconds
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

        // Week 5 Day 25: Fragment cluster based on material (if enabled)
        std::vector<VoxelCluster> fragments;
        
        if (fragmentation_enabled && voxel_world) {
            // Get material ID from first voxel in cluster
            uint8_t material_id = MaterialDatabase::CONCRETE;  // Default
            if (!cluster.voxel_positions.empty()) {
                Voxel voxel = voxel_world->GetVoxel(cluster.voxel_positions[0]);
                if (voxel.is_active) {
                    material_id = voxel.material_id;
                }
            }
            
            // Fragment based on material
            fragments = fragmenter.FractureCluster(cluster, material_id);
        } else {
            // No fragmentation - use original cluster
            fragments = {cluster};
        }

        // Spawn each fragment as a separate debris body
        for (const auto& fragment : fragments) {
            if (fragment.voxel_positions.empty()) {
                continue;
            }

            // Calculate physics properties
            Vector3 center_of_mass = CalculateCenterOfMass(fragment);
            float mass = CalculateMass(fragment);

            // Create collision shape (convex hull from voxels)
            CollisionShapeHandle shape = CreateConvexHullFromVoxels(fragment);
            if (!shape) {
                // Fallback to bounding box if convex hull fails
                shape = CreateBoundingBoxFromVoxels(fragment);
            }

            if (!shape) {
                std::cerr << "[VoxelPhysicsIntegration] Failed to create shape for fragment\n";
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
                std::cerr << "[VoxelPhysicsIntegration] Failed to create body for fragment\n";
                physics_engine->DestroyShape(shape);
                continue;
            }

            // Attach shape to body
            physics_engine->AttachShape(body, shape);

            // Week 5 Day 25: Apply material-specific velocity
            if (material_velocities_enabled && voxel_world && !fragment.voxel_positions.empty()) {
                Voxel voxel = voxel_world->GetVoxel(fragment.voxel_positions[0]);
                if (voxel.is_active) {
                    ApplyMaterialVelocity(body, voxel.material_id);
                }
            }

            // Track debris
            debris_bodies.emplace_back(body, shape, cluster.cluster_id, fragment.voxel_positions.size());

            spawned_count++;
        }

        // Mark original cluster as spawned (even if fragmented)
        cluster_to_debris[cluster.cluster_id] = debris_bodies.size() - 1;
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

    // Week 5 Day 26: Update debris settling states
    UpdateDebrisStates(deltaTime);

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

// ===== Week 5 Day 25: Material-Specific Behaviors =====

void VoxelPhysicsIntegration::SetFragmentationEnabled(bool enable) {
    fragmentation_enabled = enable;
}

void VoxelPhysicsIntegration::SetMaterialVelocitiesEnabled(bool enable) {
    material_velocities_enabled = enable;
}

void VoxelPhysicsIntegration::ApplyMaterialVelocity(PhysicsBodyHandle body, uint8_t material_id) {
    if (!body || !material_velocities_enabled) {
        return;
    }

    // Material-specific initial velocities
    Vector3 linear_vel(0, 0, 0);
    Vector3 angular_vel(0, 0, 0);

    if (material_id == MaterialDatabase::WOOD) {
        // Wood: Tumbles as it falls
        linear_vel = Vector3(0, -1.0f, 0);  // Mostly downward
        
        // Medium spin
        float spin = 5.0f;
        angular_vel = Vector3(
            fragmenter.RandomRange(-spin, spin),
            fragmenter.RandomRange(-spin, spin),
            fragmenter.RandomRange(-spin, spin)
        );
    }
    else if (material_id == MaterialDatabase::CONCRETE) {
        // Concrete: Falls with slight randomness
        linear_vel = Vector3(
            fragmenter.RandomRange(-0.5f, 0.5f),
            -1.0f,
            fragmenter.RandomRange(-0.5f, 0.5f)
        );
        
        // Low spin
        float spin = 2.0f;
        angular_vel = Vector3(
            fragmenter.RandomRange(-spin, spin),
            fragmenter.RandomRange(-spin, spin),
            fragmenter.RandomRange(-spin, spin)
        );
    }
    else if (material_id == MaterialDatabase::BRICK) {
        // Brick: Similar to concrete but less random
        linear_vel = Vector3(
            fragmenter.RandomRange(-0.3f, 0.3f),
            -1.5f,
            fragmenter.RandomRange(-0.3f, 0.3f)
        );
        
        // Low spin
        float spin = 3.0f;
        angular_vel = Vector3(
            fragmenter.RandomRange(-spin, spin),
            fragmenter.RandomRange(-spin, spin),
            fragmenter.RandomRange(-spin, spin)
        );
    }
    else {
        // Default: Simple fall
        linear_vel = Vector3(0, -1.0f, 0);
        
        float spin = 2.0f;
        angular_vel = Vector3(
            fragmenter.RandomRange(-spin, spin),
            fragmenter.RandomRange(-spin, spin),
            fragmenter.RandomRange(-spin, spin)
        );
    }

    // Apply linear velocity to physics body
    physics_engine->SetBodyLinearVelocity(body, linear_vel);

    // Note: Angular velocity would need to be set during body creation
    // via RigidBodyDesc.angular_velocity. For now, we only set linear velocity.
    // TODO: Enhance to support angular velocity via recreation or new API
}

// ===== Week 5 Day 26: Settling Detection =====

int VoxelPhysicsIntegration::GetSettledDebrisCount() const {
    int count = 0;
    for (const auto& debris : debris_bodies) {
        if (debris.state == DebrisState::SETTLED) {
            count++;
        }
    }
    return count;
}

int VoxelPhysicsIntegration::GetActiveDebrisCount() const {
    int count = 0;
    for (const auto& debris : debris_bodies) {
        if (debris.state == DebrisState::ACTIVE || debris.state == DebrisState::SETTLING) {
            count++;
        }
    }
    return count;
}

void VoxelPhysicsIntegration::SetSettlingThresholds(float linear_threshold,
                                                     float angular_threshold,
                                                     float time_threshold) {
    settling_linear_threshold = linear_threshold;
    settling_angular_threshold = angular_threshold;
    settling_time_threshold = time_threshold;
}

void VoxelPhysicsIntegration::UpdateDebrisStates(float deltaTime) {
    if (!physics_engine) {
        return;
    }

    for (auto& debris : debris_bodies) {
        // Skip already settled debris
        if (debris.state == DebrisState::SETTLED) {
            continue;
        }

        // Get current linear velocity
        Vector3 linear_vel = physics_engine->GetBodyLinearVelocity(debris.body);
        float linear_speed = linear_vel.Length();

        // Note: We only check linear velocity since IPhysicsEngine doesn't expose
        // GetBodyAngularVelocity(). This is sufficient for basic settling detection.
        // Week 5 Day 26: settling_angular_threshold is reserved for future use.

        // Check if below linear threshold
        bool below_threshold = (linear_speed < settling_linear_threshold);

        if (below_threshold) {
            // Increment time below threshold
            debris.time_below_threshold += deltaTime;

            // Check if settled for long enough
            if (debris.time_below_threshold >= settling_time_threshold) {
                debris.state = DebrisState::SETTLED;
            } else {
                debris.state = DebrisState::SETTLING;
            }
        } else {
            // Above threshold - reset to active
            debris.state = DebrisState::ACTIVE;
            debris.time_below_threshold = 0.0f;
        }
    }
}

int VoxelPhysicsIntegration::ConvertSettledToStatic() {
    if (!physics_engine) {
        return 0;
    }

    // Note: IPhysicsEngine doesn't currently expose SetBodyKinematic() to change
    // a body's kinematic status after creation. This would require either:
    // 1. Adding SetBodyKinematic() to IPhysicsEngine interface
    // 2. Recreating bodies as kinematic (expensive)
    // 3. Just using the state tracking for queries (current approach)
    //
    // For now, we just track settled state without converting to kinematic.
    // The state information is still useful for:
    // - Optimizing updates (skip settled debris)
    // - Culling settled debris that are out of view
    // - Statistics and debugging
    //
    // Week 5 Day 26: This is acceptable for the prototype.

    int settled_count = GetSettledDebrisCount();

    if (settled_count > 0) {
        std::cout << "[VoxelPhysicsIntegration] " << settled_count
                  << " debris are settled (state tracked, not converted to kinematic)\n";
    }

    return settled_count;
}
