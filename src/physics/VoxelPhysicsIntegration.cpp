#include "VoxelPhysicsIntegration.h"
#include "JobSystem.h"  // Week 13 Day 43: Parallel debris spawning
#include "MockPhysicsEngine.h"  // For dynamic_cast checks
#include <iostream>
#include <algorithm>
#include <mutex>  // Week 13 Day 43: Thread safety for debris spawning

// Week 5 Day 28: Bullet Physics includes for impact detection
#ifdef USE_BULLET
#include "BulletEngine.h"
#include "ImpactDetector.h"
#include <btBulletDynamicsCommon.h>
#endif

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
    , impact_detector(nullptr)          // Week 5 Day 28: Disabled by default
    , debris_collides_debris(true)      // Week 5 Day 29: Enabled by default
    , simulation_time(0.0f)             // Week 5 Day 29: Start at 0
    , settled_cleanup_delay(1.0f)
    , settled_visual_lifetime(0.0f)
    , ground_body(nullptr)
    , ground_shape(nullptr)
{
    if (!physics_engine) {
        throw std::runtime_error("VoxelPhysicsIntegration: physics_engine cannot be null");
    }

    std::cout << "[VoxelPhysicsIntegration] Initialized with "
              << physics_engine->GetEngineName() << "\n";

    CreateGroundPlane();
}

VoxelPhysicsIntegration::~VoxelPhysicsIntegration() {
    DisableImpactDetection(); // Week 5 Day 28: Cleanup impact detector
    ClearDebris();
    if (physics_engine) {
        if (ground_body) {
            physics_engine->DestroyRigidBody(ground_body);
            ground_body = nullptr;
        }
        if (ground_shape) {
            physics_engine->DestroyShape(ground_shape);
            ground_shape = nullptr;
        }
    }
    std::cout << "[VoxelPhysicsIntegration] Shutdown complete\n";
}

// ===== Debris Management =====

int VoxelPhysicsIntegration::SpawnDebris(const std::vector<VoxelCluster>& clusters) {
    int spawned_count = 0;

    // Week 13 Day 43: Determine if we should use parallel processing
    // Note: MockPhysicsEngine operations are too fast to benefit from parallelization
    // due to JobSystem overhead. Only parallelize for real physics engines with
    // substantial workloads (empirically determined: need 200+ clusters for break-even).
    const bool is_mock_engine = dynamic_cast<MockPhysicsEngine*>(physics_engine) != nullptr;
    const bool use_parallel = g_JobSystem &&
                              g_JobSystem->IsRunning() &&
                              !is_mock_engine &&  // Don't parallelize MockPhysicsEngine
                              clusters.size() > 200;  // Only parallelize for large workloads

    if (!use_parallel) {
        // Serial path: Original implementation
        return SpawnDebrisSerial(clusters);
    }

    // Week 13 Day 43: Parallel path
    // Strategy: Prepare all debris data in parallel, then create bodies sequentially

    // Step 1: Collect clusters that need spawning
    std::vector<const VoxelCluster*> clusters_to_spawn;
    for (const auto& cluster : clusters) {
        if (!IsClusterSpawned(cluster.cluster_id) && !cluster.voxel_positions.empty()) {
            clusters_to_spawn.push_back(&cluster);
        }
    }

    if (clusters_to_spawn.empty()) {
        return 0;
    }

    // Step 2: Prepare debris data in parallel
    struct PreparedDebris {
        VoxelCluster fragment;
        Vector3 center_of_mass;
        float mass;
        CollisionShapeHandle shape;
        uint8_t material_id;
        uint32_t original_cluster_id;
        bool valid;

        PreparedDebris() : shape(nullptr), material_id(0), original_cluster_id(0), valid(false) {}
    };

    std::vector<std::vector<PreparedDebris>> prepared_per_cluster(clusters_to_spawn.size());
    std::mutex error_mutex;  // For error reporting

    // Parallelize fragmentation and shape creation
    g_JobSystem->ParallelFor(static_cast<int>(clusters_to_spawn.size()), [&](int idx) {
        const VoxelCluster& cluster = *clusters_to_spawn[idx];
        std::vector<PreparedDebris>& prepared = prepared_per_cluster[idx];

        // Fragment cluster based on material (if enabled)
        std::vector<VoxelCluster> fragments;
        if (fragmentation_enabled && voxel_world) {
            uint8_t material_id = MaterialDatabase::CONCRETE;
            if (!cluster.voxel_positions.empty()) {
                Voxel voxel = voxel_world->GetVoxel(cluster.voxel_positions[0]);
                if (voxel.is_active) {
                    material_id = voxel.material_id;
                }
            }
            fragments = fragmenter.FractureCluster(cluster, material_id);
        } else {
            fragments = {cluster};
        }

        // Prepare each fragment
        for (const auto& fragment : fragments) {
            if (fragment.voxel_positions.empty()) continue;

            PreparedDebris prep;
            prep.fragment = fragment;
            prep.original_cluster_id = cluster.cluster_id;

            // Calculate physics properties (all thread-safe, read-only operations)
            prep.center_of_mass = CalculateCenterOfMass(fragment);
            prep.mass = CalculateMass(fragment);

            // Get material ID
            if (voxel_world && !fragment.voxel_positions.empty()) {
                Voxel voxel = voxel_world->GetVoxel(fragment.voxel_positions[0]);
                if (voxel.is_active) {
                    prep.material_id = voxel.material_id;
                }
            }

            // Create collision shape (thread-safe, independent operation)
            prep.shape = CreateConvexHullFromVoxels(fragment);
            if (!prep.shape) {
                prep.shape = CreateBoundingBoxFromVoxels(fragment);
            }

            if (prep.shape) {
                prep.valid = true;
                prepared.push_back(prep);
            } else {
                std::lock_guard<std::mutex> lock(error_mutex);
                std::cerr << "[VoxelPhysicsIntegration] Failed to create shape for fragment\n";
            }
        }
    });

    // Step 3: Create rigid bodies sequentially (modifies physics world state)
    for (size_t i = 0; i < prepared_per_cluster.size(); ++i) {
        const auto& prepared_list = prepared_per_cluster[i];

        for (const auto& prep : prepared_list) {
            if (!prep.valid) continue;

            // Create rigid body
            RigidBodyDesc desc;
            desc.transform.position = prep.center_of_mass;
            desc.transform.rotation = Quaternion::Identity();
            desc.mass = prep.mass;
            desc.linear_damping = 0.1f;
            desc.angular_damping = 0.1f;
            desc.friction = friction;
            desc.restitution = restitution;
            desc.kinematic = false;

            PhysicsBodyHandle body = physics_engine->CreateRigidBody(desc);
            if (!body) {
                std::cerr << "[VoxelPhysicsIntegration] Failed to create body\n";
                physics_engine->DestroyShape(prep.shape);
                continue;
            }

            // Attach shape to body
            physics_engine->AttachShape(body, prep.shape);

            // Apply material-specific velocity
            uint8_t fragment_material = prep.material_id == 0 ? MaterialDatabase::CONCRETE
                                                              : prep.material_id;
            if (material_velocities_enabled) {
                ApplyMaterialVelocity(body, fragment_material);
            }

            // Apply collision filtering (Bullet only)
#ifdef USE_BULLET
            if (dynamic_cast<BulletEngine*>(physics_engine)) {
                btRigidBody* bt_body = static_cast<btRigidBody*>(body);
                if (bt_body && bt_body->getBroadphaseHandle()) {
                    short debris_mask = debris_collides_debris ?
                        (COL_GROUND | COL_DEBRIS | COL_UNITS) :
                        (COL_GROUND | COL_UNITS);
                    bt_body->getBroadphaseHandle()->m_collisionFilterGroup = COL_DEBRIS;
                    bt_body->getBroadphaseHandle()->m_collisionFilterMask = debris_mask;
                }
            }
#endif

            // Track debris
            debris_bodies.emplace_back(body,
                                       prep.shape,
                                       prep.original_cluster_id,
                                       static_cast<int>(prep.fragment.voxel_positions.size()),
                                       simulation_time,
                                       fragment_material);
            spawned_count++;
        }

        // Mark original cluster as spawned
        if (!prepared_list.empty() && prepared_list[0].valid) {
            cluster_to_debris[prepared_list[0].original_cluster_id] = debris_bodies.size() - 1;
        }
    }

    if (spawned_count > 0) {
        std::cout << "[VoxelPhysicsIntegration] Spawned " << spawned_count
                  << " debris bodies (total: " << debris_bodies.size() << ") [PARALLEL]\n";
    }

    return spawned_count;
}

// Week 13 Day 43: Serial fallback implementation
int VoxelPhysicsIntegration::SpawnDebrisSerial(const std::vector<VoxelCluster>& clusters) {
    int spawned_count = 0;

    for (const auto& cluster : clusters) {
        if (IsClusterSpawned(cluster.cluster_id)) continue;
        if (cluster.voxel_positions.empty()) continue;

        // Fragment cluster based on material (if enabled)
        std::vector<VoxelCluster> fragments;
        if (fragmentation_enabled && voxel_world) {
            uint8_t material_id = MaterialDatabase::CONCRETE;
            if (!cluster.voxel_positions.empty()) {
                Voxel voxel = voxel_world->GetVoxel(cluster.voxel_positions[0]);
                if (voxel.is_active) {
                    material_id = voxel.material_id;
                }
            }
            fragments = fragmenter.FractureCluster(cluster, material_id);
        } else {
            fragments = {cluster};
        }

        // Spawn each fragment
        for (const auto& fragment : fragments) {
            if (fragment.voxel_positions.empty()) continue;

            Vector3 center_of_mass = CalculateCenterOfMass(fragment);
            float mass = CalculateMass(fragment);
            uint8_t fragment_material = fragment.dominant_material;
            if (fragment_material == 0 && voxel_world && !fragment.voxel_positions.empty()) {
                Voxel voxel = voxel_world->GetVoxel(fragment.voxel_positions[0]);
                if (voxel.is_active) {
                    fragment_material = voxel.material_id;
                }
            }
            if (fragment_material == 0) {
                fragment_material = MaterialDatabase::CONCRETE;
            }

            CollisionShapeHandle shape = CreateConvexHullFromVoxels(fragment);
            if (!shape) {
                shape = CreateBoundingBoxFromVoxels(fragment);
            }

            if (!shape) {
                std::cerr << "[VoxelPhysicsIntegration] Failed to create shape\n";
                continue;
            }

            RigidBodyDesc desc;
            desc.transform.position = center_of_mass;
            desc.transform.rotation = Quaternion::Identity();
            desc.mass = mass;
            desc.linear_damping = 0.1f;
            desc.angular_damping = 0.1f;
            desc.friction = friction;
            desc.restitution = restitution;
            desc.kinematic = false;

            PhysicsBodyHandle body = physics_engine->CreateRigidBody(desc);
            if (!body) {
                std::cerr << "[VoxelPhysicsIntegration] Failed to create body\n";
                physics_engine->DestroyShape(shape);
                continue;
            }

            physics_engine->AttachShape(body, shape);

            if (material_velocities_enabled) {
                ApplyMaterialVelocity(body, fragment_material);
            }

#ifdef USE_BULLET
            if (dynamic_cast<BulletEngine*>(physics_engine)) {
                btRigidBody* bt_body = static_cast<btRigidBody*>(body);
                if (bt_body && bt_body->getBroadphaseHandle()) {
                    short debris_mask = debris_collides_debris ?
                        (COL_GROUND | COL_DEBRIS | COL_UNITS) :
                        (COL_GROUND | COL_UNITS);
                    bt_body->getBroadphaseHandle()->m_collisionFilterGroup = COL_DEBRIS;
                    bt_body->getBroadphaseHandle()->m_collisionFilterMask = debris_mask;
                }
            }
#endif

            debris_bodies.emplace_back(body,
                                       shape,
                                       cluster.cluster_id,
                                       static_cast<int>(fragment.voxel_positions.size()),
                                       simulation_time,
                                       fragment_material);
            spawned_count++;
        }

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

    // Week 5 Day 29: Track total simulation time
    simulation_time += deltaTime;

    // Step physics simulation
    physics_engine->Step(deltaTime);

    // Week 5 Day 26: Update debris settling states
    UpdateDebrisStates(deltaTime);
    int converted = CleanupSettledDebris();
    if (converted > 0) {
        std::cout << "[VoxelPhysicsIntegration] Converted " << converted
                  << " debris to static visuals (total static: " << settled_visuals.size() << ")\n";
    }
    RemoveExpiredSettledVisuals();

#ifdef USE_BULLET
    // Week 5 Day 28: Detect impacts and spawn particles
    if (impact_detector) {
        impact_detector->DetectAndSpawnParticles();
    }
#endif

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
    settled_visuals.clear();

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

void VoxelPhysicsIntegration::SetSettledCleanupDelay(float seconds) {
    settled_cleanup_delay = std::max(0.0f, seconds);
}

void VoxelPhysicsIntegration::SetSettledVisualLifetime(float seconds) {
    settled_visual_lifetime = seconds;
}

// ===== Queries =====

int VoxelPhysicsIntegration::GetDebrisCount() const {
    return static_cast<int>(debris_bodies.size());
}

void VoxelPhysicsIntegration::GetDebrisRenderData(std::vector<Transform>& out_transforms,
                                                  std::vector<uint8_t>& out_material_ids) const {
    out_transforms.clear();
    out_material_ids.clear();

    if (!physics_engine) {
        return;
    }

    out_transforms.reserve(debris_bodies.size() + settled_visuals.size());
    out_material_ids.reserve(debris_bodies.size() + settled_visuals.size());

    for (const auto& debris : debris_bodies) {
        if (!debris.body) {
            continue;
        }
        out_transforms.push_back(physics_engine->GetBodyTransform(debris.body));
        out_material_ids.push_back(debris.material_id);
    }

    for (const auto& settled : settled_visuals) {
        out_transforms.push_back(settled.transform);
        out_material_ids.push_back(settled.material_id);
    }
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

int VoxelPhysicsIntegration::CleanupSettledDebris() {
    if (!physics_engine) {
        return 0;
    }

    int converted = 0;
    for (size_t i = 0; i < debris_bodies.size();) {
        auto& debris = debris_bodies[i];
        bool ready = debris.state == DebrisState::SETTLED &&
                     (simulation_time - debris.spawn_time) >= settled_cleanup_delay;
        if (!ready) {
            ++i;
            continue;
        }

        SettledDebrisVisual visual;
        visual.transform = physics_engine->GetBodyTransform(debris.body);
        visual.material_id = debris.material_id;
        visual.cluster_id = debris.cluster_id;
        visual.settled_time = simulation_time;
        settled_visuals.push_back(visual);

        physics_engine->DestroyRigidBody(debris.body);
        physics_engine->DestroyShape(debris.shape);
        cluster_to_debris.erase(debris.cluster_id);

        if (i < debris_bodies.size() - 1) {
            debris_bodies[i] = debris_bodies.back();
            cluster_to_debris[debris_bodies[i].cluster_id] = i;
        }
        debris_bodies.pop_back();
        ++converted;
    }
    return converted;
}

void VoxelPhysicsIntegration::RemoveExpiredSettledVisuals() {
    if (settled_visual_lifetime <= 0.0f) {
        return;
    }

    size_t before = settled_visuals.size();
    settled_visuals.erase(
        std::remove_if(settled_visuals.begin(), settled_visuals.end(),
                       [&](const SettledDebrisVisual& visual) {
                           return (simulation_time - visual.settled_time) >= settled_visual_lifetime;
                       }),
        settled_visuals.end());

    size_t removed = before - settled_visuals.size();
    if (removed > 0) {
        std::cout << "[VoxelPhysicsIntegration] Removed " << removed
                  << " expired static debris visuals\n";
    }
}

void VoxelPhysicsIntegration::CreateGroundPlane() {
    if (!physics_engine || ground_body) {
        return;
    }

    BoxShapeDesc ground_shape_desc;
    ground_shape_desc.half_extents = Vector3(500.0f, 0.5f, 500.0f);
    ground_shape = physics_engine->CreateBoxShape(ground_shape_desc);
    if (!ground_shape) {
        std::cerr << "[VoxelPhysicsIntegration] Failed to create ground shape\n";
        return;
    }

    RigidBodyDesc ground_desc;
    ground_desc.transform.position = Vector3(0.0f, -0.5f, 0.0f);
    ground_desc.mass = 0.0f;
    ground_desc.linear_damping = 0.0f;
    ground_desc.angular_damping = 0.0f;
    ground_desc.friction = friction;
    ground_desc.restitution = restitution * 0.5f;
    ground_desc.kinematic = true;

    ground_body = physics_engine->CreateRigidBody(ground_desc);
    if (!ground_body) {
        std::cerr << "[VoxelPhysicsIntegration] Failed to create ground body\n";
        physics_engine->DestroyShape(ground_shape);
        ground_shape = nullptr;
        return;
    }

    physics_engine->AttachShape(ground_body, ground_shape);
    std::cout << "[VoxelPhysicsIntegration] Ground plane created (size 1000x1000)\n";
}

// ===== Impact Detection (Week 5 Day 28) =====

void VoxelPhysicsIntegration::EnableImpactDetection(ParticleSystem* particle_system, float impulse_threshold) {
#ifdef USE_BULLET
    // Disable existing detector if any
    DisableImpactDetection();

    if (!particle_system) {
        std::cerr << "[VoxelPhysicsIntegration] Cannot enable impact detection: particle_system is null\n";
        return;
    }

    // Get Bullet dynamics world from physics engine
    BulletEngine* bullet_engine = dynamic_cast<BulletEngine*>(physics_engine);
    if (!bullet_engine) {
        std::cerr << "[VoxelPhysicsIntegration] Impact detection requires BulletEngine\n";
        return;
    }

    btDynamicsWorld* dynamics_world = bullet_engine->GetDynamicsWorld();
    if (!dynamics_world) {
        std::cerr << "[VoxelPhysicsIntegration] BulletEngine dynamics world is null\n";
        return;
    }

    // Create impact detector
    impact_detector = new ImpactDetector(particle_system, dynamics_world);
    impact_detector->SetImpulseThreshold(impulse_threshold);

    // Register materials for existing debris
    for (const auto& debris : debris_bodies) {
        // Get material from voxel world
        uint8_t material_id = MaterialDatabase::CONCRETE; // Default
        if (voxel_world && !debris.voxel_count == 0) {
            // Try to get material from first voxel (simplified)
            material_id = MaterialDatabase::CONCRETE;
        }

        // Register with impact detector
        btRigidBody* body = static_cast<btRigidBody*>(debris.body);
        impact_detector->RegisterBodyMaterial(body, material_id);
    }

    std::cout << "[VoxelPhysicsIntegration] Impact detection enabled (threshold: "
              << impulse_threshold << " N·s)\n";
#else
    (void)particle_system;
    (void)impulse_threshold;
    std::cerr << "[VoxelPhysicsIntegration] Impact detection requires Bullet Physics (USE_BULLET)\n";
#endif
}

void VoxelPhysicsIntegration::DisableImpactDetection() {
#ifdef USE_BULLET
    if (impact_detector) {
        delete impact_detector;
        impact_detector = nullptr;
        std::cout << "[VoxelPhysicsIntegration] Impact detection disabled\n";
    }
#endif
}

bool VoxelPhysicsIntegration::IsImpactDetectionEnabled() const {
#ifdef USE_BULLET
    return impact_detector != nullptr;
#else
    return false;
#endif
}

ImpactDetector* VoxelPhysicsIntegration::GetImpactDetector() {
#ifdef USE_BULLET
    return impact_detector;
#else
    return nullptr;
#endif
}

// ===== Week 5 Day 29: Optimization =====

void VoxelPhysicsIntegration::SetCollisionFiltering(bool debris_collides_debris_flag) {
    debris_collides_debris = debris_collides_debris_flag;

#ifdef USE_BULLET
    // Only apply if using BulletEngine
    BulletEngine* bullet_engine = dynamic_cast<BulletEngine*>(physics_engine);
    if (!bullet_engine) {
        return; // Not using Bullet, skip
    }

    // Determine collision mask for debris
    short debris_mask;
    if (debris_collides_debris_flag) {
        // Debris collides with everything
        debris_mask = COL_GROUND | COL_DEBRIS | COL_UNITS;
    } else {
        // Debris collides with ground and units, NOT other debris (optimization)
        debris_mask = COL_GROUND | COL_UNITS;
    }

    // Apply collision filtering to all existing debris
    for (const auto& debris : debris_bodies) {
        btRigidBody* body = static_cast<btRigidBody*>(debris.body);
        if (body && body->getBroadphaseHandle()) {
            body->getBroadphaseHandle()->m_collisionFilterGroup = COL_DEBRIS;
            body->getBroadphaseHandle()->m_collisionFilterMask = debris_mask;
        }
    }

    std::cout << "[VoxelPhysicsIntegration] Collision filtering: debris-debris = "
              << (debris_collides_debris_flag ? "ENABLED" : "DISABLED") << "\n";
#else
    (void)debris_collides_debris_flag; // Unused without Bullet
#endif
}

int VoxelPhysicsIntegration::RemoveDebrisOlderThan(float max_age) {
    if (!physics_engine) {
        return 0;
    }

    int removed_count = 0;
    auto it = debris_bodies.begin();

    while (it != debris_bodies.end()) {
        float age = simulation_time - it->spawn_time;

        if (age > max_age) {
            // Unregister from impact detector if active
#ifdef USE_BULLET
            if (impact_detector) {
                btRigidBody* body = static_cast<btRigidBody*>(it->body);
                impact_detector->UnregisterBody(body);
            }
#endif

            // Destroy physics objects
            physics_engine->DestroyRigidBody(it->body);
            physics_engine->DestroyShape(it->shape);

            // Remove from cluster tracking
            cluster_to_debris.erase(it->cluster_id);

            // Remove from list
            it = debris_bodies.erase(it);
            removed_count++;
        } else {
            ++it;
        }
    }

    if (removed_count > 0) {
        std::cout << "[VoxelPhysicsIntegration] Removed " << removed_count
                  << " debris older than " << max_age << "s\n";
    }

    return removed_count;
}

int VoxelPhysicsIntegration::RemoveDebrisBeyondDistance(const Vector3& position, float max_distance) {
    if (!physics_engine) {
        return 0;
    }

    int removed_count = 0;
    float max_distance_sq = max_distance * max_distance; // Avoid sqrt
    auto it = debris_bodies.begin();

    while (it != debris_bodies.end()) {
        // Get debris position (use Bullet directly when available)
        Vector3 debris_pos;
        bool used_bullet_path = false;

#ifdef USE_BULLET
        if (dynamic_cast<BulletEngine*>(physics_engine)) {
            used_bullet_path = true;
            btRigidBody* bt_body = static_cast<btRigidBody*>(it->body);
            if (bt_body) {
                btVector3 bt_pos = bt_body->getWorldTransform().getOrigin();
                debris_pos = Vector3(bt_pos.x(), bt_pos.y(), bt_pos.z());
            } else {
                ++it;
                continue;
            }
        }
#endif
        if (!used_bullet_path) {
            // For non-Bullet engines (or Mock), use velocity as proxy (simplified)
            Vector3 vel = physics_engine->GetBodyLinearVelocity(it->body);
            if (vel.Length() < 0.01f) {
                // Assume settled debris is at spawn position (simplified)
                ++it;
                continue;
            }
            debris_pos = Vector3::Zero(); // Fallback
        }

        Vector3 delta = debris_pos - position;
        float distance_sq = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;

        if (distance_sq > max_distance_sq) {
            // Unregister from impact detector if active
#ifdef USE_BULLET
            if (impact_detector) {
                btRigidBody* body = static_cast<btRigidBody*>(it->body);
                impact_detector->UnregisterBody(body);
            }
#endif

            // Destroy physics objects
            physics_engine->DestroyRigidBody(it->body);
            physics_engine->DestroyShape(it->shape);

            // Remove from cluster tracking
            cluster_to_debris.erase(it->cluster_id);

            // Remove from list
            it = debris_bodies.erase(it);
            removed_count++;
        } else {
            ++it;
        }
    }

    if (removed_count > 0) {
        std::cout << "[VoxelPhysicsIntegration] Removed " << removed_count
                  << " debris beyond " << max_distance << "m\n";
    }

    return removed_count;
}
