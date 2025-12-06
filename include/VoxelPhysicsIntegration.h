#pragma once

#include "IPhysicsEngine.h"
#include "VoxelCluster.h"
#include "VoxelWorld.h"
#include "VoxelFragmentation.h"
#include <vector>
#include <unordered_map>

// Forward declarations
class ParticleSystem;
class ImpactDetector;

/**
 * Collision groups for filtering (Week 5 Day 29, Week 13 Day 45+)
 */
enum CollisionGroup {
    COL_GROUND = 1 << 0,  // 0x0001 - Ground/static geometry
    COL_DEBRIS = 1 << 1,  // 0x0002 - Falling debris
    COL_UNITS  = 1 << 2,  // 0x0004 - Game units/characters
    COL_VOXELS = 1 << 3,  // 0x0008 - Voxel collision meshes (Week 13 Day 45+)
    COL_PROJECTILES = 1 << 4  // 0x0010 - Projectiles
};

/**
 * VoxelPhysicsIntegration
 *
 * Bridges structural analysis and physics simulation by converting
 * failed voxel clusters into rigid body debris with collision detection.
 *
 * Key Responsibilities:
 * - Convert VoxelCluster → RigidBody (with convex hull collision shape)
 * - Manage debris lifecycle (spawn, update, cleanup)
 * - Sync physics simulation with voxel world
 * - Track debris bodies for efficient updates
 *
 * Architecture:
 * - Engine-agnostic (works with MockEngine, BulletEngine, PhysXEngine)
 * - Owns collision shapes (creates and destroys)
 * - Uses cluster ID for tracking (prevents duplicate spawns)
 *
 * Week 5 Day 23: VoxelPhysicsIntegration
 */
class VoxelPhysicsIntegration {
public:
    /**
     * Constructor
     * @param engine Physics engine to use (must be initialized)
     * @param world Voxel world to sync with (optional, can be nullptr)
     */
    VoxelPhysicsIntegration(IPhysicsEngine* engine, VoxelWorld* world = nullptr);

    /**
     * Destructor - cleans up all debris bodies and shapes
     */
    ~VoxelPhysicsIntegration();

    // ===== Debris Management =====

    /**
     * Spawn debris from failed voxel clusters
     * @param clusters Failed clusters from StructuralAnalyzer
     * @return Number of new debris bodies spawned
     *
     * Each cluster becomes a rigid body with:
     * - Position: cluster center of mass
     * - Mass: based on voxel count (voxel_density kg/m³)
     * - Shape: convex hull from voxel positions
     *
     * Week 13 Day 43: Automatically uses parallel processing if JobSystem is available
     */
    int SpawnDebris(const std::vector<VoxelCluster>& clusters);

    /**
     * Spawn debris using serial implementation (for testing/fallback)
     * @param clusters Failed clusters from StructuralAnalyzer
     * @return Number of new debris bodies spawned
     *
     * Week 13 Day 43: Serial version for performance comparison
     */
    int SpawnDebrisSerial(const std::vector<VoxelCluster>& clusters);

    /**
     * Update physics simulation for all debris
     * @param deltaTime Time step in seconds
     *
     * Steps the physics engine and syncs debris positions
     */
    void Step(float deltaTime);

    /**
     * Clear all debris from physics world
     * Destroys all rigid bodies and collision shapes
     */
    void ClearDebris();

    /**
     * Remove debris that have fallen out of bounds
     * @param min_y Minimum Y coordinate (debris below this are removed)
     * @return Number of debris bodies removed
     */
    int RemoveOutOfBoundsDebris(float min_y = -100.0f);

    // ===== Configuration =====

    /**
     * Set voxel density (affects debris mass calculation)
     * @param density Density in kg/m³ (default: 1000.0 for water-like)
     */
    void SetVoxelDensity(float density);

    /**
     * Set physics material properties for debris
     * @param friction Surface friction (default: 0.5)
     * @param restitution Bounciness (default: 0.3)
     */
    void SetDebrisMaterial(float friction, float restitution);

    /**
     * Enable/disable material-specific fracture patterns
     * @param enable If true, clusters will fracture based on material type
     *
     * Week 5 Day 25: Material-specific fracture patterns
     */
    void SetFragmentationEnabled(bool enable);

    /**
     * Enable/disable material-specific initial velocities
     * @param enable If true, debris will have velocities based on material
     *
     * Week 5 Day 25: Material-specific behaviors
     */
    void SetMaterialVelocitiesEnabled(bool enable);

    // ===== Week 5 Day 29: Optimization =====

    /**
     * Enable optimized collision filtering
     * @param debris_collides_debris If false, debris won't collide with other debris (major optimization)
     *
     * Week 5 Day 29: Prevents debris-debris collisions for better performance
     */
    void SetCollisionFiltering(bool debris_collides_debris);

    /**
     * Remove debris older than specified age
     * @param max_age Maximum age in seconds
     * @return Number of debris removed
     *
     * Week 5 Day 29: Cleanup optimization
     */
    int RemoveDebrisOlderThan(float max_age);

    /**
     * Remove debris beyond specified distance from position
     * @param position Reference position (usually camera)
     * @param max_distance Maximum distance
     * @return Number of debris removed
     *
     * Week 5 Day 29: Distance-based cleanup
     */
    int RemoveDebrisBeyondDistance(const Vector3& position, float max_distance);
    void SetSettledCleanupDelay(float seconds);
    void SetSettledVisualLifetime(float seconds);
    int GetSettledVisualCount() const { return static_cast<int>(settled_visuals.size()); }

    // ===== Queries =====

    /**
     * Get number of active debris bodies
     */
    int GetDebrisCount() const;

    /**
     * Collect transforms/material IDs for rendering debris
     */
    void GetDebrisRenderData(std::vector<Transform>& out_transforms,
                             std::vector<uint8_t>& out_material_ids) const;

    /**
     * Check if a cluster has already been spawned
     * @param cluster_id Cluster ID to check
     */
    bool IsClusterSpawned(uint32_t cluster_id) const;

    /**
     * Get physics engine being used
     */
    IPhysicsEngine* GetPhysicsEngine() const { return physics_engine; }

    // ===== Week 5 Day 26: Settling Detection =====

    /**
     * Get number of settled debris bodies
     */
    int GetSettledDebrisCount() const;

    /**
     * Get number of active (non-settled) debris bodies
     */
    int GetActiveDebrisCount() const;

    /**
     * Set settling thresholds for debris
     * @param linear_threshold Linear velocity threshold (m/s)
     * @param angular_threshold Angular velocity threshold (rad/s)
     * @param time_threshold Time below thresholds before settling (seconds)
     */
    void SetSettlingThresholds(float linear_threshold,
                               float angular_threshold,
                               float time_threshold);

    /**
     * Convert settled debris to static (kinematic) bodies
     * Improves physics performance by making settled debris non-dynamic
     * @return Number of debris converted to static
     */
    int ConvertSettledToStatic();

    // ===== Impact Detection (Week 5 Day 28) =====

    /**
     * Enable impact detection with particle effects
     * @param particle_system Particle system for spawning effects (must remain valid)
     * @param impulse_threshold Minimum impulse to trigger effects (Newton-seconds)
     *
     * Requires Bullet Physics (USE_BULLET). When enabled, detects collisions
     * and spawns material-specific particles based on impact intensity.
     */
    void EnableImpactDetection(ParticleSystem* particle_system, float impulse_threshold = 10.0f);

    /**
     * Disable impact detection
     */
    void DisableImpactDetection();

    /**
     * Check if impact detection is enabled
     */
    bool IsImpactDetectionEnabled() const;

    /**
     * Get impact detector (for advanced configuration)
     * @return Pointer to impact detector, or nullptr if disabled
     */
    ImpactDetector* GetImpactDetector();

private:
    // ===== Internal Data =====

    IPhysicsEngine* physics_engine;
    VoxelWorld* voxel_world;

    /**
     * Debris state for settling detection
     */
    enum class DebrisState {
        ACTIVE,      // Moving (high velocity)
        SETTLING,    // Slowing down (below threshold but not yet settled)
        SETTLED      // At rest (converted to static or flagged for removal)
    };

    /**
     * Debris body tracking
     */
    struct DebrisBody {
        PhysicsBodyHandle body;          // Physics rigid body handle
        CollisionShapeHandle shape;      // Collision shape handle
        uint32_t cluster_id;             // Original cluster ID
        int voxel_count;                 // Number of voxels in cluster
        uint8_t material_id;             // Dominant material ID
        DebrisState state;               // Current state
        float time_below_threshold;      // Time spent below velocity thresholds
        float spawn_time;                // Time when debris was spawned (Week 5 Day 29)

        DebrisBody(PhysicsBodyHandle b, CollisionShapeHandle s, uint32_t id, int count,
                   float time, uint8_t mat_id)
            : body(b), shape(s), cluster_id(id), voxel_count(count), material_id(mat_id),
              state(DebrisState::ACTIVE), time_below_threshold(0.0f), spawn_time(time) {}
    };

    std::vector<DebrisBody> debris_bodies;
    std::unordered_map<uint32_t, size_t> cluster_to_debris;  // cluster_id → debris index

    // Configuration
    float voxel_density;   // kg/m³
    float friction;        // Surface friction
    float restitution;     // Bounciness

    // Week 5 Day 25: Material-specific behaviors
    bool fragmentation_enabled;         // Enable fracture patterns
    bool material_velocities_enabled;   // Enable material-specific velocities
    VoxelFragmentation fragmenter;      // Fracture pattern generator

    // Week 5 Day 26: Settling detection
    float settling_linear_threshold;    // Linear velocity threshold (m/s)
    float settling_angular_threshold;   // Angular velocity threshold (rad/s)
    float settling_time_threshold;      // Time below thresholds (seconds)

    // Week 5 Day 28: Impact detection
    ImpactDetector* impact_detector;    // Impact detector (nullptr when disabled)

    // Week 5 Day 29: Optimization
    bool debris_collides_debris;        // Whether debris collides with other debris
    float simulation_time;              // Total elapsed simulation time

    // ===== Helper Functions =====

    /**
     * Calculate center of mass for a voxel cluster
     * @param cluster Voxel cluster
     * @return Center of mass in world coordinates
     */
    Vector3 CalculateCenterOfMass(const VoxelCluster& cluster);

    /**
     * Calculate mass for a voxel cluster
     * @param cluster Voxel cluster
     * @return Mass in kg (based on voxel count and density)
     */
    float CalculateMass(const VoxelCluster& cluster);

    /**
     * Create convex hull collision shape from voxel positions
     * @param cluster Voxel cluster
     * @return Collision shape handle (must be destroyed later)
     *
     * Converts voxel grid positions into a simplified convex hull
     * for efficient collision detection
     */
    CollisionShapeHandle CreateConvexHullFromVoxels(const VoxelCluster& cluster);

    /**
     * Create a simple box collision shape for a cluster
     * @param cluster Voxel cluster
     * @return Collision shape handle
     *
     * Fallback if convex hull generation fails
     */
    CollisionShapeHandle CreateBoundingBoxFromVoxels(const VoxelCluster& cluster);

    /**
     * Apply material-specific initial velocity to debris
     * @param body Physics body handle
     * @param material_id Material type
     *
     * Week 5 Day 25: Material-specific velocities
     */
    void ApplyMaterialVelocity(PhysicsBodyHandle body, uint8_t material_id);

    /**
     * Update debris states based on velocity
     * @param deltaTime Time step in seconds
     *
     * Week 5 Day 26: Settling detection
     */
    void UpdateDebrisStates(float deltaTime);

    int CleanupSettledDebris();
    void RemoveExpiredSettledVisuals();
    void CreateGroundPlane();

    struct SettledDebrisVisual {
        Transform transform;
        uint8_t material_id;
        uint32_t cluster_id;
        float settled_time;
    };

    std::vector<SettledDebrisVisual> settled_visuals;
    float settled_cleanup_delay;
    float settled_visual_lifetime;
    PhysicsBodyHandle ground_body = nullptr;
    CollisionShapeHandle ground_shape = nullptr;
};
