#pragma once

#include "IPhysicsEngine.h"
#include "VoxelCluster.h"
#include "VoxelWorld.h"
#include "VoxelFragmentation.h"
#include <vector>
#include <unordered_map>

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
     */
    int SpawnDebris(const std::vector<VoxelCluster>& clusters);

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

    // ===== Queries =====

    /**
     * Get number of active debris bodies
     */
    int GetDebrisCount() const;

    /**
     * Check if a cluster has already been spawned
     * @param cluster_id Cluster ID to check
     */
    bool IsClusterSpawned(uint32_t cluster_id) const;

    /**
     * Get physics engine being used
     */
    IPhysicsEngine* GetPhysicsEngine() const { return physics_engine; }

private:
    // ===== Internal Data =====

    IPhysicsEngine* physics_engine;
    VoxelWorld* voxel_world;

    /**
     * Debris body tracking
     */
    struct DebrisBody {
        PhysicsBodyHandle body;          // Physics rigid body handle
        CollisionShapeHandle shape;      // Collision shape handle
        uint32_t cluster_id;             // Original cluster ID
        int voxel_count;                 // Number of voxels in cluster

        DebrisBody(PhysicsBodyHandle b, CollisionShapeHandle s, uint32_t id, int count)
            : body(b), shape(s), cluster_id(id), voxel_count(count) {}
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
};
