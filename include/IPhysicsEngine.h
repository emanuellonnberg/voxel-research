#pragma once

#include "Vector3.h"
#include <cstdint>
#include <vector>

/**
 * Physics Engine Abstraction Layer (Week 5 Day 21)
 *
 * Generic interface for 3D physics engines (Bullet, PhysX, etc.)
 * Provides engine-agnostic rigid body simulation for voxel destruction.
 *
 * Design Philosophy:
 * - Simple: Only essential features for voxel debris simulation
 * - Generic: Works with any physics engine backend
 * - Efficient: Minimal overhead, direct mapping to engine APIs
 *
 * Supported Engines:
 * - Bullet Physics 3.x (primary, implemented)
 * - NVIDIA PhysX 5.x (stub, future implementation)
 * - Custom engines (easy to add)
 */

// Forward declarations for engine-specific types
// These are opaque handles that hide implementation details
using PhysicsBodyHandle = void*;
using CollisionShapeHandle = void*;

/**
 * Quaternion for 3D rotations
 * Simple struct, can be converted to/from engine-specific formats
 */
struct Quaternion {
    float x, y, z, w;

    Quaternion() : x(0), y(0), z(0), w(1) {}
    Quaternion(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}

    static Quaternion Identity() { return Quaternion(0, 0, 0, 1); }
};

/**
 * Transform (position + rotation)
 */
struct Transform {
    Vector3 position;
    Quaternion rotation;

    Transform() : position(Vector3::Zero()), rotation(Quaternion::Identity()) {}
    Transform(const Vector3& pos) : position(pos), rotation(Quaternion::Identity()) {}
    Transform(const Vector3& pos, const Quaternion& rot) : position(pos), rotation(rot) {}
};

/**
 * Rigid body descriptor
 * Describes properties for creating a rigid body
 */
struct RigidBodyDesc {
    Transform transform;           // Initial position and rotation
    float mass;                    // Mass in kg (0 = static/infinite mass)
    Vector3 linear_velocity;       // Initial linear velocity (m/s)
    Vector3 angular_velocity;      // Initial angular velocity (rad/s)
    float linear_damping;          // Linear damping (0-1, typically 0.1)
    float angular_damping;         // Angular damping (0-1, typically 0.1)
    float friction;                // Surface friction (0-1, typically 0.5)
    float restitution;             // Bounciness (0-1, typically 0.3)
    bool kinematic;                // Kinematic (controlled by code, not physics)

    RigidBodyDesc()
        : mass(1.0f)
        , linear_velocity(Vector3::Zero())
        , angular_velocity(Vector3::Zero())
        , linear_damping(0.1f)
        , angular_damping(0.1f)
        , friction(0.5f)
        , restitution(0.3f)
        , kinematic(false)
    {}
};

/**
 * Collision shape types
 */
enum class ShapeType {
    BOX,           // Axis-aligned box (fast, simple)
    SPHERE,        // Sphere (fastest)
    CONVEX_HULL,   // Convex hull from points (accurate, moderate cost)
    COMPOUND       // Multiple shapes combined (for complex debris)
};

/**
 * Box collision shape descriptor
 */
struct BoxShapeDesc {
    Vector3 half_extents;  // Half-widths along each axis

    BoxShapeDesc() : half_extents(0.5f, 0.5f, 0.5f) {}
    BoxShapeDesc(const Vector3& extents) : half_extents(extents) {}
};

/**
 * Sphere collision shape descriptor
 */
struct SphereShapeDesc {
    float radius;

    SphereShapeDesc() : radius(0.5f) {}
    SphereShapeDesc(float r) : radius(r) {}
};

/**
 * Convex hull collision shape descriptor
 */
struct ConvexHullShapeDesc {
    std::vector<Vector3> vertices;  // Points that define the convex hull

    ConvexHullShapeDesc() {}
    ConvexHullShapeDesc(const std::vector<Vector3>& verts) : vertices(verts) {}
};

/**
 * Ray for raycasting
 */
struct Ray {
    Vector3 origin;
    Vector3 direction;  // Should be normalized
    float max_distance;

    Ray() : origin(Vector3::Zero()), direction(Vector3(0, 0, 1)), max_distance(1000.0f) {}
    Ray(const Vector3& orig, const Vector3& dir, float max_dist = 1000.0f)
        : origin(orig), direction(dir), max_distance(max_dist) {}
};

/**
 * Raycast hit result
 */
struct RaycastHit {
    bool hit;                      // True if ray hit something
    Vector3 point;                 // Hit point in world space
    Vector3 normal;                // Surface normal at hit point
    float distance;                // Distance from ray origin to hit point
    PhysicsBodyHandle body;        // Body that was hit (nullptr if none)

    RaycastHit() : hit(false), point(Vector3::Zero()), normal(Vector3::Zero()),
                   distance(0.0f), body(nullptr) {}
};

/**
 * Abstract physics engine interface
 *
 * All physics engines (Bullet, PhysX, custom) must implement this interface.
 *
 * Usage:
 *   IPhysicsEngine* physics = new BulletEngine();
 *   auto body = physics->CreateRigidBody(desc);
 *   auto shape = physics->CreateBoxShape(BoxShapeDesc(Vector3(0.5f, 0.5f, 0.5f)));
 *   physics->AttachShape(body, shape);
 *   physics->Step(1.0f / 60.0f);
 *   Transform transform = physics->GetBodyTransform(body);
 */
class IPhysicsEngine {
public:
    virtual ~IPhysicsEngine() = default;

    // ===== Initialization =====

    /**
     * Initialize the physics engine
     * Must be called before any other operations
     */
    virtual bool Initialize() = 0;

    /**
     * Shutdown and cleanup
     */
    virtual void Shutdown() = 0;

    // ===== Simulation =====

    /**
     * Step the simulation forward by deltaTime seconds
     * Typically called once per frame (e.g., 1/60.0f for 60 FPS)
     */
    virtual void Step(float deltaTime) = 0;

    /**
     * Set gravity vector (default: 0, -9.81, 0)
     */
    virtual void SetGravity(const Vector3& gravity) = 0;

    // ===== Rigid Bodies =====

    /**
     * Create a rigid body
     * Returns handle to the body (nullptr on failure)
     */
    virtual PhysicsBodyHandle CreateRigidBody(const RigidBodyDesc& desc) = 0;

    /**
     * Destroy a rigid body and free resources
     */
    virtual void DestroyRigidBody(PhysicsBodyHandle body) = 0;

    /**
     * Get body transform (position and rotation)
     */
    virtual Transform GetBodyTransform(PhysicsBodyHandle body) = 0;

    /**
     * Set body transform (for kinematic bodies or teleporting)
     */
    virtual void SetBodyTransform(PhysicsBodyHandle body, const Transform& transform) = 0;

    /**
     * Get body linear velocity
     */
    virtual Vector3 GetBodyLinearVelocity(PhysicsBodyHandle body) = 0;

    /**
     * Set body linear velocity
     */
    virtual void SetBodyLinearVelocity(PhysicsBodyHandle body, const Vector3& velocity) = 0;

    /**
     * Apply force to body at its center of mass
     */
    virtual void ApplyForce(PhysicsBodyHandle body, const Vector3& force) = 0;

    /**
     * Apply impulse to body at its center of mass (instant velocity change)
     */
    virtual void ApplyImpulse(PhysicsBodyHandle body, const Vector3& impulse) = 0;

    // ===== Collision Shapes =====

    /**
     * Create box collision shape
     */
    virtual CollisionShapeHandle CreateBoxShape(const BoxShapeDesc& desc) = 0;

    /**
     * Create sphere collision shape
     */
    virtual CollisionShapeHandle CreateSphereShape(const SphereShapeDesc& desc) = 0;

    /**
     * Create convex hull collision shape from points
     */
    virtual CollisionShapeHandle CreateConvexHullShape(const ConvexHullShapeDesc& desc) = 0;

    /**
     * Destroy collision shape and free resources
     */
    virtual void DestroyShape(CollisionShapeHandle shape) = 0;

    /**
     * Attach collision shape to rigid body
     * Body takes ownership, shape can be shared by multiple bodies
     */
    virtual void AttachShape(PhysicsBodyHandle body, CollisionShapeHandle shape) = 0;

    // ===== Queries =====

    /**
     * Perform raycast against all bodies
     * Returns first hit along ray
     */
    virtual RaycastHit Raycast(const Ray& ray) = 0;

    /**
     * Check if a point is inside any collision shape
     */
    virtual bool OverlapPoint(const Vector3& point) = 0;

    // ===== Debug & Stats =====

    /**
     * Get number of active rigid bodies
     */
    virtual int GetBodyCount() const = 0;

    /**
     * Get engine name (e.g., "Bullet", "PhysX")
     */
    virtual const char* GetEngineName() const = 0;

    /**
     * Get engine version string
     */
    virtual const char* GetEngineVersion() const = 0;
};
