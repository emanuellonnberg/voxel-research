#pragma once

#include "IPhysicsEngine.h"

// Forward declare Bullet types to avoid including headers here
class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
class btBroadphaseInterface;
class btSequentialImpulseConstraintSolver;
class btDiscreteDynamicsWorld;
class btRigidBody;
class btCollisionShape;

/**
 * Bullet Physics Engine (Week 5 Day 22)
 *
 * Full implementation using Bullet Physics 3.x
 *
 * Features:
 * - Real collision detection and response
 * - Rigid body dynamics with rotation
 * - Contact solving (stacking, sliding, bouncing)
 * - Works on ALL platforms (Linux, Windows, Mac, Web, Mobile)
 * - Open source (zlib license)
 * - Production-ready
 *
 * Usage:
 *   BulletEngine engine;
 *   engine.Initialize();
 *   engine.SetGravity(Vector3(0, -9.81f, 0));
 *
 *   auto body = engine.CreateRigidBody(desc);
 *   auto shape = engine.CreateBoxShape(BoxShapeDesc(Vector3(0.5f, 0.5f, 0.5f)));
 *   engine.AttachShape(body, shape);
 *
 *   engine.Step(1.0f / 60.0f);  // Real physics!
 */

class BulletEngine : public IPhysicsEngine {
private:
    // Bullet world objects
    btDefaultCollisionConfiguration* collision_config;
    btCollisionDispatcher* dispatcher;
    btBroadphaseInterface* broadphase;
    btSequentialImpulseConstraintSolver* solver;
    btDiscreteDynamicsWorld* dynamics_world;

    // Tracking
    bool initialized;
    int body_count;

    // Helper functions
    static class btVector3 ToBullet(const Vector3& v);
    static Vector3 FromBullet(const class btVector3& v);
    static class btQuaternion ToBulletQuat(const Quaternion& q);
    static Quaternion FromBulletQuat(const class btQuaternion& q);

public:
    BulletEngine();
    ~BulletEngine() override;

    // ===== Initialization =====
    bool Initialize() override;
    void Shutdown() override;

    // ===== Simulation =====
    void Step(float deltaTime) override;
    void SetGravity(const Vector3& gravity) override;

    // ===== Rigid Bodies =====
    PhysicsBodyHandle CreateRigidBody(const RigidBodyDesc& desc) override;
    void DestroyRigidBody(PhysicsBodyHandle body) override;
    Transform GetBodyTransform(PhysicsBodyHandle body) override;
    void SetBodyTransform(PhysicsBodyHandle body, const Transform& transform) override;
    Vector3 GetBodyLinearVelocity(PhysicsBodyHandle body) override;
    void SetBodyLinearVelocity(PhysicsBodyHandle body, const Vector3& velocity) override;
    void ApplyForce(PhysicsBodyHandle body, const Vector3& force) override;
    void ApplyImpulse(PhysicsBodyHandle body, const Vector3& impulse) override;

    // ===== Collision Shapes =====
    CollisionShapeHandle CreateBoxShape(const BoxShapeDesc& desc) override;
    CollisionShapeHandle CreateSphereShape(const SphereShapeDesc& desc) override;
    CollisionShapeHandle CreateConvexHullShape(const ConvexHullShapeDesc& desc) override;
    void DestroyShape(CollisionShapeHandle shape) override;
    void AttachShape(PhysicsBodyHandle body, CollisionShapeHandle shape) override;

    // ===== Queries =====
    RaycastHit Raycast(const Ray& ray) override;
    bool OverlapPoint(const Vector3& point) override;

    // ===== Debug & Stats =====
    int GetBodyCount() const override;
    const char* GetEngineName() const override;
    const char* GetEngineVersion() const override;

    // Bullet-only accessors
    btDiscreteDynamicsWorld* GetDynamicsWorld() { return dynamics_world; }
    const btDiscreteDynamicsWorld* GetDynamicsWorld() const { return dynamics_world; }
};
