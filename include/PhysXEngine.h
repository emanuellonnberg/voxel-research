#pragma once

#include "IPhysicsEngine.h"
#include <stdexcept>

/**
 * PhysX Physics Engine (Week 5 Day 21)
 *
 * STUB IMPLEMENTATION - For future development
 *
 * NVIDIA PhysX 5.x integration stub.
 * This class provides the interface for PhysX but throws "not implemented" errors.
 *
 * To implement:
 * 1. Install PhysX SDK (https://github.com/NVIDIAGameWorks/PhysX)
 * 2. Link against PhysX libraries (PhysX, PhysXCommon, PhysXFoundation, PhysXCooking)
 * 3. Implement each method using PhysX API
 * 4. Update CMakeLists.txt to conditionally compile when PhysX is available
 *
 * Advantages of PhysX:
 * - GPU acceleration (CUDA)
 * - Industry-standard (used in Unreal Engine, Unity)
 * - Advanced features (cloth, fluids, vehicles)
 * - Excellent performance on NVIDIA hardware
 *
 * Disadvantages:
 * - Platform limitations (no macOS, limited mobile)
 * - Larger binary size
 * - More complex setup
 * - Nvidia-specific optimizations
 *
 * See docs/PHYSX_INTEGRATION_GUIDE.md for detailed implementation guide.
 */

class PhysXEngine : public IPhysicsEngine {
private:
    // PhysX objects would go here:
    // PxFoundation* foundation;
    // PxPhysics* physics;
    // PxScene* scene;
    // PxMaterial* defaultMaterial;
    // PxCooking* cooking;

    bool initialized;

    void ThrowNotImplemented(const char* method) const {
        std::string msg = "PhysXEngine::";
        msg += method;
        msg += " - Not yet implemented. This is a stub for future development.";
        throw std::runtime_error(msg);
    }

public:
    PhysXEngine() : initialized(false) {}

    ~PhysXEngine() override {
        if (initialized) {
            Shutdown();
        }
    }

    // ===== Initialization =====

    bool Initialize() override {
        ThrowNotImplemented("Initialize");
        return false;

        // Future implementation outline:
        // 1. Create PxFoundation
        // 2. Create PxPhysics
        // 3. Create PxScene with default settings
        // 4. Create default PxMaterial
        // 5. Create PxCooking for mesh processing
        // 6. Set up error callbacks
    }

    void Shutdown() override {
        initialized = false;

        // Future implementation:
        // 1. Release all bodies and shapes
        // 2. Release scene
        // 3. Release cooking
        // 4. Release physics
        // 5. Release foundation
    }

    // ===== Simulation =====

    void Step(float deltaTime) override {
        ThrowNotImplemented("Step");

        // Future implementation:
        // scene->simulate(deltaTime);
        // scene->fetchResults(true);
    }

    void SetGravity(const Vector3& gravity) override {
        ThrowNotImplemented("SetGravity");

        // Future implementation:
        // scene->setGravity(PxVec3(gravity.x, gravity.y, gravity.z));
    }

    // ===== Rigid Bodies =====

    PhysicsBodyHandle CreateRigidBody(const RigidBodyDesc& desc) override {
        ThrowNotImplemented("CreateRigidBody");
        return nullptr;

        // Future implementation:
        // 1. Create PxTransform from desc.transform
        // 2. Create PxRigidDynamic or PxRigidStatic
        // 3. Set mass, damping, friction, restitution
        // 4. Add to scene
        // 5. Return handle (cast PxRigidActor* to void*)
    }

    void DestroyRigidBody(PhysicsBodyHandle body) override {
        ThrowNotImplemented("DestroyRigidBody");

        // Future implementation:
        // PxRigidActor* actor = static_cast<PxRigidActor*>(body);
        // actor->release();
    }

    Transform GetBodyTransform(PhysicsBodyHandle body) override {
        ThrowNotImplemented("GetBodyTransform");
        return Transform();

        // Future implementation:
        // PxRigidActor* actor = static_cast<PxRigidActor*>(body);
        // PxTransform t = actor->getGlobalPose();
        // Convert PxTransform to our Transform
    }

    void SetBodyTransform(PhysicsBodyHandle body, const Transform& transform) override {
        ThrowNotImplemented("SetBodyTransform");

        // Future implementation:
        // PxRigidActor* actor = static_cast<PxRigidActor*>(body);
        // PxTransform t = ConvertTransform(transform);
        // actor->setGlobalPose(t);
    }

    Vector3 GetBodyLinearVelocity(PhysicsBodyHandle body) override {
        ThrowNotImplemented("GetBodyLinearVelocity");
        return Vector3::Zero();
    }

    void SetBodyLinearVelocity(PhysicsBodyHandle body, const Vector3& velocity) override {
        ThrowNotImplemented("SetBodyLinearVelocity");
    }

    void ApplyForce(PhysicsBodyHandle body, const Vector3& force) override {
        ThrowNotImplemented("ApplyForce");

        // Future implementation:
        // PxRigidDynamic* actor = static_cast<PxRigidDynamic*>(body);
        // actor->addForce(PxVec3(force.x, force.y, force.z));
    }

    void ApplyImpulse(PhysicsBodyHandle body, const Vector3& impulse) override {
        ThrowNotImplemented("ApplyImpulse");

        // Future implementation:
        // PxRigidDynamic* actor = static_cast<PxRigidDynamic*>(body);
        // actor->addForce(PxVec3(impulse.x, impulse.y, impulse.z), PxForceMode::eIMPULSE);
    }

    // ===== Collision Shapes =====

    CollisionShapeHandle CreateBoxShape(const BoxShapeDesc& desc) override {
        ThrowNotImplemented("CreateBoxShape");
        return nullptr;

        // Future implementation:
        // PxBoxGeometry box(desc.half_extents.x, desc.half_extents.y, desc.half_extents.z);
        // PxShape* shape = physics->createShape(box, *defaultMaterial);
        // return static_cast<void*>(shape);
    }

    CollisionShapeHandle CreateSphereShape(const SphereShapeDesc& desc) override {
        ThrowNotImplemented("CreateSphereShape");
        return nullptr;

        // Future implementation:
        // PxSphereGeometry sphere(desc.radius);
        // PxShape* shape = physics->createShape(sphere, *defaultMaterial);
        // return static_cast<void*>(shape);
    }

    CollisionShapeHandle CreateConvexHullShape(const ConvexHullShapeDesc& desc) override {
        ThrowNotImplemented("CreateConvexHullShape");
        return nullptr;

        // Future implementation:
        // 1. Convert vertices to PxVec3 array
        // 2. Use PxCooking to create PxConvexMesh
        // 3. Create PxConvexMeshGeometry
        // 4. Create PxShape from geometry
    }

    void DestroyShape(CollisionShapeHandle shape) override {
        ThrowNotImplemented("DestroyShape");

        // Future implementation:
        // PxShape* pxShape = static_cast<PxShape*>(shape);
        // pxShape->release();
    }

    void AttachShape(PhysicsBodyHandle body, CollisionShapeHandle shape) override {
        ThrowNotImplemented("AttachShape");

        // Future implementation:
        // PxRigidActor* actor = static_cast<PxRigidActor*>(body);
        // PxShape* pxShape = static_cast<PxShape*>(shape);
        // actor->attachShape(*pxShape);
    }

    // ===== Queries =====

    RaycastHit Raycast(const Ray& ray) override {
        ThrowNotImplemented("Raycast");
        return RaycastHit();

        // Future implementation:
        // PxRaycastBuffer hit;
        // PxVec3 origin(ray.origin.x, ray.origin.y, ray.origin.z);
        // PxVec3 direction(ray.direction.x, ray.direction.y, ray.direction.z);
        // bool status = scene->raycast(origin, direction, ray.max_distance, hit);
        // Convert to RaycastHit
    }

    bool OverlapPoint(const Vector3& point) override {
        ThrowNotImplemented("OverlapPoint");
        return false;
    }

    // ===== Debug & Stats =====

    int GetBodyCount() const override {
        return 0;

        // Future implementation:
        // return scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
    }

    const char* GetEngineName() const override {
        return "PhysX (Stub)";
    }

    const char* GetEngineVersion() const override {
        return "5.x (Not Implemented)";
    }
};
