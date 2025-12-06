#include "BulletEngine.h"
#include "ChunkedVoxelCollision.h"
#include "VoxelWorld.h"
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <iostream>

// Helper conversions
btVector3 BulletEngine::ToBullet(const Vector3& v) {
    return btVector3(v.x, v.y, v.z);
}

Vector3 BulletEngine::FromBullet(const btVector3& v) {
    return Vector3(v.x(), v.y(), v.z());
}

btQuaternion BulletEngine::ToBulletQuat(const Quaternion& q) {
    return btQuaternion(q.x, q.y, q.z, q.w);
}

Quaternion BulletEngine::FromBulletQuat(const btQuaternion& q) {
    return Quaternion(q.x(), q.y(), q.z(), q.w());
}

BulletEngine::BulletEngine()
    : collision_config(nullptr)
    , dispatcher(nullptr)
    , broadphase(nullptr)
    , solver(nullptr)
    , dynamics_world(nullptr)
    , initialized(false)
    , body_count(0)
{
}

BulletEngine::~BulletEngine() {
    if (initialized) {
        Shutdown();
    }
}

// ===== Initialization =====

bool BulletEngine::Initialize() {
    if (initialized) {
        return true;
    }

    // Create collision configuration
    collision_config = new btDefaultCollisionConfiguration();

    // Create collision dispatcher
    dispatcher = new btCollisionDispatcher(collision_config);

    // Create broadphase (for efficient collision detection)
    broadphase = new btDbvtBroadphase();

    // Create constraint solver
    solver = new btSequentialImpulseConstraintSolver();

    // Create dynamics world
    dynamics_world = new btDiscreteDynamicsWorld(
        dispatcher,
        broadphase,
        solver,
        collision_config
    );

    // Set default gravity
    dynamics_world->setGravity(btVector3(0, -9.81f, 0));

    initialized = true;
    body_count = 0;  // Reset body count
    std::cout << "[BulletEngine] Initialized successfully\n";
    return true;
}

void BulletEngine::Shutdown() {
    if (!initialized) {
        return;
    }

    // Clean up voxel collision system
    if (voxel_collision) {
        voxel_collision->Shutdown();
        delete voxel_collision;
        voxel_collision = nullptr;
    }

    // Remove all rigid bodies
    if (dynamics_world) {
        for (int i = dynamics_world->getNumCollisionObjects() - 1; i >= 0; i--) {
            btCollisionObject* obj = dynamics_world->getCollisionObjectArray()[i];
            btRigidBody* body = btRigidBody::upcast(obj);
            if (body && body->getMotionState()) {
                delete body->getMotionState();
            }
            dynamics_world->removeCollisionObject(obj);
            delete obj;
        }
    }

    // Delete dynamics world
    delete dynamics_world;
    dynamics_world = nullptr;

    // Delete solver
    delete solver;
    solver = nullptr;

    // Delete broadphase
    delete broadphase;
    broadphase = nullptr;

    // Delete dispatcher
    delete dispatcher;
    dispatcher = nullptr;

    // Delete collision configuration
    delete collision_config;
    collision_config = nullptr;

    initialized = false;
    body_count = 0;  // Reset body count
    std::cout << "[BulletEngine] Shutdown complete\n";
}

// ===== Simulation =====

void BulletEngine::Step(float deltaTime) {
    if (!initialized || !dynamics_world) {
        return;
    }

    // Step simulation
    // maxSubSteps = 10, fixedTimeStep = 1/60.0f
    dynamics_world->stepSimulation(deltaTime, 10, 1.0f / 60.0f);
}

void BulletEngine::SetGravity(const Vector3& gravity) {
    if (dynamics_world) {
        dynamics_world->setGravity(ToBullet(gravity));
    }
}

// ===== Rigid Bodies =====

PhysicsBodyHandle BulletEngine::CreateRigidBody(const RigidBodyDesc& desc) {
    if (!initialized || !dynamics_world) {
        return nullptr;
    }

    // Create transform
    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(ToBullet(desc.transform.position));
    startTransform.setRotation(ToBulletQuat(desc.transform.rotation));

    // Create motion state
    btDefaultMotionState* motionState = new btDefaultMotionState(startTransform);

    // Create rigid body construction info
    // Note: mass and inertia will be set when shape is attached
    btRigidBody::btRigidBodyConstructionInfo rbInfo(
        desc.mass,
        motionState,
        nullptr,  // No shape yet
        btVector3(0, 0, 0)  // Local inertia
    );

    rbInfo.m_linearDamping = desc.linear_damping;
    rbInfo.m_angularDamping = desc.angular_damping;
    rbInfo.m_friction = desc.friction;
    rbInfo.m_restitution = desc.restitution;

    // Create rigid body
    btRigidBody* body = new btRigidBody(rbInfo);

    // Set kinematic if requested
    if (desc.kinematic) {
        body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
        body->setActivationState(DISABLE_DEACTIVATION);
    }

    // Set initial velocity
    body->setLinearVelocity(ToBullet(desc.linear_velocity));
    body->setAngularVelocity(ToBullet(desc.angular_velocity));

    // Add to world
    dynamics_world->addRigidBody(body);

    // Track body count
    body_count++;

    return static_cast<PhysicsBodyHandle>(body);
}

void BulletEngine::DestroyRigidBody(PhysicsBodyHandle body) {
    if (!body || !dynamics_world) {
        return;
    }

    btRigidBody* btBody = static_cast<btRigidBody*>(body);

    // Remove from world
    dynamics_world->removeRigidBody(btBody);

    // Delete motion state
    if (btBody->getMotionState()) {
        delete btBody->getMotionState();
    }

    // Delete collision shapes (if owned)
    // Note: In production, you'd want proper shape management
    // For now, shapes are managed separately via DestroyShape()

    // Delete body
    delete btBody;

    // Track body count
    body_count--;
}

Transform BulletEngine::GetBodyTransform(PhysicsBodyHandle body) {
    if (!body) {
        return Transform();
    }

    btRigidBody* btBody = static_cast<btRigidBody*>(body);
    btTransform trans;

    if (btBody->getMotionState()) {
        btBody->getMotionState()->getWorldTransform(trans);
    } else {
        trans = btBody->getWorldTransform();
    }

    Transform result;
    result.position = FromBullet(trans.getOrigin());
    result.rotation = FromBulletQuat(trans.getRotation());
    return result;
}

void BulletEngine::SetBodyTransform(PhysicsBodyHandle body, const Transform& transform) {
    if (!body) {
        return;
    }

    btRigidBody* btBody = static_cast<btRigidBody*>(body);
    btTransform trans;
    trans.setOrigin(ToBullet(transform.position));
    trans.setRotation(ToBulletQuat(transform.rotation));

    btBody->setWorldTransform(trans);
    if (btBody->getMotionState()) {
        btBody->getMotionState()->setWorldTransform(trans);
    }
}

Vector3 BulletEngine::GetBodyLinearVelocity(PhysicsBodyHandle body) {
    if (!body) {
        return Vector3::Zero();
    }

    btRigidBody* btBody = static_cast<btRigidBody*>(body);
    return FromBullet(btBody->getLinearVelocity());
}

void BulletEngine::SetBodyLinearVelocity(PhysicsBodyHandle body, const Vector3& velocity) {
    if (!body) {
        return;
    }

    btRigidBody* btBody = static_cast<btRigidBody*>(body);
    btBody->setLinearVelocity(ToBullet(velocity));
}

void BulletEngine::ApplyForce(PhysicsBodyHandle body, const Vector3& force) {
    if (!body) {
        return;
    }

    btRigidBody* btBody = static_cast<btRigidBody*>(body);
    btBody->applyCentralForce(ToBullet(force));
    btBody->activate(true);  // Wake up body for physics
}

void BulletEngine::ApplyImpulse(PhysicsBodyHandle body, const Vector3& impulse) {
    if (!body) {
        return;
    }

    btRigidBody* btBody = static_cast<btRigidBody*>(body);
    btBody->applyCentralImpulse(ToBullet(impulse));
    btBody->activate(true);  // Wake up body for physics
}

// ===== Collision Shapes =====

CollisionShapeHandle BulletEngine::CreateBoxShape(const BoxShapeDesc& desc) {
    btBoxShape* shape = new btBoxShape(ToBullet(desc.half_extents));
    return static_cast<CollisionShapeHandle>(shape);
}

CollisionShapeHandle BulletEngine::CreateSphereShape(const SphereShapeDesc& desc) {
    btSphereShape* shape = new btSphereShape(desc.radius);
    return static_cast<CollisionShapeHandle>(shape);
}

CollisionShapeHandle BulletEngine::CreateConvexHullShape(const ConvexHullShapeDesc& desc) {
    btConvexHullShape* shape = new btConvexHullShape();

    for (const auto& vertex : desc.vertices) {
        shape->addPoint(ToBullet(vertex), false);  // Don't recalc AABB yet
    }

    shape->recalcLocalAabb();  // Recalc AABB once at the end
    return static_cast<CollisionShapeHandle>(shape);
}

void BulletEngine::DestroyShape(CollisionShapeHandle shape) {
    if (!shape) {
        return;
    }

    btCollisionShape* btShape = static_cast<btCollisionShape*>(shape);
    delete btShape;
}

void BulletEngine::AttachShape(PhysicsBodyHandle body, CollisionShapeHandle shape) {
    if (!body || !shape || !dynamics_world) {
        return;
    }

    btRigidBody* btBody = static_cast<btRigidBody*>(body);
    btCollisionShape* btShape = static_cast<btCollisionShape*>(shape);

    // Remove body from world temporarily
    dynamics_world->removeRigidBody(btBody);

    // Set collision shape
    btBody->setCollisionShape(btShape);

    // Recalculate mass and inertia
    btScalar mass = 1.0f / btBody->getInvMass();  // Get mass from inverse mass
    if (mass < 0.0001f) {  // Very small inverse mass means infinite mass (static)
        mass = 0.0f;
    }

    btVector3 localInertia(0, 0, 0);
    if (mass > 0.0f) {  // Dynamic body
        btShape->calculateLocalInertia(mass, localInertia);
    }

    btBody->setMassProps(mass, localInertia);
    btBody->updateInertiaTensor();

    // Re-add to world with proper collision flags
    dynamics_world->addRigidBody(btBody);

    // Activate body (but allow it to sleep later if needed)
    btBody->setActivationState(ACTIVE_TAG);
    btBody->activate(true);
}

// ===== Queries =====

RaycastHit BulletEngine::Raycast(const Ray& ray) {
    RaycastHit result;
    result.hit = false;

    if (!dynamics_world) {
        return result;
    }

    btVector3 from = ToBullet(ray.origin);
    btVector3 to = ToBullet(ray.origin + ray.direction * ray.max_distance);

    btCollisionWorld::ClosestRayResultCallback rayCallback(from, to);
    dynamics_world->rayTest(from, to, rayCallback);

    if (rayCallback.hasHit()) {
        result.hit = true;
        result.point = FromBullet(rayCallback.m_hitPointWorld);
        result.normal = FromBullet(rayCallback.m_hitNormalWorld);
        result.distance = (result.point - ray.origin).Length();
        result.body = const_cast<void*>(static_cast<const void*>(rayCallback.m_collisionObject));
    }

    return result;
}

bool BulletEngine::OverlapPoint(const Vector3& point) {
    if (!dynamics_world) {
        return false;
    }

    // Simplified implementation: check if point is inside any body's AABB
    // Production implementation would use proper collision testing
    btVector3 btPoint = ToBullet(point);

    for (int i = 0; i < dynamics_world->getNumCollisionObjects(); i++) {
        btCollisionObject* obj = dynamics_world->getCollisionObjectArray()[i];
        btVector3 aabbMin, aabbMax;
        obj->getCollisionShape()->getAabb(obj->getWorldTransform(), aabbMin, aabbMax);

        if (btPoint.x() >= aabbMin.x() && btPoint.x() <= aabbMax.x() &&
            btPoint.y() >= aabbMin.y() && btPoint.y() <= aabbMax.y() &&
            btPoint.z() >= aabbMin.z() && btPoint.z() <= aabbMax.z()) {
            return true;
        }
    }

    return false;
}

// ===== Debug & Stats =====

int BulletEngine::GetBodyCount() const {
    return body_count;
}

const char* BulletEngine::GetEngineName() const {
    return "Bullet";
}

const char* BulletEngine::GetEngineVersion() const {
    return "3.25";  // Bullet3 version
}

// ===== Debris-Voxel Collision System =====

void BulletEngine::SetVoxelWorld(VoxelWorld* world) {
    voxel_world = world;

    // Initialize collision system if we have both world and physics
    if (voxel_world && dynamics_world && !voxel_collision) {
        voxel_collision = new ChunkedVoxelCollision(10.0f);  // 10m chunks
        voxel_collision->Initialize(dynamics_world, voxel_world);
        std::cout << "[BulletEngine] Voxel collision system initialized\n";
    }
}

void BulletEngine::BuildVoxelCollision() {
    if (!voxel_collision || !voxel_world) {
        std::cerr << "[BulletEngine] ERROR: Cannot build voxel collision - not initialized!\n";
        return;
    }

    std::cout << "[BulletEngine] Building voxel collision meshes...\n";
    voxel_collision->BuildChunksForWorld();
}

void BulletEngine::OnVoxelsChanged(const std::vector<Vector3>& positions) {
    if (!voxel_collision) return;

    // Mark affected chunks dirty
    for (const auto& pos : positions) {
        voxel_collision->MarkChunkDirty(pos);
    }
}

void BulletEngine::UpdateVoxelCollision(float deltaTime) {
    if (voxel_collision) {
        voxel_collision->Update(deltaTime);
    }
}
