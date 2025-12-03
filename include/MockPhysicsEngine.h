#pragma once

#include "IPhysicsEngine.h"
#include <unordered_map>
#include <memory>

/**
 * Mock Physics Engine (Week 5 Day 21)
 *
 * Simple physics engine for testing and prototyping.
 * Uses basic Euler integration without collision detection.
 *
 * Features:
 * - Rigid body simulation (position, velocity, forces)
 * - Gravity
 * - Simple damping
 * - NO collision detection (for now)
 * - Fast and deterministic
 *
 * Use Cases:
 * - Unit testing physics integration
 * - Prototyping without external dependencies
 * - Debugging voxel-to-physics conversion
 * - CI/CD environments without Bullet/PhysX
 */

class MockPhysicsEngine : public IPhysicsEngine {
private:
    struct MockRigidBody {
        Transform transform;
        Vector3 linear_velocity;
        Vector3 angular_velocity;
        float mass;
        float inv_mass;  // 1/mass (0 for static bodies)
        Vector3 accumulated_force;
        float linear_damping;
        float angular_damping;
        float friction;
        float restitution;
        bool kinematic;
        std::vector<CollisionShapeHandle> shapes;

        MockRigidBody(const RigidBodyDesc& desc)
            : transform(desc.transform)
            , linear_velocity(desc.linear_velocity)
            , angular_velocity(desc.angular_velocity)
            , mass(desc.mass)
            , inv_mass(desc.mass > 0.0f ? 1.0f / desc.mass : 0.0f)
            , accumulated_force(Vector3::Zero())
            , linear_damping(desc.linear_damping)
            , angular_damping(desc.angular_damping)
            , friction(desc.friction)
            , restitution(desc.restitution)
            , kinematic(desc.kinematic)
        {}
    };

    struct MockShape {
        ShapeType type;
        // Store shape data separately (no union due to constructor issues)
        BoxShapeDesc box;
        SphereShapeDesc sphere;
        std::vector<Vector3> convex_hull_vertices;  // For convex hulls

        MockShape(ShapeType t) : type(t), box(), sphere() {}
    };

    std::unordered_map<PhysicsBodyHandle, std::unique_ptr<MockRigidBody>> bodies;
    std::unordered_map<CollisionShapeHandle, std::unique_ptr<MockShape>> shapes;
    Vector3 gravity;
    int next_body_id;
    int next_shape_id;
    bool initialized;

public:
    MockPhysicsEngine()
        : gravity(0.0f, -9.81f, 0.0f)
        , next_body_id(1)
        , next_shape_id(1)
        , initialized(false)
    {}

    ~MockPhysicsEngine() override {
        if (initialized) {
            Shutdown();
        }
    }

    // ===== Initialization =====

    bool Initialize() override {
        if (initialized) {
            return true;
        }
        bodies.clear();
        shapes.clear();
        next_body_id = 1;
        next_shape_id = 1;
        gravity = Vector3(0.0f, -9.81f, 0.0f);
        initialized = true;
        return true;
    }

    void Shutdown() override {
        bodies.clear();
        shapes.clear();
        initialized = false;
    }

    // ===== Simulation =====

    void Step(float deltaTime) override {
        if (!initialized || deltaTime <= 0.0f) {
            return;
        }

        // Simple Euler integration (no collision detection)
        for (auto& [handle, body] : bodies) {
            if (body->kinematic || body->inv_mass == 0.0f) {
                continue;  // Skip kinematic and static bodies
            }

            // Apply gravity
            Vector3 force = body->accumulated_force + gravity * body->mass;

            // F = ma => a = F/m
            Vector3 acceleration = force * body->inv_mass;

            // Integrate velocity
            body->linear_velocity = body->linear_velocity + acceleration * deltaTime;

            // Apply damping
            body->linear_velocity = body->linear_velocity * (1.0f - body->linear_damping * deltaTime);
            body->angular_velocity = body->angular_velocity * (1.0f - body->angular_damping * deltaTime);

            // Integrate position
            body->transform.position = body->transform.position + body->linear_velocity * deltaTime;

            // Reset accumulated forces
            body->accumulated_force = Vector3::Zero();
        }
    }

    void SetGravity(const Vector3& g) override {
        gravity = g;
    }

    // ===== Rigid Bodies =====

    PhysicsBodyHandle CreateRigidBody(const RigidBodyDesc& desc) override {
        if (!initialized) {
            return nullptr;
        }

        PhysicsBodyHandle handle = reinterpret_cast<PhysicsBodyHandle>(static_cast<intptr_t>(next_body_id++));
        bodies[handle] = std::make_unique<MockRigidBody>(desc);
        return handle;
    }

    void DestroyRigidBody(PhysicsBodyHandle body) override {
        bodies.erase(body);
    }

    Transform GetBodyTransform(PhysicsBodyHandle body) override {
        auto it = bodies.find(body);
        if (it != bodies.end()) {
            return it->second->transform;
        }
        return Transform();
    }

    void SetBodyTransform(PhysicsBodyHandle body, const Transform& transform) override {
        auto it = bodies.find(body);
        if (it != bodies.end()) {
            it->second->transform = transform;
        }
    }

    Vector3 GetBodyLinearVelocity(PhysicsBodyHandle body) override {
        auto it = bodies.find(body);
        if (it != bodies.end()) {
            return it->second->linear_velocity;
        }
        return Vector3::Zero();
    }

    void SetBodyLinearVelocity(PhysicsBodyHandle body, const Vector3& velocity) override {
        auto it = bodies.find(body);
        if (it != bodies.end()) {
            it->second->linear_velocity = velocity;
        }
    }

    void ApplyForce(PhysicsBodyHandle body, const Vector3& force) override {
        auto it = bodies.find(body);
        if (it != bodies.end()) {
            it->second->accumulated_force = it->second->accumulated_force + force;
        }
    }

    void ApplyImpulse(PhysicsBodyHandle body, const Vector3& impulse) override {
        auto it = bodies.find(body);
        if (it != bodies.end()) {
            // Impulse = instant velocity change (J = m * Δv)
            it->second->linear_velocity = it->second->linear_velocity + impulse * it->second->inv_mass;
        }
    }

    // ===== Collision Shapes =====

    CollisionShapeHandle CreateBoxShape(const BoxShapeDesc& desc) override {
        if (!initialized) {
            return nullptr;
        }

        CollisionShapeHandle handle = reinterpret_cast<CollisionShapeHandle>(static_cast<intptr_t>(next_shape_id++));
        auto shape = std::make_unique<MockShape>(ShapeType::BOX);
        shape->box = desc;
        shapes[handle] = std::move(shape);
        return handle;
    }

    CollisionShapeHandle CreateSphereShape(const SphereShapeDesc& desc) override {
        if (!initialized) {
            return nullptr;
        }

        CollisionShapeHandle handle = reinterpret_cast<CollisionShapeHandle>(static_cast<intptr_t>(next_shape_id++));
        auto shape = std::make_unique<MockShape>(ShapeType::SPHERE);
        shape->sphere = desc;
        shapes[handle] = std::move(shape);
        return handle;
    }

    CollisionShapeHandle CreateConvexHullShape(const ConvexHullShapeDesc& desc) override {
        if (!initialized) {
            return nullptr;
        }

        CollisionShapeHandle handle = reinterpret_cast<CollisionShapeHandle>(static_cast<intptr_t>(next_shape_id++));
        auto shape = std::make_unique<MockShape>(ShapeType::CONVEX_HULL);
        shape->convex_hull_vertices = desc.vertices;
        shapes[handle] = std::move(shape);
        return handle;
    }

    void DestroyShape(CollisionShapeHandle shape) override {
        shapes.erase(shape);
    }

    void AttachShape(PhysicsBodyHandle body, CollisionShapeHandle shape) override {
        auto it = bodies.find(body);
        if (it != bodies.end()) {
            it->second->shapes.push_back(shape);
        }
    }

    // ===== Queries =====

    RaycastHit Raycast(const Ray& ray) override {
        // Mock raycast - not implemented
        RaycastHit hit;
        hit.hit = false;
        return hit;
    }

    bool OverlapPoint(const Vector3& point) override {
        // Mock overlap - not implemented
        return false;
    }

    // ===== Debug & Stats =====

    int GetBodyCount() const override {
        return static_cast<int>(bodies.size());
    }

    const char* GetEngineName() const override {
        return "Mock";
    }

    const char* GetEngineVersion() const override {
        return "1.0.0";
    }
};
