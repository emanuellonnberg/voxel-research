#include <gtest/gtest.h>
#include "IPhysicsEngine.h"
#include "MockPhysicsEngine.h"
#include "PhysXEngine.h"

/**
 * Physics Engine Abstraction Tests (Week 5 Day 21)
 *
 * Tests the generic IPhysicsEngine interface with different implementations.
 * Demonstrates engine-agnostic code that works with any backend.
 */

class PhysicsEngineTest : public ::testing::Test {
protected:
    IPhysicsEngine* physics;

    void SetUp() override {
        physics = nullptr;
    }

    void TearDown() override {
        if (physics) {
            physics->Shutdown();
            delete physics;
        }
    }

    // Helper to test any physics engine
    void TestEngineBasics(IPhysicsEngine* engine) {
        ASSERT_NE(engine, nullptr);

        // Initialize
        EXPECT_TRUE(engine->Initialize());
        EXPECT_GT(strlen(engine->GetEngineName()), 0);
        EXPECT_GT(strlen(engine->GetEngineVersion()), 0);

        // Set gravity
        engine->SetGravity(Vector3(0, -9.81f, 0));

        // Create rigid body
        RigidBodyDesc desc;
        desc.transform = Transform(Vector3(0, 10, 0));
        desc.mass = 1.0f;

        auto body = engine->CreateRigidBody(desc);
        ASSERT_NE(body, nullptr);
        EXPECT_EQ(engine->GetBodyCount(), 1);

        // Check transform
        Transform t = engine->GetBodyTransform(body);
        EXPECT_FLOAT_EQ(t.position.y, 10.0f);

        // Create collision shape
        auto shape = engine->CreateBoxShape(BoxShapeDesc(Vector3(0.5f, 0.5f, 0.5f)));
        ASSERT_NE(shape, nullptr);

        // Attach shape to body
        engine->AttachShape(body, shape);

        // Simulate
        engine->Step(1.0f / 60.0f);

        // Check that body fell due to gravity (for engines that support it)
        Transform t2 = engine->GetBodyTransform(body);
        // Mock engine should have gravity, so body should fall
        if (strcmp(engine->GetEngineName(), "Mock") == 0) {
            EXPECT_LT(t2.position.y, t.position.y);
        }

        // Cleanup
        engine->DestroyRigidBody(body);
        engine->DestroyShape(shape);
        EXPECT_EQ(engine->GetBodyCount(), 0);
    }
};

// ===== MockPhysicsEngine Tests =====

TEST_F(PhysicsEngineTest, MockEngine_Initialize) {
    physics = new MockPhysicsEngine();
    EXPECT_TRUE(physics->Initialize());
    EXPECT_STREQ(physics->GetEngineName(), "Mock");
    EXPECT_EQ(physics->GetBodyCount(), 0);
}

TEST_F(PhysicsEngineTest, MockEngine_CreateBody) {
    physics = new MockPhysicsEngine();
    physics->Initialize();

    RigidBodyDesc desc;
    desc.transform.position = Vector3(1, 2, 3);
    desc.mass = 5.0f;

    auto body = physics->CreateRigidBody(desc);
    ASSERT_NE(body, nullptr);
    EXPECT_EQ(physics->GetBodyCount(), 1);

    Transform t = physics->GetBodyTransform(body);
    EXPECT_FLOAT_EQ(t.position.x, 1.0f);
    EXPECT_FLOAT_EQ(t.position.y, 2.0f);
    EXPECT_FLOAT_EQ(t.position.z, 3.0f);

    physics->DestroyRigidBody(body);
    EXPECT_EQ(physics->GetBodyCount(), 0);
}

TEST_F(PhysicsEngineTest, MockEngine_Gravity) {
    physics = new MockPhysicsEngine();
    physics->Initialize();
    physics->SetGravity(Vector3(0, -9.81f, 0));

    RigidBodyDesc desc;
    desc.transform.position = Vector3(0, 10, 0);
    desc.mass = 1.0f;

    auto body = physics->CreateRigidBody(desc);

    // Initial position
    Transform t1 = physics->GetBodyTransform(body);
    EXPECT_FLOAT_EQ(t1.position.y, 10.0f);

    // Simulate for 1 second
    for (int i = 0; i < 60; i++) {
        physics->Step(1.0f / 60.0f);
    }

    // Should have fallen
    Transform t2 = physics->GetBodyTransform(body);
    EXPECT_LT(t2.position.y, t1.position.y);
    EXPECT_LT(t2.position.y, 7.0f);  // Fallen significantly (damping slows it down)

    physics->DestroyRigidBody(body);
}

TEST_F(PhysicsEngineTest, MockEngine_Velocity) {
    physics = new MockPhysicsEngine();
    physics->Initialize();
    physics->SetGravity(Vector3::Zero());  // No gravity for this test

    RigidBodyDesc desc;
    desc.transform.position = Vector3(0, 0, 0);
    desc.mass = 1.0f;
    desc.linear_velocity = Vector3(10, 0, 0);  // Moving in +X

    auto body = physics->CreateRigidBody(desc);

    // Initial position
    Transform t1 = physics->GetBodyTransform(body);
    EXPECT_FLOAT_EQ(t1.position.x, 0.0f);

    // Simulate for 1 second
    for (int i = 0; i < 60; i++) {
        physics->Step(1.0f / 60.0f);
    }

    // Should have moved in +X direction
    Transform t2 = physics->GetBodyTransform(body);
    EXPECT_GT(t2.position.x, 5.0f);  // Moved significantly (accounting for damping)

    physics->DestroyRigidBody(body);
}

TEST_F(PhysicsEngineTest, MockEngine_Force) {
    physics = new MockPhysicsEngine();
    physics->Initialize();
    physics->SetGravity(Vector3::Zero());

    RigidBodyDesc desc;
    desc.transform.position = Vector3(0, 0, 0);
    desc.mass = 1.0f;

    auto body = physics->CreateRigidBody(desc);

    // Apply upward force
    physics->ApplyForce(body, Vector3(0, 100, 0));

    // Simulate
    physics->Step(1.0f / 60.0f);

    // Should have moved upward
    Transform t = physics->GetBodyTransform(body);
    EXPECT_GT(t.position.y, 0.0f);

    physics->DestroyRigidBody(body);
}

TEST_F(PhysicsEngineTest, MockEngine_Impulse) {
    physics = new MockPhysicsEngine();
    physics->Initialize();
    physics->SetGravity(Vector3::Zero());

    RigidBodyDesc desc;
    desc.transform.position = Vector3(0, 0, 0);
    desc.mass = 1.0f;

    auto body = physics->CreateRigidBody(desc);

    // Apply impulse (instant velocity change)
    physics->ApplyImpulse(body, Vector3(0, 5, 0));  // 5 kg*m/s impulse

    // Check velocity changed instantly
    Vector3 vel = physics->GetBodyLinearVelocity(body);
    EXPECT_FLOAT_EQ(vel.y, 5.0f);  // Impulse / mass = 5 / 1 = 5 m/s

    physics->DestroyRigidBody(body);
}

TEST_F(PhysicsEngineTest, MockEngine_Shapes) {
    physics = new MockPhysicsEngine();
    physics->Initialize();

    // Create different shape types
    auto box = physics->CreateBoxShape(BoxShapeDesc(Vector3(1, 2, 3)));
    ASSERT_NE(box, nullptr);

    auto sphere = physics->CreateSphereShape(SphereShapeDesc(5.0f));
    ASSERT_NE(sphere, nullptr);

    ConvexHullShapeDesc hull_desc;
    hull_desc.vertices = {
        Vector3(-1, -1, -1),
        Vector3(1, -1, -1),
        Vector3(1, 1, -1),
        Vector3(-1, 1, -1),
        Vector3(0, 0, 1)
    };
    auto hull = physics->CreateConvexHullShape(hull_desc);
    ASSERT_NE(hull, nullptr);

    // Create body and attach shapes
    auto body = physics->CreateRigidBody(RigidBodyDesc());
    physics->AttachShape(body, box);
    physics->AttachShape(body, sphere);
    physics->AttachShape(body, hull);

    // Cleanup
    physics->DestroyRigidBody(body);
    physics->DestroyShape(box);
    physics->DestroyShape(sphere);
    physics->DestroyShape(hull);
}

TEST_F(PhysicsEngineTest, MockEngine_FullWorkflow) {
    physics = new MockPhysicsEngine();
    TestEngineBasics(physics);
}

// ===== PhysXEngine Tests =====

TEST_F(PhysicsEngineTest, PhysXEngine_IsStub) {
    physics = new PhysXEngine();
    EXPECT_STREQ(physics->GetEngineName(), "PhysX (Stub)");

    // Should throw "not implemented" for all operations
    EXPECT_THROW(physics->Initialize(), std::runtime_error);
}

// ===== Integration Tests =====

TEST_F(PhysicsEngineTest, Abstraction_EngineAgnostic) {
    // Demonstrate that code works with any engine

    // Test with Mock engine
    {
        IPhysicsEngine* engine = new MockPhysicsEngine();
        engine->Initialize();

        RigidBodyDesc desc;
        desc.mass = 10.0f;
        auto body = engine->CreateRigidBody(desc);

        EXPECT_EQ(engine->GetBodyCount(), 1);

        engine->DestroyRigidBody(body);
        engine->Shutdown();
        delete engine;
    }

    // Could test with Bullet engine here when available
    // Could test with PhysX engine here when implemented
}

TEST_F(PhysicsEngineTest, Abstraction_MultipleEngines) {
    // Can run multiple engines simultaneously
    MockPhysicsEngine mock1;
    MockPhysicsEngine mock2;

    mock1.Initialize();
    mock2.Initialize();

    auto body1 = mock1.CreateRigidBody(RigidBodyDesc());
    auto body2 = mock2.CreateRigidBody(RigidBodyDesc());

    EXPECT_EQ(mock1.GetBodyCount(), 1);
    EXPECT_EQ(mock2.GetBodyCount(), 1);

    mock1.Shutdown();
    mock2.Shutdown();
}
