#include "PhysicsProxySimulator.h"

#include "Material.h"
#include "StructuralAnalyzer.h"
#include "VoxelWorld.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <unordered_map>

using Clock = std::chrono::high_resolution_clock;

#ifdef USE_BULLET
#include <btBulletDynamicsCommon.h>
#endif

PhysicsProxyResult PhysicsProxySimulator::Run(
    const PhysicsProxySettings& settings,
    VoxelWorld& world,
    const std::vector<SpringNode*>& nodes,
    const std::vector<Vector3>& damaged_positions)
{
    PhysicsProxyResult result;

#ifndef USE_BULLET
    (void)settings;
    (void)world;
    (void)nodes;
    (void)damaged_positions;

    std::cerr << "[PhysicsProxySimulator] Bullet Physics is disabled - skipping proxy simulation\n";
    return result;
#else
    if (nodes.empty()) {
        return result;
    }

    if (settings.max_bodies <= 0 || settings.simulation_time <= 0.0f || settings.time_step <= 0.0f) {
        std::cerr << "[PhysicsProxySimulator] Invalid proxy settings - skipping run\n";
        return result;
    }

    struct CandidateNode {
        SpringNode* node;
        float distance;
    };

    std::vector<CandidateNode> in_radius;
    std::vector<CandidateNode> all_nodes;
    in_radius.reserve(nodes.size());
    all_nodes.reserve(nodes.size());

    for (auto* node : nodes) {
        if (!node) {
            continue;
        }

        float closest = damaged_positions.empty() ?
                        0.0f :
                        std::numeric_limits<float>::max();

        if (!damaged_positions.empty()) {
            for (const auto& damage : damaged_positions) {
                float dist = node->position.Distance(damage);
                if (dist < closest) {
                    closest = dist;
                }
            }
        }

        all_nodes.push_back({node, closest});

        if (damaged_positions.empty() || closest <= settings.selection_radius) {
            in_radius.push_back({node, closest});
        }
    }

    const std::vector<CandidateNode>& selection_source =
        in_radius.empty() ? all_nodes : in_radius;

    if (selection_source.empty()) {
        return result;
    }

    std::vector<CandidateNode> selected = selection_source;

    std::sort(selected.begin(), selected.end(),
              [](const CandidateNode& a, const CandidateNode& b) {
                  return a.distance < b.distance;
              });

    if (static_cast<int>(selected.size()) > settings.max_bodies) {
        selected.resize(settings.max_bodies);
    }

    struct ProxyBody {
        SpringNode* node;
        Vector3 initial_position;
        btRigidBody* body;
    };

    auto build_start = Clock::now();

    auto* collision_config = new btDefaultCollisionConfiguration();
    auto* dispatcher = new btCollisionDispatcher(collision_config);
    auto* broadphase = new btDbvtBroadphase();
    auto* solver = new btSequentialImpulseConstraintSolver();
    auto* dynamics_world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collision_config);
    dynamics_world->setGravity(btVector3(0.0f, -9.81f, 0.0f));

    std::vector<btCollisionShape*> collision_shapes;
    std::vector<btRigidBody*> rigid_bodies;
    std::vector<ProxyBody> proxy_bodies;
    proxy_bodies.reserve(selected.size());

    auto cleanup = [&]() {
        for (auto* body : rigid_bodies) {
            dynamics_world->removeRigidBody(body);
            delete body->getMotionState();
            delete body;
        }
        for (auto* shape : collision_shapes) {
            delete shape;
        }
        delete dynamics_world;
        delete solver;
        delete broadphase;
        delete dispatcher;
        delete collision_config;
    };

    // Ground plane
    {
        auto* ground_shape = new btStaticPlaneShape(btVector3(0.0f, 1.0f, 0.0f), -settings.ground_level);
        collision_shapes.push_back(ground_shape);

        btTransform ground_transform;
        ground_transform.setIdentity();
        ground_transform.setOrigin(btVector3(0.0f, settings.ground_level, 0.0f));

        auto* motion_state = new btDefaultMotionState(ground_transform);

        btRigidBody::btRigidBodyConstructionInfo ground_info(0.0f, motion_state, ground_shape);
        auto* ground_body = new btRigidBody(ground_info);
        rigid_bodies.push_back(ground_body);
        dynamics_world->addRigidBody(ground_body);
    }

    const float voxel_size = world.GetVoxelSize();
    const btVector3 half_extents(voxel_size * 0.5f, voxel_size * 0.5f, voxel_size * 0.5f);
    auto* shared_shape = new btBoxShape(half_extents);
    collision_shapes.push_back(shared_shape);

    for (const auto& entry : selected) {
        auto* node = entry.node;
        if (!node) {
            continue;
        }

        const Material& mat = g_materials.GetMaterial(node->material_id);
        float node_mass = mat.GetVoxelMass(voxel_size);
        float simulated_mass = node_mass + node->mass_supported;
        if (simulated_mass < 0.05f) {
            simulated_mass = 0.05f;
        }
        if (simulated_mass > 2000.0f) {
            simulated_mass = 2000.0f;
        }

        float mass = node->is_ground_anchor ? 0.0f : simulated_mass;
        btVector3 inertia(0.0f, 0.0f, 0.0f);
        if (mass > 0.0f) {
            shared_shape->calculateLocalInertia(mass, inertia);
        }

        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(btVector3(node->position.x, node->position.y, node->position.z));

        auto* motion_state = new btDefaultMotionState(transform);

        btRigidBody::btRigidBodyConstructionInfo info(mass, motion_state, shared_shape, inertia);
        info.m_linearDamping = 0.12f;
        info.m_angularDamping = 0.15f;
        info.m_friction = 0.8f;
        info.m_restitution = 0.05f;

        auto* body = new btRigidBody(info);
        if (node->is_ground_anchor) {
            body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
        } else {
            body->setActivationState(DISABLE_DEACTIVATION);
        }

        dynamics_world->addRigidBody(body);
        rigid_bodies.push_back(body);

        proxy_bodies.push_back({node, node->position, body});
    }

    result.build_time_ms = std::chrono::duration<float, std::milli>(Clock::now() - build_start).count();

    if (proxy_bodies.empty()) {
        cleanup();
        return result;
    }

    auto simulate_start = Clock::now();
    const int max_substeps = static_cast<int>(settings.simulation_time / settings.time_step);
    const float fixed_timestep = settings.time_step;

    for (int step = 0; step < max_substeps; ++step) {
        dynamics_world->stepSimulation(fixed_timestep, 0);
    }

    result.simulate_time_ms = std::chrono::duration<float, std::milli>(Clock::now() - simulate_start).count();
    result.ran_simulation = true;
    result.bodies_simulated = static_cast<int>(proxy_bodies.size());

    for (const auto& proxy : proxy_bodies) {
        btTransform transform;
        proxy.body->getMotionState()->getWorldTransform(transform);
        btVector3 origin = transform.getOrigin();
        btVector3 linear_velocity = proxy.body->getLinearVelocity();

        float drop = proxy.initial_position.y - origin.getY();
        float velocity = linear_velocity.length();

        if (!proxy.node->is_ground_anchor &&
            (drop > settings.drop_threshold || velocity > settings.velocity_threshold)) {
            result.failed_node_ids.push_back(proxy.node->id);
        }
    }

    cleanup();
    return result;
#endif
}
