#include "PhysicsProxySimulator.h"

#include "Material.h"
#include "StructuralAnalyzer.h"
#include "VoxelWorld.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <unordered_set>

using Clock = std::chrono::high_resolution_clock;

#ifdef USE_BULLET
#include <btBulletDynamicsCommon.h>

class ProxyMotionState : public btMotionState {
public:
    ProxyMotionState() {
        transform_.setIdentity();
    }

    void getWorldTransform(btTransform& worldTrans) const override {
        worldTrans = transform_;
    }

    void setWorldTransform(const btTransform& worldTrans) override {
        transform_ = worldTrans;
    }

    void SetTransform(const btTransform& worldTrans) {
        transform_ = worldTrans;
    }

private:
    btTransform transform_;
};

class BoxShapeCache {
public:
    ~BoxShapeCache() {
        for (auto& kv : shapes_) {
            delete kv.second;
        }
    }

    btBoxShape* GetShape(float voxel_size) {
        auto it = shapes_.find(voxel_size);
        if (it != shapes_.end()) {
            return it->second;
        }
        btVector3 half_extents(voxel_size * 0.5f, voxel_size * 0.5f, voxel_size * 0.5f);
        auto* shape = new btBoxShape(half_extents);
        shapes_[voxel_size] = shape;
        return shape;
    }

private:
    std::unordered_map<float, btBoxShape*> shapes_;
};

class RigidBodyPool {
public:
    ~RigidBodyPool() {
        for (auto* body : pool_) {
            if (!body) {
                continue;
            }
            delete body->getMotionState();
            delete body;
        }
    }

    btRigidBody* Acquire(btCollisionShape* shape, bool& reused) {
        btRigidBody* body = nullptr;
        if (!pool_.empty()) {
            body = pool_.back();
            pool_.pop_back();
            reused = true;
        } else {
            reused = false;
            body = Create(shape);
        }

        if (!body) {
            reused = false;
            return nullptr;
        }

        body->setCollisionShape(shape);
        body->setCollisionFlags(btCollisionObject::CF_DYNAMIC_OBJECT);
        body->setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
        body->setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
        body->clearForces();
        body->setActivationState(ACTIVE_TAG);
        return body;
    }

    void Release(btRigidBody* body) {
        if (!body) {
            return;
        }
        body->setCollisionShape(nullptr);
        body->setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
        body->setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
        body->clearForces();
        body->setUserIndex(-1);
        pool_.push_back(body);
    }

private:
    btRigidBody* Create(btCollisionShape* shape) {
        auto* motion_state = new ProxyMotionState();
        btRigidBody::btRigidBodyConstructionInfo info(1.0f, motion_state, shape);
        auto* body = new btRigidBody(info);
        body->setUserIndex(-1);
        return body;
    }

    std::vector<btRigidBody*> pool_;
};

BoxShapeCache& GetShapeCache() {
    static BoxShapeCache cache;
    return cache;
}

RigidBodyPool& GetRigidBodyPool() {
    static RigidBodyPool pool;
    return pool;
}

#endif

namespace {
struct GridKey {
    int x;
    int y;
    int z;

    bool operator==(const GridKey& other) const noexcept {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct GridKeyHash {
    size_t operator()(const GridKey& key) const noexcept {
        size_t h1 = std::hash<int>{}(key.x);
        size_t h2 = std::hash<int>{}(key.y);
        size_t h3 = std::hash<int>{}(key.z);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

static float ComputeDistanceToDamage(
    const SpringNode* node,
    const std::vector<Vector3>& damaged_positions)
{
    if (!node || damaged_positions.empty()) {
        return 0.0f;
    }

    float closest = std::numeric_limits<float>::max();
    for (const auto& damage : damaged_positions) {
        closest = std::min(closest, node->position.Distance(damage));
    }
    return closest;
}

static float ComputeOverloadRatio(const SpringNode* node) {
    if (!node || node->is_ground_anchor) {
        return 0.0f;
    }

    float capacity = node->load_capacity;
    if (!std::isfinite(capacity) || capacity <= 0.0f) {
        return 0.0f;
    }

    float ratio = node->redistributed_load / capacity;
    if (!std::isfinite(ratio)) {
        return 0.0f;
    }
    return std::max(0.0f, ratio);
}

static float ComputeSupportScore(const SpringNode* node) {
    if (!node) {
        return 0.0f;
    }
    if (!std::isfinite(node->ground_path_length) || node->ground_path_length <= 0.0f) {
        return node->is_ground_anchor ? 1.0f : 0.0f;
    }
    return 1.0f / (1.0f + node->ground_path_length);
}
} // namespace

std::vector<SpringNode*> PhysicsProxySimulator::SelectCandidateNodes(
    const PhysicsProxySettings& settings,
    VoxelWorld& world,
    const std::vector<SpringNode*>& nodes,
    const std::vector<Vector3>& damaged_positions) const
{
    std::vector<SpringNode*> selection;
    if (nodes.empty() || settings.max_bodies <= 0) {
        return selection;
    }

    struct RawCandidate {
        SpringNode* node;
        float distance;
        float overload_ratio;
        float support_score;
        bool overloaded;
    };

    std::vector<RawCandidate> raw;
    raw.reserve(nodes.size());
    bool has_damage = !damaged_positions.empty();
    float radius = std::max(0.001f, settings.selection_radius);
    float grid_size = settings.selection_grid_size;
    if (grid_size <= 0.0f) {
        grid_size = world.GetVoxelSize() * 0.5f;
        if (grid_size <= 0.0f) {
            grid_size = 0.05f;
        }
    }

    for (auto* node : nodes) {
        if (!node) {
            continue;
        }

        float distance = has_damage ? ComputeDistanceToDamage(node, damaged_positions) : 0.0f;
        float overload_ratio = ComputeOverloadRatio(node);
        bool overloaded = (settings.borderline_load_ratio > 0.0f) &&
                          (overload_ratio >= settings.borderline_load_ratio);
        float support_score = ComputeSupportScore(node);

        raw.push_back({node, distance, overload_ratio, support_score, overloaded});
    }

    std::vector<RawCandidate> prioritized;
    prioritized.reserve(raw.size());
    for (const auto& cand : raw) {
        bool within_radius = !has_damage || cand.distance <= radius;
        if (within_radius || cand.overloaded) {
            prioritized.push_back(cand);
        }
    }

    if (prioritized.empty()) {
        prioritized = raw;  // Fallback to entire set
    }

    struct RankedCandidate {
        SpringNode* node;
        float distance;
        float score;
    };

    auto quantize = [grid_size](const Vector3& pos) -> GridKey {
        return GridKey{
            static_cast<int>(std::floor(pos.x / grid_size)),
            static_cast<int>(std::floor(pos.y / grid_size)),
            static_cast<int>(std::floor(pos.z / grid_size))
        };
    };

    std::unordered_map<GridKey, RankedCandidate, GridKeyHash> cell_best;
    for (const auto& cand : prioritized) {
        if (!cand.node) {
            continue;
        }

        float distance_score = has_damage ?
            std::max(0.0f, 1.0f - (cand.distance / radius)) :
            0.5f;

        float overload_component = cand.overload_ratio * settings.overload_priority_weight;
        float support_component = cand.support_score * settings.support_priority_weight;
        float score = distance_score + overload_component + support_component;

        auto key = quantize(cand.node->position);
        auto it = cell_best.find(key);
        if (it == cell_best.end() || score > it->second.score) {
            cell_best[key] = {cand.node, cand.distance, score};
        }
    }

    std::vector<RankedCandidate> sorted;
    sorted.reserve(cell_best.size());
    for (auto& kv : cell_best) {
        sorted.push_back(kv.second);
    }

    std::sort(sorted.begin(), sorted.end(),
              [](const RankedCandidate& a, const RankedCandidate& b) {
                  if (std::abs(a.score - b.score) < 1e-5f) {
                      return a.distance < b.distance;
                  }
                  return a.score > b.score;
              });

    std::unordered_set<const SpringNode*> seen;
    seen.reserve(sorted.size());
    for (const auto& cand : sorted) {
        if (!cand.node) {
            continue;
        }
        if (!seen.insert(cand.node).second) {
            continue;
        }
        selection.push_back(cand.node);
        if (static_cast<int>(selection.size()) >= settings.max_bodies) {
            break;
        }
    }

    if (settings.support_chain_depth > 0 && !selection.empty()) {
        const int limit = settings.max_bodies;
        size_t seed_count = selection.size();
        for (size_t i = 0; i < seed_count && static_cast<int>(selection.size()) < limit; ++i) {
            SpringNode* current = selection[i];
            if (!current) {
                continue;
            }

            SpringNode* step_node = current;
            for (int depth = 0; depth < settings.support_chain_depth; ++depth) {
                if (!step_node || static_cast<int>(selection.size()) >= limit) {
                    break;
                }

                SpringNode* next = nullptr;
                float best_path = step_node->ground_path_length;

                for (auto* neighbor : step_node->neighbors) {
                    if (!neighbor || !std::isfinite(neighbor->ground_path_length)) {
                        continue;
                    }
                    if (!next || neighbor->ground_path_length + 1e-4f < best_path) {
                        best_path = neighbor->ground_path_length;
                        next = neighbor;
                    }
                }

                if (!next || next == step_node) {
                    break;
                }

                step_node = next;
                if (seen.insert(step_node).second) {
                    selection.push_back(step_node);
                }

                if (step_node->is_ground_anchor) {
                    break;
                }
            }
        }
    }

    if (static_cast<int>(selection.size()) > settings.max_bodies) {
        selection.resize(settings.max_bodies);
    }

    return selection;
}

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

    auto selected_nodes = SelectCandidateNodes(settings, world, nodes, damaged_positions);
    if (selected_nodes.empty()) {
        return result;
    }

    struct ProxyBody {
        SpringNode* node;
        Vector3 initial_position;
        btRigidBody* body;
        float final_drop;
        float final_velocity;

        ProxyBody()
            : node(nullptr)
            , initial_position(Vector3::Zero())
            , body(nullptr)
            , final_drop(0.0f)
            , final_velocity(0.0f)
        {}
    };

    auto build_start = Clock::now();

    auto* collision_config = new btDefaultCollisionConfiguration();
    auto* dispatcher = new btCollisionDispatcher(collision_config);
    auto* broadphase = new btDbvtBroadphase();
    auto* solver = new btSequentialImpulseConstraintSolver();
    auto* dynamics_world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collision_config);
    dynamics_world->setGravity(btVector3(0.0f, -9.81f, 0.0f));

    std::vector<btCollisionShape*> collision_shapes;
    std::vector<btRigidBody*> owned_rigid_bodies;
    std::vector<btRigidBody*> pooled_rigid_bodies;
    std::vector<ProxyBody> proxy_bodies;
    proxy_bodies.reserve(selected_nodes.size());
    std::unordered_map<const btRigidBody*, PhysicsProxyResult::BodyTelemetry> telemetry_map;
    int reused_bodies = 0;
    int created_bodies = 0;

    auto cleanup = [&]() {
        for (auto* body : pooled_rigid_bodies) {
            dynamics_world->removeRigidBody(body);
            GetRigidBodyPool().Release(body);
        }
        for (auto* body : owned_rigid_bodies) {
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
        owned_rigid_bodies.push_back(ground_body);
        dynamics_world->addRigidBody(ground_body);
    }

    const float voxel_size = world.GetVoxelSize();
    auto* shared_shape = GetShapeCache().GetShape(voxel_size);

    for (auto* node : selected_nodes) {
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

        bool reused = false;
        btRigidBody* body = GetRigidBodyPool().Acquire(shared_shape, reused);
        if (!body) {
            continue;
        }
        if (reused) {
            reused_bodies++;
        } else {
            created_bodies++;
        }

        auto* motion_state = static_cast<ProxyMotionState*>(body->getMotionState());
        if (motion_state) {
            motion_state->SetTransform(transform);
        }
        body->setWorldTransform(transform);
        body->setCenterOfMassTransform(transform);
        body->setMassProps(mass, inertia);
        body->updateInertiaTensor();
        body->setDamping(0.12f, 0.15f);
        body->setFriction(0.8f);
        body->setRestitution(0.05f);

        if (node->is_ground_anchor) {
            body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
            body->setActivationState(WANTS_DEACTIVATION);
        } else {
            body->setCollisionFlags((body->getCollisionFlags() | btCollisionObject::CF_DYNAMIC_OBJECT) &
                                    ~btCollisionObject::CF_STATIC_OBJECT);
            body->setActivationState(DISABLE_DEACTIVATION);
        }

        dynamics_world->addRigidBody(body);
        pooled_rigid_bodies.push_back(body);

        proxy_bodies.push_back(ProxyBody());
        proxy_bodies.back().node = node;
        proxy_bodies.back().initial_position = node->position;
        proxy_bodies.back().body = body;

        PhysicsProxyResult::BodyTelemetry telemetry;
        telemetry.node_id = node->id;
        telemetry_map[body] = telemetry;
    }

    result.build_time_ms = std::chrono::duration<float, std::milli>(Clock::now() - build_start).count();

    if (proxy_bodies.empty()) {
        cleanup();
        return result;
    }

    auto simulate_start = Clock::now();
    const int max_substeps = static_cast<int>(settings.simulation_time / settings.time_step);
    const float fixed_timestep = settings.time_step;

    auto update_body_peaks = [&](ProxyBody& proxy) {
        if (!proxy.body) {
            return;
        }
        auto it = telemetry_map.find(proxy.body);
        if (it == telemetry_map.end()) {
            return;
        }

        btTransform transform;
        proxy.body->getMotionState()->getWorldTransform(transform);
        float drop = proxy.initial_position.y - transform.getOrigin().getY();
        float speed = proxy.body->getLinearVelocity().length();
        if (drop > it->second.peak_drop) {
            it->second.peak_drop = drop;
        }
        if (speed > it->second.peak_speed) {
            it->second.peak_speed = speed;
        }
    };

    auto accumulate_impulse = [&](const btRigidBody* body, float impulse) {
        if (!body) {
            return;
        }
        auto it = telemetry_map.find(body);
        if (it == telemetry_map.end()) {
            return;
        }
        if (impulse > it->second.peak_impulse) {
            it->second.peak_impulse = impulse;
        }

        if (settings.record_impulse_history && settings.max_impulse_samples > 0) {
            auto& history = it->second.impulse_samples;
            if (static_cast<int>(history.size()) < settings.max_impulse_samples) {
                history.push_back(impulse);
            }
        }
    };

    btDispatcher* contact_dispatcher = dynamics_world->getDispatcher();

    for (int step = 0; step < max_substeps; ++step) {
        dynamics_world->stepSimulation(fixed_timestep, 0);

        for (auto& proxy : proxy_bodies) {
            update_body_peaks(proxy);
        }

        if (contact_dispatcher) {
            int manifold_count = contact_dispatcher->getNumManifolds();
            for (int i = 0; i < manifold_count; ++i) {
                btPersistentManifold* manifold = contact_dispatcher->getManifoldByIndexInternal(i);
                if (!manifold) {
                    continue;
                }
                const btRigidBody* body0 = btRigidBody::upcast(manifold->getBody0());
                const btRigidBody* body1 = btRigidBody::upcast(manifold->getBody1());
                int contacts = manifold->getNumContacts();
                for (int j = 0; j < contacts; ++j) {
                    const btManifoldPoint& pt = manifold->getContactPoint(j);
                    float impulse = std::abs(pt.getAppliedImpulse());
                    if (impulse <= 0.0f) {
                        continue;
                    }
                    accumulate_impulse(body0, impulse);
                    accumulate_impulse(body1, impulse);
                }
            }
        }
    }

    result.simulate_time_ms = std::chrono::duration<float, std::milli>(Clock::now() - simulate_start).count();
    result.ran_simulation = true;
    result.bodies_simulated = static_cast<int>(proxy_bodies.size());
    result.bodies_reused = reused_bodies;
    result.bodies_created = created_bodies;

    result.body_stats.reserve(proxy_bodies.size());

    for (auto& proxy : proxy_bodies) {
        btTransform transform;
        proxy.body->getMotionState()->getWorldTransform(transform);
        btVector3 origin = transform.getOrigin();
        btVector3 linear_velocity = proxy.body->getLinearVelocity();

        float drop = proxy.initial_position.y - origin.getY();
        float velocity = linear_velocity.length();
        proxy.final_drop = drop;
        proxy.final_velocity = velocity;

        auto it = telemetry_map.find(proxy.body);
        if (it != telemetry_map.end()) {
            if (drop > it->second.peak_drop) {
                it->second.peak_drop = drop;
            }
            if (velocity > it->second.peak_speed) {
                it->second.peak_speed = velocity;
            }
            result.body_stats.push_back(it->second);
        }

        if (!proxy.node->is_ground_anchor &&
            (drop > settings.drop_threshold || velocity > settings.velocity_threshold)) {
            result.failed_node_ids.push_back(proxy.node->id);
        }
    }

    cleanup();
    return result;
#endif
}
