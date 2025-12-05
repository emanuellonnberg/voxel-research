#include "MaxFlowStructuralAnalyzer.h"
#include "StructuralAnalyzer.h"
#include "VoxelUtils.h"
#include "VoxelCluster.h"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <iostream>
#include <set>

static constexpr float GRAVITY = 9.81f;  // m/s²
static constexpr float INF = std::numeric_limits<float>::infinity();

MaxFlowStructuralAnalyzer::MaxFlowStructuralAnalyzer()
    : world(nullptr)
    , source_id(-1)
    , sink_id(-1)
    , last_max_flow(0.0f)
    , last_required_flow(0.0f)
    , flow_iterations(0)
{
}

MaxFlowStructuralAnalyzer::~MaxFlowStructuralAnalyzer() {
    Clear();
}

void MaxFlowStructuralAnalyzer::Clear() {
    // Delete all edges
    for (FlowEdge* edge : all_edges) {
        delete edge;
    }
    all_edges.clear();

    // Delete all nodes
    for (FlowNode* node : nodes) {
        delete node;
    }
    nodes.clear();

    position_to_node_id.clear();
    level.clear();
    iter.clear();

    source_id = -1;
    sink_id = -1;
    last_max_flow = 0.0f;
    last_required_flow = 0.0f;
    flow_iterations = 0;
}

AnalysisResult MaxFlowStructuralAnalyzer::Analyze(
    VoxelWorld& voxel_world,
    const std::vector<Vector3>& damaged_positions,
    float time_budget_ms)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    AnalysisResult result;
    world = &voxel_world;

    // Clear previous analysis
    Clear();

    // Step 1: Build flow network
    auto build_start = std::chrono::high_resolution_clock::now();
    BuildFlowNetwork(damaged_positions);
    auto build_end = std::chrono::high_resolution_clock::now();
    result.node_graph_time_ms = std::chrono::duration<float, std::milli>(build_end - build_start).count();
    result.nodes_analyzed = static_cast<int>(nodes.size());

    // If no voxel nodes, nothing to analyze
    if (nodes.size() <= 2) {  // Only source and sink
        result.structure_failed = false;
        result.calculation_time_ms = result.node_graph_time_ms;
        return result;
    }

    // Step 2: Compute required flow (total mass × g)
    last_required_flow = ComputeRequiredFlow();

    // Step 3: Compute max flow using Dinic's algorithm
    auto solver_start = std::chrono::high_resolution_clock::now();
    last_max_flow = ComputeMaxFlow();
    auto solver_end = std::chrono::high_resolution_clock::now();
    result.solver_time_ms = std::chrono::duration<float, std::milli>(solver_end - solver_start).count();
    result.iterations_used = flow_iterations;
    result.converged = true;  // Max-flow always converges

    // Step 4: Check if structure failed
    auto failure_start = std::chrono::high_resolution_clock::now();

    const float FLOW_TOLERANCE = 1e-3f;  // Small tolerance for floating-point comparison
    if (last_required_flow > last_max_flow + FLOW_TOLERANCE) {
        // Structure cannot support its own weight
        result.structure_failed = true;

        // Find saturated edges (failure points)
        std::vector<FlowEdge*> saturated = FindSaturatedEdges();

        // Find disconnected clusters
        result.failed_clusters = FindFailedClusters(saturated);
        result.num_failed_nodes = 0;
        for (const auto& cluster : result.failed_clusters) {
            result.num_failed_nodes += static_cast<int>(cluster.voxel_positions.size());
        }

        result.debug_info = "Max-flow analysis: Required " + std::to_string(last_required_flow) +
                           "N > Max " + std::to_string(last_max_flow) + "N";
    } else {
        // Structure is stable
        result.structure_failed = false;
        result.debug_info = "Max-flow analysis: Structure stable (Required " +
                           std::to_string(last_required_flow) + "N ≤ Max " +
                           std::to_string(last_max_flow) + "N)";
    }

    auto failure_end = std::chrono::high_resolution_clock::now();
    result.failure_detection_time_ms = std::chrono::duration<float, std::milli>(failure_end - failure_start).count();

    // Total time
    auto end_time = std::chrono::high_resolution_clock::now();
    result.calculation_time_ms = std::chrono::duration<float, std::milli>(end_time - start_time).count();

    return result;
}

// ============================================================================
// Network Construction
// ============================================================================

void MaxFlowStructuralAnalyzer::BuildFlowNetwork(const std::vector<Vector3>& damaged_positions) {
    // Step 1: Find voxels to analyze (surface voxels near damage)
    std::vector<Vector3> voxels_to_analyze;

    if (damaged_positions.empty()) {
        // No damage specified, analyze all surface voxels
        if (params.use_surface_only) {
            const auto& surface = world->GetSurfaceVoxels();
            voxels_to_analyze.assign(surface.begin(), surface.end());
        } else {
            voxels_to_analyze = world->GetAllVoxelPositions();
        }
    } else {
        // Find voxels within influence radius of damage
        std::unordered_set<Vector3, Vector3::Hash> unique_voxels;

        for (const Vector3& damage_pos : damaged_positions) {
            std::vector<Vector3> nearby;
            if (params.use_surface_only) {
                // Only surface voxels
                const auto& surface = world->GetSurfaceVoxels();
                for (const Vector3& pos : surface) {
                    if ((pos - damage_pos).Length() <= params.influence_radius) {
                        nearby.push_back(pos);
                    }
                }
            } else {
                // All voxels in radius
                nearby = world->GetVoxelsInRadius(damage_pos, params.influence_radius);
            }

            for (const Vector3& pos : nearby) {
                unique_voxels.insert(pos);
            }
        }

        voxels_to_analyze.assign(unique_voxels.begin(), unique_voxels.end());
    }

    // Step 2: Create super source and sink
    CreateSuperSourceAndSink();

    // Step 3: Create voxel nodes
    CreateVoxelNodes(voxels_to_analyze);

    // Step 4: Connect source to voxels (gravity source)
    ConnectSourceToVoxels();

    // Step 5: Connect voxels to each other (structural connections)
    ConnectVoxelToVoxel();

    // Step 6: Connect ground voxels to sink
    ConnectVoxelsToSink();
}

void MaxFlowStructuralAnalyzer::CreateSuperSourceAndSink() {
    source_id = static_cast<int>(nodes.size());
    FlowNode* source = new FlowNode(source_id, FlowNode::SOURCE);
    nodes.push_back(source);

    sink_id = static_cast<int>(nodes.size());
    FlowNode* sink = new FlowNode(sink_id, FlowNode::SINK);
    nodes.push_back(sink);
}

void MaxFlowStructuralAnalyzer::CreateVoxelNodes(const std::vector<Vector3>& positions) {
    for (const Vector3& pos : positions) {
        if (!world->HasVoxel(pos)) continue;

        Voxel voxel = world->GetVoxel(pos);
        if (voxel.material_id == 0) continue;  // Skip air

        int node_id = static_cast<int>(nodes.size());
        const Material& mat = g_materials.GetMaterial(voxel.material_id);
        float mass = mat.density * 1.0f;  // 1m³ voxel volume

        FlowNode* node = new FlowNode(node_id, pos, voxel.material_id, mass);
        nodes.push_back(node);
        position_to_node_id[pos] = node_id;
    }
}

void MaxFlowStructuralAnalyzer::ConnectSourceToVoxels() {
    // Source → Voxel edges with capacity = cumulative mass above + own mass
    // This allows load to flow from top voxels through bottom voxels to ground
    // Each voxel must support its own weight plus all weight above it
    for (size_t i = 2; i < nodes.size(); ++i) {  // Skip source(0) and sink(1)
        FlowNode* voxel = nodes[i];
        if (voxel->type != FlowNode::VOXEL) continue;

        // Calculate total mass supported by this voxel (own + above)
        float cumulative_mass = CalculateCumulativeMass(voxel->position);
        float load = cumulative_mass * GRAVITY;

        AddFlowEdge(source_id, voxel->id, load);
    }
}

void MaxFlowStructuralAnalyzer::ConnectVoxelToVoxel() {
    // Connect voxels to neighbors with capacity = structural strength
    // To avoid duplicate edges, only add edges where voxel.id < neighbor.id
    // Then add the reverse direction explicitly

    std::set<std::pair<int, int>> added_edges;

    for (size_t i = 2; i < nodes.size(); ++i) {
        FlowNode* voxel = nodes[i];
        if (voxel->type != FlowNode::VOXEL) continue;

        // Find neighbor positions
        std::vector<Vector3> neighbors = FindNeighborPositions(voxel->position);

        for (const Vector3& neighbor_pos : neighbors) {
            auto it = position_to_node_id.find(neighbor_pos);
            if (it == position_to_node_id.end()) continue;

            FlowNode* neighbor = nodes[it->second];

            // Check if we should create edges between these nodes
            if (IsDownwardOrLateralConnection(voxel, neighbor)) {
                // Avoid duplicates - only add edge if not already added
                int id1 = voxel->id;
                int id2 = neighbor->id;
                auto edge_pair = std::make_pair(std::min(id1, id2), std::max(id1, id2));

                if (added_edges.find(edge_pair) == added_edges.end()) {
                    float capacity = ComputeEdgeCapacity(voxel, neighbor);

                    // Add edges in BOTH directions with capacity
                    // This creates true bidirectional flow network
                    AddFlowEdge(voxel->id, neighbor->id, capacity);
                    AddFlowEdge(neighbor->id, voxel->id, capacity);

                    added_edges.insert(edge_pair);
                }
            }
        }
    }
}

void MaxFlowStructuralAnalyzer::ConnectVoxelsToSink() {
    // Ground voxels → Sink with infinite capacity
    for (size_t i = 2; i < nodes.size(); ++i) {
        FlowNode* voxel = nodes[i];
        if (voxel->type != FlowNode::VOXEL) continue;

        if (IsGroundAnchor(voxel->position)) {
            AddFlowEdge(voxel->id, sink_id, INF);
        }
    }
}

void MaxFlowStructuralAnalyzer::AddFlowEdge(int from, int to, float capacity) {
    // Create forward edge
    FlowEdge* forward = new FlowEdge(from, to, capacity);
    // Create reverse edge (for flow algorithm, initially 0 capacity)
    FlowEdge* reverse = new FlowEdge(to, from, 0.0f);

    forward->reverse = reverse;
    reverse->reverse = forward;

    nodes[from]->edges.push_back(forward);
    nodes[to]->edges.push_back(reverse);

    all_edges.push_back(forward);
    all_edges.push_back(reverse);
}

// ============================================================================
// Dinic's Max-Flow Algorithm
// ============================================================================

float MaxFlowStructuralAnalyzer::ComputeMaxFlow() {
    float total_flow = 0.0f;
    flow_iterations = 0;

    // Initialize iteration arrays
    level.assign(nodes.size(), -1);
    iter.assign(nodes.size(), 0);

    // Dinic's algorithm: repeat BFS + DFS until no augmenting path
    while (BuildLevelGraph()) {
        iter.assign(nodes.size(), 0);

        // Find blocking flows
        while (true) {
            float pushed = SendBlockingFlow(source_id, INF);
            if (pushed <= 0) break;  // No more blocking flows
            total_flow += pushed;
        }

        flow_iterations++;
    }

    return total_flow;
}

bool MaxFlowStructuralAnalyzer::BuildLevelGraph() {
    // BFS from source to create level graph
    level.assign(nodes.size(), -1);
    level[source_id] = 0;

    std::queue<int> q;
    q.push(source_id);

    while (!q.empty()) {
        int u = q.front();
        q.pop();

        for (FlowEdge* edge : nodes[u]->edges) {
            if (level[edge->to_id] < 0 && edge->ResidualCapacity() > 0) {
                level[edge->to_id] = level[u] + 1;
                q.push(edge->to_id);
            }
        }
    }

    // Return true if sink is reachable
    return level[sink_id] >= 0;
}

float MaxFlowStructuralAnalyzer::SendBlockingFlow(int node_id, float pushed_flow) {
    // DFS to find blocking flow
    if (node_id == sink_id) {
        return pushed_flow;
    }

    FlowNode* node = nodes[node_id];

    for (size_t& i = iter[node_id]; i < node->edges.size(); ++i) {
        FlowEdge* edge = node->edges[i];

        // Only use edges in level graph (going "forward")
        if (level[edge->to_id] == level[node_id] + 1 && edge->ResidualCapacity() > 0) {
            float bottleneck = std::min(pushed_flow, edge->ResidualCapacity());
            float flow = SendBlockingFlow(edge->to_id, bottleneck);

            if (flow > 0) {
                // Update flow on forward and reverse edges
                edge->flow += flow;
                edge->reverse->flow -= flow;
                return flow;
            }
        }
    }

    return 0.0f;  // No augmenting path found
}

// ============================================================================
// Failure Detection
// ============================================================================

float MaxFlowStructuralAnalyzer::ComputeRequiredFlow() const {
    // Sum of all voxel masses × g
    float total_load = 0.0f;
    for (size_t i = 2; i < nodes.size(); ++i) {  // Skip source and sink
        const FlowNode* node = nodes[i];
        if (node->type == FlowNode::VOXEL) {
            total_load += node->mass * GRAVITY;
        }
    }
    return total_load;
}

std::vector<FlowEdge*> MaxFlowStructuralAnalyzer::FindSaturatedEdges() const {
    std::vector<FlowEdge*> saturated;

    for (FlowEdge* edge : all_edges) {
        // Skip reverse edges and edges connected to source/sink
        if (edge->from_id == sink_id || edge->to_id == source_id) continue;
        if (edge->capacity <= 0) continue;  // Skip reverse edges

        if (edge->IsSaturated()) {
            saturated.push_back(edge);
        }
    }

    return saturated;
}

std::vector<VoxelCluster> MaxFlowStructuralAnalyzer::FindFailedClusters(
    const std::vector<FlowEdge*>& saturated)
{
    // Remove saturated edges from graph (conceptually)
    std::unordered_set<int> broken_connections;
    for (const FlowEdge* edge : saturated) {
        // Mark both directions as broken
        broken_connections.insert(edge->from_id * 100000 + edge->to_id);
        broken_connections.insert(edge->to_id * 100000 + edge->from_id);
    }

    // Find nodes disconnected from ground using BFS from ground nodes
    std::unordered_set<int> reachable_from_ground;
    std::queue<int> q;

    // Start BFS from all ground-anchored voxels
    for (size_t i = 2; i < nodes.size(); ++i) {
        const FlowNode* node = nodes[i];
        if (node->type == FlowNode::VOXEL && IsGroundAnchor(node->position)) {
            reachable_from_ground.insert(node->id);
            q.push(node->id);
        }
    }

    // BFS to find all nodes reachable from ground
    while (!q.empty()) {
        int u = q.front();
        q.pop();

        for (const FlowEdge* edge : nodes[u]->edges) {
            int neighbor_id = edge->to_id;

            // Skip if broken connection
            int connection_key = u * 100000 + neighbor_id;
            if (broken_connections.count(connection_key) > 0) continue;

            // Skip if already visited or not a voxel
            if (reachable_from_ground.count(neighbor_id) > 0) continue;
            if (nodes[neighbor_id]->type != FlowNode::VOXEL) continue;

            reachable_from_ground.insert(neighbor_id);
            q.push(neighbor_id);
        }
    }

    // Collect floating voxels
    std::vector<Vector3> floating_voxels;
    for (size_t i = 2; i < nodes.size(); ++i) {
        const FlowNode* node = nodes[i];
        if (node->type == FlowNode::VOXEL) {
            if (reachable_from_ground.count(node->id) == 0) {
                floating_voxels.push_back(node->position);
            }
        }
    }

    // Use VoxelClustering to group floating voxels into clusters
    std::vector<VoxelCluster> clusters;
    if (!floating_voxels.empty()) {
        std::unordered_set<Vector3, Vector3::Hash> visited;
        std::unordered_set<Vector3, Vector3::Hash> floating_set(
            floating_voxels.begin(), floating_voxels.end());

        // Use flood-fill to find connected components in floating voxels
        for (const Vector3& pos : floating_voxels) {
            if (visited.count(pos) > 0) continue;

            VoxelCluster cluster;
            std::queue<Vector3> q;
            q.push(pos);
            visited.insert(pos);

            // BFS to find all connected voxels
            while (!q.empty()) {
                Vector3 current = q.front();
                q.pop();
                cluster.voxel_positions.push_back(current);

                // Check 6-connected neighbors
                std::vector<Vector3> neighbors = FindNeighborPositions(current);
                for (const Vector3& neighbor : neighbors) {
                    if (visited.count(neighbor) == 0 && floating_set.count(neighbor) > 0) {
                        visited.insert(neighbor);
                        q.push(neighbor);
                    }
                }
            }

            // Calculate cluster properties
            VoxelClustering::CalculateClusterProperties(*world, cluster);
            clusters.push_back(cluster);
        }
    }

    return clusters;
}

// ============================================================================
// Helper Methods
// ============================================================================

bool MaxFlowStructuralAnalyzer::IsGroundAnchor(const Vector3& position) const {
    return position.y <= params.ground_level + 0.01f;
}

float MaxFlowStructuralAnalyzer::ComputeEdgeCapacity(
    const FlowNode* from,
    const FlowNode* to) const
{
    // Edge capacity = structural strength of connection
    const Material& mat_from = g_materials.GetMaterial(from->material_id);
    const Material& mat_to = g_materials.GetMaterial(to->material_id);

    // Use minimum of both materials (weakest link)
    // Convert from compressive strength (Pa) to approximate joint strength (N)
    // Assume contact area of ~0.1 m² (10cm × 10cm contact patch on 1m³ voxel)
    const float CONTACT_AREA = 0.1f;

    float capacity_from = mat_from.compressive_strength * CONTACT_AREA;
    float capacity_to = mat_to.compressive_strength * CONTACT_AREA;
    float capacity = std::min(capacity_from, capacity_to);

    // Apply tuning multiplier
    capacity *= params.edge_capacity_multiplier;

    return capacity;
}

bool MaxFlowStructuralAnalyzer::IsDownwardOrLateralConnection(
    const FlowNode* from,
    const FlowNode* to) const
{
    // Allow all connections - structures distribute loads in all directions
    // This creates a bidirectional flow network that properly models:
    // - Compression (downward)
    // - Tension (upward - reaction forces)
    // - Shear (lateral)
    // - Load redistribution through strongest available paths
    //
    // Physical justification: Newton's 3rd law - if voxel A pushes on B,
    // then B pushes back on A with equal force. Both directions needed!
    return true;
}

std::vector<Vector3> MaxFlowStructuralAnalyzer::FindNeighborPositions(
    const Vector3& position) const
{
    // Find 6-connected neighbors (face neighbors only)
    std::vector<Vector3> neighbors;
    neighbors.reserve(6);

    const Vector3 offsets[] = {
        Vector3(1, 0, 0), Vector3(-1, 0, 0),
        Vector3(0, 1, 0), Vector3(0, -1, 0),
        Vector3(0, 0, 1), Vector3(0, 0, -1)
    };

    for (const Vector3& offset : offsets) {
        Vector3 neighbor_pos = position + offset;
        if (world->HasVoxel(neighbor_pos)) {
            neighbors.push_back(neighbor_pos);
        }
    }

    return neighbors;
}

float MaxFlowStructuralAnalyzer::CalculateCumulativeMass(const Vector3& position) const
{
    // Calculate total mass this voxel must support (own mass + all mass above)
    float total_mass = 0.0f;
    float voxel_size = world->GetVoxelSize();

    // Raycast upward to find all voxels above this position
    Vector3 current_pos = position;

    while (true) {
        if (!world->HasVoxel(current_pos)) {
            break;
        }

        Voxel voxel = world->GetVoxel(current_pos);
        if (voxel.material_id == 0) {
            break;  // Hit air
        }

        // Add this voxel's mass
        const Material& mat = g_materials.GetMaterial(voxel.material_id);
        total_mass += mat.density * 1.0f;  // 1m³ voxel volume

        // Move up one voxel
        current_pos.y += voxel_size;
    }

    return total_mass;
}
