# Static Collision Integration Plan

## Goals
1. **Give Bullet awareness of intact structures.** Build static or kinematic bodies that represent grounded voxel clusters so falling debris collides with surviving walls/floors instead of only the global ground plane.
2. **Keep the representation in sync with edits.** When voxels are added/removed or a cluster collapses, update or destroy the associated static collider so the physics view matches the voxel world.
3. **Limit cost.** Avoid per-voxel bodies. Prefer batched geometry (convex hulls or compound boxes per cluster) and incremental updates triggered by the structural analyzer/cluster tracker.

## Functional Requirements
1. **Cluster Extraction**
   - Reuse `VoxelClusterTracker` to identify grounded clusters (connected to ground plane).
   - Assign stable cluster IDs so we can map each cluster to a physics body and avoid duplicates.
2. **Collision Shape Generation**
   - Generate either:
     - Convex hulls per cluster (fast update, coarse accuracy), or
     - Compound of aligned boxes (more precise, higher cost).
   - Minimum: hull per grounded cluster with bounding box fallback.
3. **Kinematic Body Management**
   - Create Bullet bodies with `mass = 0` (static) or `kinematic = true`.
   - Store mapping `cluster_id → StaticBodyInfo` for updates/removals.
   - When analyzer marks a cluster as failed (i.e., it becomes debris), remove its static collider before spawning dynamic debris.
4. **Sync Hooks**
   - On voxel edits: mark affected clusters dirty. Rebuild static collider after next analyzer pass or on a timed schedule.
   - On analyzer success: update list of grounded clusters → add/update/remove colliders.
   - On analyzer failure: remove static collider for failed cluster, spawn dynamic debris (already implemented).
5. **Performance Safeguards**
   - Limit collider rebuilds per frame (budget). Queue work if many clusters dirty.
   - Collapse tiny clusters into a single `voxel_size` box to avoid degenerate hulls.
6. **Debug/Telemetry**
   - Console logs summarizing number of static colliders, rebuild time, and skipped clusters.
   - Optional wireframe overlay toggles to visualize static shapes for validation.

## Implementation Steps
1. **Data Plumbing**
   - Extend `VoxelClusterTracker` to report grounded vs. floating clusters (reuse existing grounding utility).
   - Add `StaticClusterBody` struct inside `VoxelPhysicsIntegration` with handles + cluster metadata.
2. **Shape Builders**
   - Implement helpers: `CreateStaticClusterShape(const VoxelCluster&)`, returning hull or compound.
   - Cache shapes per voxel size/material when possible.
3. **Lifecycle API**
   - `VoxelPhysicsIntegration::SyncStaticClusters(const std::vector<VoxelCluster>& grounded)`
     - Diff against existing map (add, update, remove).
     - Exposed through `VoxelPhysicsIntegration` so the demo/analyzer can call after each run.
4. **Analyzer Hook**
   - After `AnalysisResult` is produced, call `SyncStaticClusters` with current grounded clusters (clusters not in `result.failed_clusters` and still marked grounded).
   - Before spawning debris, call `RemoveStaticClusterBody(cluster_id)` to avoid double geometry.
5. **Demo Integration**
   - Queue syncs via analyzer loop to avoid doing heavy work mid-frame.
   - Provide config knobs (max clusters, rebuild interval) for tuning.
6. **Settling Improvements (optional)**
   - Use contact information (once static colliders exist) to only mark debris settled when touching a static body or ground plane.

## Validation
1. Unit tests for `SyncStaticClusters` using `MockPhysicsEngine` (ensure bodies created/removed as expected).
2. Manual demo tests:
   - Remove part of a wall: debris hits remaining wall.
   - Bridge collapse: debris lands on intact spans without falling through.
3. Performance profiling: log time spent building static shapes; ensure it stays within frame budget.

## Future Enhancements
1. **Mesh Simplification:** For large clusters, build marching-cubes style surface mesh → convex decomposition.
2. **Selective Collision:** Allow toggling collisions per material or cluster size to balance performance.
3. **Reintegration:** Once debris settles, optionally merge it back into voxel world and recreate static collider.
