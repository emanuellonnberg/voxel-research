# Structural Analyzer Modes Plan

## Mode 1: Heuristic Support (Current + Refinements)
- Add per-material max_support_distance and optional neighbor requirements.
- Track shortest path to ground and neighbor counts per node during flood-fill.
- Fail nodes whose path length/neighbor count exceed their material’s limits even if displacement is small.

## Mode 2: Stack-Depth Requirements ✅
- Introduced per-material `min_support_stack` (meters of contiguous voxels below).
- Added `vertical_stack_depth` to `SpringNode` populated after mass calculation.
- Failure pass now trips when `vertical_stack_depth` falls beneath the material threshold even if nodes remain connected.
- Remaining ideas: optional max span per material, better visualization/debug output for stack-related collapses.

## Mode 3: Load Redistribution ✅
- ✅ Added per-material `max_load_ratio` and per-node `redistributed_load`/`load_capacity` scaffolding.
- ✅ Implemented iterative redistribution loop with configurable iteration count/tolerance plus optional ground absorption step.
- ✅ Surfaced tuning knobs through the INI save/load path: `analysis_mode`, `load_capacity_scale`, `load_redistribution_iterations`, `load_redistribution_tolerance`, and `allow_ground_absorption`.
- ✅ Added telemetry (`LoadRedistributionStats`) so results report how much load moved, how many nodes remain overloaded, and how much mass dumped to ground anchors. Tests cover spreading, overload detection, capacity scaling, and the new toggles.
- Next: soak-test redistribution on large clustered structures (bridge/tower suites) to calibrate default tolerances, then decide whether to expose these toggles in higher-level gameplay settings or auto-select heuristics per material set.

## Mode 4: Physics Proxy
- Implemented baseline: `PhysicsProxySimulator` selects nodes within a configurable radius, spawns them as Bullet rigid bodies, runs a short burst simulation, and feeds failed node IDs back into `StructuralAnalyzer`. AnalysisResult now records proxy telemetry and the analyzer automatically falls back to heuristics when the proxy cannot run (e.g., Bullet disabled).
- ✅ Smarter selection heuristics now prioritize overloaded nodes, quantize picks onto a coarse grid, and stitch in their ground-support chains so proxy bursts cover full load paths. The new knobs (`proxy_selection_grid_size`, `proxy_borderline_load_ratio`, weight sliders, and `proxy_support_chain_depth`) are persisted through INI/config and regression-tested in `PhysicsProxySelectionTest`.
- ✅ Telemetry upgrade: every proxy rigid body now reports peak drop, peak linear speed, and the largest contact impulse observed during the Bullet burst (plus optional impulse history samples gated by `proxy_record_impulse_history`). These metrics let designers inspect borderline collapses and tune material thresholds precisely.
- ✅ Body pooling: reusable Bullet rigid bodies and shape caches drastically reduce proxy build cost. Result stats (`bodies_created`, `bodies_reused`) confirm cache hits in regression tests (`PhysicsProxyBodyReuseStats`).
- Remaining next steps: capture full impulse/constraint histories if needed and explore caching pools so repeated bursts amortize allocation cost.

Implementation outline for the next iteration:
  1. **Proxy Selection (DONE):** When heuristics/redistribution flag borderline clusters, collect voxels within a configurable radius, quantize to a coarse grid, and cap the total voxel count (e.g. 1–2 K) to keep proxy creation deterministic. Overloaded nodes bypass the radius gate and seed support-chain expansion.
  2. **Proxy Build:** Reuse `VoxelPhysicsIntegration` helpers to create a temporary `VoxelWorld`/`VoxelCluster`, then feed that into a stripped-down Bullet scene (no fancy materials, just box shapes + gravity). Map each proxy rigid body back to the originating `SpringNode`.
  3. **Burst Simulation:** Step the proxy for N substeps (e.g. 0.25–0.5 s of sim time) with high damping. Track contact impulses, constraint forces, and peak accelerations.
  4. **Failure Mapping:** Convert high-impulse events back into load multipliers per node. If a proxy body exceeds its material’s allowable stress or moves farther than the structural solver predicted, mark it as failed in the main analyzer.
  5. **Cleanup & Reuse:** Tear down Bullet state immediately after each burst, but keep a small cache of reusable rigid-body pools to amortize allocation cost.
  6. **Testing Hooks:** Start with micro-scenes (two-column bridge, cantilever) to validate that proxy verdicts match known physical behavior before wiring it into the full analyzer.

## Implementation Notes
1. Extend Material with new parameters (max_support_distance, min_support_stack, optional max_load).
2. Extend SpringNode metadata (ground_path_length, ertical_stack_depth, load-sharing arrays).
3. In StructuralAnalyzer::Analyze, branch based on params.analysis_mode and honor the new tuning knobs:
   - HeuristicSupport: run displacement + connectivity heuristics.
   - StackDepth: evaluate `vertical_stack_depth` vs. material requirements.
   - LoadRedistribution: execute redistribution loop (respecting iteration/tolerance/ground absorption params) before SolveDisplacements and emit telemetry through `AnalysisResult`.
   - PhysicsProxy: build/run proxy, feed results back, then fall back to solver if needed.
4. Add dedicated tests per mode (support distance failure, stack depth collapse, redistribution stress + telemetry/ground absorption, proxy micro-scene).
5. Integrate with VoxelPhysicsIntegration/Bullet for the proxy path (temporary rigid bodies) once Mode 4 work begins.
