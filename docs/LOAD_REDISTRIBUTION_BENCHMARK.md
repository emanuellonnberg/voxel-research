# Load Redistribution Benchmark (2025-12-04)

## Methodology
- Rebuilt `BenchmarkStructuralAnalysis` so it accepts an optional parameter file argument (see `examples/benchmark_structural_analysis.cpp`).
- Ran the full 12-scenario suite three times in `Debug` on Windows (MSVC, same environment as the rest of the repo).
- Each scenario executes between 5–20 iterations as defined by the benchmark harness. All numbers below are the average analysis time reported by the tool.
- Used three parameter presets:
  1. **Baseline heuristic** – `analysis_mode=HeuristicSupport` via `config/optimal_params.ini`.
  2. **Load redistribution (balanced)** – `analysis_mode=LoadRedistribution` with `load_capacity_scale=1.0`, `load_redistribution_iterations=10`, `load_redistribution_tolerance=1e-4`, ground absorption enabled (see `config/load_redistribution_balanced.ini`).
  3. **Load redistribution (conservative)** – same timestep/damping, but `load_capacity_scale=0.85`, `load_redistribution_iterations=16`, tighter tolerance (`5e-5`), and ground absorption disabled (`config/load_redistribution_conservative.ini`).

## Average Time (ms) by Scenario

| Scenario                | Heuristic (optimal) | Load-Redist Balanced | Load-Redist Conservative |
|------------------------|--------------------:|---------------------:|-------------------------:|
| Small Tower (10 vox.)  | 3.39 ± 1.10         | **2.92 ± 0.36**      | 3.01 ± 0.49              |
| Medium Tower (20 vox.) | **6.68 ± 0.85**     | 7.12 ± 1.66          | 7.03 ± 1.56              |
| Large Tower (50 vox.)  | 19.30 ± 1.65        | **18.01 ± 1.18**     | 18.79 ± 2.19             |
| Small Cube (5³)        | 110.14 ± 3.12       | **87.90 ± 2.76**     | 103.73 ± 3.88            |
| Medium Cube (10³)      | 542.81 ± 6.17       | **454.20 ± 10.04**   | 521.46 ± 5.25            |

_Bold_ indicates the fastest configuration for that row.

## Observations
- **Balanced redistribution helps large clusters**: the medium cube (999 voxels) dropped from ~543 ms to ~454 ms because we stop iterating early once overloaded supports hand off mass to neighbors/ground anchors. Small-to-mid structures stayed within ±10 % of the heuristic baseline.
- **Conservative tuning is slower but signals stress**: disabling ground absorption and tightening `load_capacity_scale` pushed medium-cube time back up to ~521 ms, but it also surfaced substantially more overload flags (222 nodes exceeded capacity vs. 0 in heuristic and 150 in the balanced run).
- **Load-specific failures now show up**:
  - Medium Tower logged ~5 overload-triggered nodes per iteration in the balanced set (`Detected … 5 load …`), whereas the heuristic run never tagged load failures.
  - Medium Cube produced 150 overload failures with the balanced config and 222 with the conservative config, giving downstream systems (fragmentation/physics) a richer picture of how the span collapses.
- **Ground absorption matters**: allowing anchors to eat excess load (balanced config) kept times down and limited overload counts; disabling it forced redistribution to stay within the graph, which both increased overload detections and slowed convergence.
- **Recommendation**: keep the balanced preset as the default load-redistribution profile, but expose the conservative preset through tooling so designers can intentionally stress-test worst-case fan-outs without editing INI files by hand.
