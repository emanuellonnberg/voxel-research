# FEM Validation Module

This module provides Finite Element Method (FEM) ground truth data for validating the structural analyzers.

## Purpose

- Generate accurate ground truth results using FEM
- Compare Spring and Max-Flow analyzers against FEM
- Measure actual accuracy percentages (not estimates)
- Validate that analyzers work correctly

## Architecture

```
validation/
├── README.md                 # This file
├── requirements.txt          # Python dependencies
├── test_structures.py        # Define test structures in voxels
├── fem_solver.py            # FEM solver (simplified or FEniCS-based)
├── generate_ground_truth.py # Generate all JSON files
├── results/                 # JSON output files
│   ├── tower_simple.json
│   ├── cantilever.json
│   ├── bridge.json
│   └── ...
└── visualize.py            # Optional: visualize FEM results
```

## Test Structure Format

Each test structure is defined as:
```python
{
    "name": "tower_simple",
    "voxels": [(x, y, z, material_id), ...],
    "damaged": [(x, y, z), ...],  # Which voxels are removed
    "voxel_size": 0.05,           # meters
}
```

## Ground Truth Format (JSON)

FEM results are exported as JSON:
```json
{
    "structure_name": "tower_simple",
    "fem_solver": "simplified_analytical",
    "should_fail": true,
    "failed_voxels": [
        {"position": [0.0, 0.05, 0.0], "stress": 12500000.0},
        {"position": [0.0, 0.10, 0.0], "stress": 11200000.0}
    ],
    "max_stress": 12500000.0,
    "computation_time_ms": 45.2
}
```

## Usage

### 1. Generate Ground Truth

```bash
cd validation
python generate_ground_truth.py
```

This creates JSON files in `validation/results/`.

### 2. Run C++ Validation Tests

```bash
cd build
./bin/VoxelTests --gtest_filter="FEMValidation.*"
```

C++ tests load JSON files and compare analyzer results.

### 3. Measure Accuracy

```bash
python measure_accuracy.py
```

Outputs:
```
Spring Analyzer: 87.3% accuracy vs FEM (13/15 structures correct)
Max-Flow Analyzer: 93.2% accuracy vs FEM (14/15 structures correct)
```

## Implementation Approaches

### Approach 1: Analytical Solutions (Easiest)

For simple structures (cantilever beam, simple tower), use closed-form solutions from mechanics:

**Cantilever beam stress:**
```
σ = M*y / I
where M = bending moment, y = distance from neutral axis, I = moment of inertia
```

**Simple compression:**
```
σ = F / A
where F = force, A = cross-sectional area
```

✅ **Pros:** Exact answers, fast, no dependencies
❌ **Cons:** Only works for simple geometries

### Approach 2: Simplified FEM (Medium)

Implement basic FEM with:
- Linear tetrahedral elements
- Simple stiffness matrix assembly
- Conjugate gradient solver
- Von Mises stress criterion

✅ **Pros:** Works for arbitrary geometries, educational
❌ **Cons:** Time to implement, may have bugs

### Approach 3: FEniCS Library (Most Accurate)

Use professional FEM library:

```bash
pip install fenics
```

✅ **Pros:** Industry-standard accuracy, well-tested
❌ **Cons:** Heavy dependency, complex API

### Approach 4: Hybrid (Recommended)

- Use analytical solutions for simple cases (tower, cantilever)
- Use numerical approximation for complex cases (bridge, building)
- Document which method was used for each test case

## Test Cases

### 1. Simple Tower (Analytical)
- 5 voxels stacked vertically
- Remove bottom voxel
- **Expected:** Tower fails (unsupported)
- **FEM method:** Analytical (weight > support)

### 2. Cantilever Beam (Analytical)
- 10 voxels horizontal, fixed at one end
- Remove support voxel
- **Expected:** Beam fails (bending stress > strength)
- **FEM method:** Beam equation σ = M*y/I

### 3. Bridge with Two Supports (Numerical)
- 20 voxels spanning gap
- Remove one support
- **Expected:** Depends on material and span
- **FEM method:** Simplified FEM or numerical

### 4. L-Shape Corner (Complex)
- L-shaped structure
- Remove corner voxel
- **Expected:** Complex stress distribution
- **FEM method:** Simplified FEM

### 5. Large Building (Stress Test)
- 100+ voxels
- Remove column
- **Expected:** Partial collapse
- **FEM method:** Simplified FEM

## Integration with C++ Tests

C++ test loads JSON and compares:

```cpp
TEST(FEMValidation, SimpleTower) {
    // Load ground truth
    auto fem_result = LoadFEMGroundTruth("validation/results/tower_simple.json");

    // Build same structure
    VoxelWorld world;
    BuildStructure(world, fem_result.structure_name);

    // Run analyzers
    StructuralAnalyzer spring;
    auto spring_result = spring.Analyze(world, fem_result.damaged);

    MaxFlowStructuralAnalyzer maxflow;
    auto maxflow_result = maxflow.Analyze(world, fem_result.damaged);

    // Compare
    EXPECT_EQ(spring_result.structure_failed, fem_result.should_fail);
    EXPECT_EQ(maxflow_result.structure_failed, fem_result.should_fail);

    // Measure accuracy (% of voxels that match FEM failure prediction)
    float spring_accuracy = CompareFailedVoxels(spring_result, fem_result);
    float maxflow_accuracy = CompareFailedVoxels(maxflow_result, fem_result);

    std::cout << "Spring accuracy: " << spring_accuracy << "%\n";
    std::cout << "Max-flow accuracy: " << maxflow_accuracy << "%\n";
}
```

## Next Steps

1. ✅ Create module structure
2. ⏳ Define test structures (5-10 cases covering simple to complex)
3. ⏳ Implement analytical solver for simple cases
4. ⏳ Implement simplified FEM or numerical approximation
5. ⏳ Generate ground truth JSON files
6. ⏳ Create C++ validation tests
7. ⏳ Run and measure actual accuracy
8. ⏳ Update main README with real numbers

## References

- Beer & Johnston, "Mechanics of Materials" (analytical solutions)
- Zienkiewicz & Taylor, "The Finite Element Method" (FEM theory)
- FEniCS documentation: https://fenicsproject.org/
- deal.II tutorial: https://www.dealii.org/
