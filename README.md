# Voxel Destruction Engine

An 8-week prototype implementing realistic voxel-based structural destruction with physics simulation.

## Project Structure

```
voxel-research/
├── src/
│   ├── core/           # Core utilities (Vector3, Material)
│   ├── voxel/          # Voxel world system (Week 1-2)
│   ├── structural/     # Structural integrity analysis (Week 3-4)
│   ├── physics/        # Physics simulation (Week 5-6)
│   └── rendering/      # OpenGL rendering
├── include/            # Public headers
├── tests/              # Unit tests
├── examples/           # Example programs
└── docs/               # Documentation

```

## Features

### Current (Week 3 Day 15)
- ✅ Project structure
- ✅ CMake build system
- ✅ Vector3 math library (29 tests)
- ✅ Material system with realistic physical properties (18 tests)
- ✅ Voxel struct with damage system (4 tests)
- ✅ VoxelWorld with sparse storage (27 tests)
- ✅ Spatial queries (radius, box, raycast)
- ✅ Camera system with WASD + mouse look (21 tests)
- ✅ Rendering framework (stub for OpenGL)
- ✅ VoxelUtils with structure generation (28 tests)
- ✅ Surface detection with caching (10x speedup)
- ✅ Spatial hashing for large structures (8x speedup)
- ✅ Voxel clustering and connected components (18 tests)
- ✅ Structural stability analysis system (26 tests)
- ✅ Support chain analysis and ground detection
- ✅ Stability metrics (COM, tip risk, support area)
- ✅ **Structural integrity analyzer with spring system (46 tests)**
- ✅ **Displacement-based failure detection**
- ✅ **Mass calculation via raycasting with caching**
- ✅ **Performance profiling (per-phase timing)**
- ✅ **Cache management and statistics**
- ✅ **Enhanced solver convergence (displacement tracking)**
- ✅ **Displacement clamping (physical correctness)**
- ✅ **Parameter sensitivity testing**
- ✅ **Ground connectivity detection (flood-fill)**
- ✅ **Dual failure criteria (displacement + connectivity)**
- ✅ **Week 3 integration testing suite (8 comprehensive tests)**
- ✅ Demo application with 6 test scenes
- ✅ **240 unit tests - ALL PASSING ✓**

### Planned
- **Week 1-2:** Voxel world foundation (storage, rendering, clustering)
- **Week 3-4:** Track 1 - Structural integrity analysis
- **Week 5-6:** Track 3 - Physics integration (PhysX)
- **Week 7-8:** Integration and polish

## Dependencies

### Required
- C++17 compatible compiler (GCC 7+, Clang 5+, MSVC 2017+)
- CMake 3.15+
- OpenGL 3.3+

### Optional (auto-downloaded if not found)
- GoogleTest (testing framework)
- GLM (math library, fallback available)
- GLFW3 (windowing, needed for rendering in Week 1 Day 3)

## Building

### Linux/macOS

```bash
# Install dependencies (Ubuntu/Debian)
sudo apt-get install build-essential cmake libglfw3-dev libglm-dev

# Build
mkdir build && cd build
cmake ..
make

# Run tests
./bin/VoxelTests

# Run demo (when available)
# ./bin/VoxelDemo
```

### Windows (Visual Studio)

```bash
# Install dependencies via vcpkg
vcpkg install glfw3 glm gtest

# Generate Visual Studio solution
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=[vcpkg root]/scripts/buildsystems/vcpkg.cmake

# Open VoxelDestruction.sln in Visual Studio and build
```

## Testing

```bash
cd build
ctest --output-on-failure

# Or run tests directly
./bin/VoxelTests
```

## Running the Demo

The demo application showcases the voxel system with various test scenarios:

```bash
cd build

# Run with default scene (small building)
./bin/VoxelDemo

# Run specific scenes
./bin/VoxelDemo tower      # Vertical tower (tests vertical loads)
./bin/VoxelDemo wall       # 2D wall (tests basic stability)
./bin/VoxelDemo building   # Hollow building with 4 walls
./bin/VoxelDemo bridge     # Span with support pillars (tests horizontal loads)
./bin/VoxelDemo arch       # Architectural arch (tests compression)
./bin/VoxelDemo overhang   # Cantilevered structure (tests tension)

# Run with performance benchmarks
./bin/VoxelDemo building --benchmark
```

The demo will display:
- Material database with physics properties
- Voxel creation progress
- Spatial query results
- Performance benchmarks (if --benchmark flag used)
- Rendering info (stub implementation in non-OpenGL environments)

## Development Progress

### Day 1 ✅
- [x] Project structure
- [x] CMake build system
- [x] Vector3 implementation with 29 unit tests
- [x] Material system with physics properties
- [x] Material database (Air, Wood, Brick, Concrete, Steel)
- [x] Comprehensive unit tests (47 total)

### Day 2 ✅
- [x] Voxel struct with HP and damage system
- [x] VoxelWorld class with sparse storage
- [x] Grid snapping and position management
- [x] Neighbor finding (6-connected and 26-connected)
- [x] Spatial queries (radius, box, raycast)
- [x] BoundingBox utilities
- [x] 27 new unit tests (74 total)

### Day 3 ✅
- [x] Camera system with full controls
- [x] View/projection matrix generation
- [x] Renderer framework (VoxelRenderer, Window classes)
- [x] Instanced cube rendering architecture
- [x] GLSL shader code (vertex + fragment)
- [x] Demo application with test scenes
- [x] 21 new unit tests (95 total)
- [x] Stub implementation (ready for OpenGL)

### Day 4 (Current) ✅
- [x] Surface voxel detection (IsSurfaceVoxel, FindSurfaceVoxels)
- [x] Structure generation (CreateBox, CreateSphere, CreateCylinder)
- [x] Architectural elements (CreateWall, CreateFloor, CreatePillar)
- [x] Voxel manipulation (FloodFill, ReplaceAll, RemoveDisconnected)
- [x] Analysis tools (GetBounds, GetCenterOfMass, GetTotalMass)
- [x] StructureBuilder with fluent API
- [x] 28 new unit tests (123 total)

### Day 5 (Current) ✅
- [x] Additional test scenarios (bridge, arch, overhang)
- [x] Enhanced scene selector with 6 different structures
- [x] Performance benchmarking suite
- [x] Profiled spatial queries, surface detection, and analysis tools
- [x] Updated demo application with --benchmark flag
- [x] All 123 tests passing

### Week 1 Deliverable ✅
**Completed:**
- ✅ Running demo application with 6 test scenes
- ✅ Camera system (WASD + mouse controls)
- ✅ Voxel rendering framework (stub for OpenGL)
- ✅ Comprehensive test suite (123 tests)
- ✅ Performance benchmarks showing efficient operations

**Performance Metrics:**
- 8.26M voxels/sec insertion rate
- 0.62 μs average radius query
- 0.255 μs average neighbor query
- 90 μs surface detection (180 voxels)

### Week 2: Voxel System Completion

#### Day 6 ✅
- [x] Integrated surface detection into VoxelWorld class
- [x] Implemented surface cache with automatic invalidation
- [x] Cache-aware SetVoxel/RemoveVoxel operations
- [x] GetSurfaceVoxels() method with lazy cache updates
- [x] 10 new unit tests for surface caching (133 total)
- [x] Performance test showing 10x+ speedup from caching

#### Day 7 ✅
- [x] SpatialHash class with grid-based partitioning
- [x] GridCell structure with efficient hashing
- [x] Integrated into VoxelWorld (optional, enable with EnableSpatialHashing())
- [x] Automatic maintenance on SetVoxel/RemoveVoxel/Clear
- [x] Optimized GetVoxelsInRadius and GetVoxelsInBox
- [x] 17 new unit tests (150 total)
- [x] Performance test showing 8x+ speedup for 50K voxel structures

#### Day 8 ✅
- [x] VoxelCluster structure with cluster properties (COM, mass, bounds)
- [x] VoxelClustering class with static analysis methods
- [x] BFS flood-fill algorithm for finding connected components
- [x] FindAllClusters() - identify all disconnected structures
- [x] FindClustersWithGrounding() - detect grounded vs floating clusters
- [x] CalculateClusterProperties() - compute mass, COM, and bounds
- [x] 18 new unit tests covering clustering algorithms (168 total)
- [x] Support for 6-connectivity (face neighbors only)

#### Day 9 ✅
- [x] VoxelStability system for structural analysis
- [x] FindGroundSupportVoxels() - identify voxels touching ground
- [x] FindSupportedVoxels() - BFS from ground to find supported voxels
- [x] ClassifySupport() - classify each voxel as GROUNDED/SUPPORTED/UNSUPPORTED
- [x] AnalyzeStability() - comprehensive stability analysis
- [x] Stability metrics: score (0-1), tip risk (0-1), support area
- [x] IsCenterOfMassSupported() - check if COM over support base
- [x] 26 new unit tests for stability analysis (194 total)

#### Day 10 (Current) ✅
- [x] Fixed all failing tests (4 test case corrections)
- [x] Corrected voxel spacing in stability tests
- [x] Adjusted test expectations for realistic physics
- [x] **Full test suite: 194/194 tests passing (100%)**
- [x] Week 2 polish and finalization complete

### Week 2 Summary ✅
**Voxel System Completion - All objectives achieved:**
- ✅ Surface detection with intelligent caching (10x speedup)
- ✅ Spatial hashing for large-scale structures (8x speedup)
- ✅ Connected component analysis and clustering (BFS flood-fill)
- ✅ Comprehensive stability analysis system
- ✅ Support chain tracking and ground detection
- ✅ 76 new tests added (118 → 194 tests)
- ✅ **100% test pass rate maintained**
- ✅ Foundation ready for Week 3-4 structural analysis

### Week 3: Structural Integrity Analysis (Track 1)

#### Day 11 ✅
- [x] SpringNode data structure (displacement, velocity, mass_supported)
- [x] StructuralAnalyzer class with full pipeline
- [x] BuildNodeGraph() - Creates nodes from surface voxels near damage
- [x] CalculateMassSupported() - Raycasts upward to calculate supported mass
- [x] SolveDisplacements() - Iterative spring solver with damping
- [x] DetectFailures() - Checks displacement vs material limits
- [x] FindFailedClusters() - Groups failed nodes for physics
- [x] 16 new unit tests (210 total)
- [x] **All tests passing (100%)**

#### Day 12 ✅
- [x] Detailed performance profiling (per-phase timing)
- [x] Cache invalidation (InvalidateMassCache, ClearMassCache)
- [x] Cache statistics tracking (hits/misses)
- [x] GetCacheStats() for cache performance monitoring
- [x] ProfilingMetrics in AnalysisResult
- [x] 6 new comprehensive tests (216 total)
- [x] **All tests passing (100%)**

#### Day 13 ✅
- [x] Displacement-based convergence (tracks max change)
- [x] Displacement clamping (prevents upward sag)
- [x] Dual convergence criteria (displacement + velocity)
- [x] Enhanced convergence logging
- [x] Parameter sensitivity testing (timestep, damping)
- [x] Numerical stability checks (NaN/Inf detection)
- [x] 8 new solver tests (224 total)
- [x] **All tests passing (100%)**

#### Day 14 (Current) ✅
- [x] Ground connectivity detection (BFS flood-fill)
- [x] FindFloatingNodes() - Finds disconnected structures
- [x] Dual failure criteria (displacement + connectivity)
- [x] Disconnected node tracking (num_disconnected_nodes)
- [x] Enhanced failure logging (breakdown by type)
- [x] 8 new failure detection tests (232 total)
- [x] **All tests passing (100%)**

## Material Properties

The system includes realistic material properties based on real-world data:

| Material | Density (kg/m³) | Strength (MPa) | Spring Constant | Max Displacement |
|----------|----------------|----------------|-----------------|------------------|
| Wood     | 600            | 10             | 10,000 N/m     | 2.0 cm           |
| Brick    | 1,900          | 30             | 50,000 N/m     | 1.0 cm           |
| Concrete | 2,400          | 40             | 100,000 N/m    | 0.5 cm           |
| Steel    | 7,850          | 250            | 500,000 N/m    | 0.2 cm           |

## Documentation

Comprehensive design documentation available in `/docs`:
- **8_week_sprint_plan.md** - Detailed implementation roadmap
- **track_1_structural_integrity_deep_dive.md** - Structural analysis theory (4,169 lines)
- **track_2_xcom_integration_deep_dive.md** - XCOM-style tactical mechanics (deferred)
- **track_3_physics_integration_deep_dive.md** - Physics simulation guide

## Architecture

### Core Systems

1. **Voxel World** (Week 1-2)
   - Sparse voxel storage
   - Spatial partitioning
   - Surface detection
   - Clustering algorithms

2. **Structural Analyzer** (Week 3-4)
   - Displacement-based spring system
   - Mass calculation via raycasting
   - Failure detection
   - Cluster generation

3. **Physics System** (Week 5-6)
   - PhysX integration
   - Convex hull generation
   - Debris management
   - Turn-based simulation

## Performance Targets

- **Voxel System:** Handle 100K+ voxels at 60 FPS
- **Structural Analysis:** < 500ms per analysis (turn-based)
- **Physics Simulation:** 3-5 seconds per turn (budget)

## Contributing

This is a research prototype following a structured 8-week development plan. See `docs/8_week_sprint_plan.md` for the detailed roadmap.

## License

[To be determined]

## Contact

[Your contact information]
