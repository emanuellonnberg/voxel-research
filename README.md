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

### Current (Week 1, Day 3)
- ✅ Project structure
- ✅ CMake build system
- ✅ Vector3 math library (29 tests)
- ✅ Material system with realistic physical properties (18 tests)
- ✅ Voxel struct with damage system (4 tests)
- ✅ VoxelWorld with sparse storage (17 tests)
- ✅ Spatial queries (radius, box, raycast) (6 tests)
- ✅ Camera system with WASD + mouse look (21 tests)
- ✅ Rendering framework (stub for OpenGL)
- ✅ Demo application with test scenes
- ✅ 95 unit tests passing

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

### Day 5 (Next)
- [ ] Week 1 testing & polish
- [ ] Performance optimization
- [ ] Benchmark suite

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
