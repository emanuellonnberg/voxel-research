#include <gtest/gtest.h>
#include <chrono>
#include "VoxelWorld.h"
#include "Material.h"

// Test fixture for VoxelWorld tests
class VoxelWorldTest : public ::testing::Test {
protected:
    const float voxel_size = 0.05f;
    VoxelWorld world{voxel_size};  // Initialize directly (VoxelWorld not copyable due to mutexes)

    void SetUp() override {
        // World is already initialized with voxel_size
        world.Clear();  // Clear for each test
    }
};

// Voxel struct tests
TEST(VoxelTest, DefaultConstructor) {
    Voxel v;
    EXPECT_EQ(v.material_id, 0);
    EXPECT_EQ(v.current_hp, 0);
    EXPECT_FALSE(v.is_active);
    EXPECT_FALSE(v.IsSolid());
}

TEST(VoxelTest, ConstructorWithMaterial) {
    Voxel v(MaterialDatabase::BRICK);
    EXPECT_EQ(v.material_id, MaterialDatabase::BRICK);
    EXPECT_EQ(v.current_hp, 255);
    EXPECT_TRUE(v.is_active);
    EXPECT_TRUE(v.IsSolid());
}

TEST(VoxelTest, HealthPercent) {
    Voxel v(MaterialDatabase::WOOD);
    EXPECT_FLOAT_EQ(v.GetHealthPercent(), 1.0f);

    v.current_hp = 128;
    EXPECT_NEAR(v.GetHealthPercent(), 0.502f, 0.01f);

    v.current_hp = 0;
    EXPECT_FLOAT_EQ(v.GetHealthPercent(), 0.0f);
}

TEST(VoxelTest, TakeDamage) {
    Voxel v(MaterialDatabase::CONCRETE);
    EXPECT_EQ(v.current_hp, 255);

    v.TakeDamage(50);
    EXPECT_EQ(v.current_hp, 205);
    EXPECT_TRUE(v.is_active);

    v.TakeDamage(200);
    EXPECT_EQ(v.current_hp, 5);
    EXPECT_TRUE(v.is_active);

    v.TakeDamage(10);
    EXPECT_EQ(v.current_hp, 0);
    EXPECT_FALSE(v.is_active);  // Destroyed
}

// VoxelWorld basic operations
TEST_F(VoxelWorldTest, InitiallyEmpty) {
    EXPECT_EQ(world.GetVoxelCount(), 0);
}

TEST_F(VoxelWorldTest, SetAndHasVoxel) {
    Vector3 pos(0, 0, 0);
    EXPECT_FALSE(world.HasVoxel(pos));

    Voxel voxel(MaterialDatabase::BRICK);
    world.SetVoxel(pos, voxel);

    EXPECT_TRUE(world.HasVoxel(pos));
    EXPECT_EQ(world.GetVoxelCount(), 1);
}

TEST_F(VoxelWorldTest, GetVoxel) {
    Vector3 pos(0.1f, 0.2f, 0.3f);
    Voxel voxel(MaterialDatabase::CONCRETE);
    world.SetVoxel(pos, voxel);

    Voxel retrieved = world.GetVoxel(pos);
    EXPECT_EQ(retrieved.material_id, MaterialDatabase::CONCRETE);
    EXPECT_EQ(retrieved.current_hp, 255);
    EXPECT_TRUE(retrieved.is_active);
}

TEST_F(VoxelWorldTest, GetNonexistentVoxel) {
    Vector3 pos(100, 100, 100);
    Voxel retrieved = world.GetVoxel(pos);

    // Should return air voxel
    EXPECT_EQ(retrieved.material_id, 0);
    EXPECT_FALSE(retrieved.IsSolid());
}

TEST_F(VoxelWorldTest, RemoveVoxel) {
    Vector3 pos(0, 0, 0);
    world.SetVoxel(pos, Voxel(MaterialDatabase::WOOD));
    EXPECT_TRUE(world.HasVoxel(pos));

    world.RemoveVoxel(pos);
    EXPECT_FALSE(world.HasVoxel(pos));
    EXPECT_EQ(world.GetVoxelCount(), 0);
}

TEST_F(VoxelWorldTest, Clear) {
    // Add several voxels
    world.SetVoxel(Vector3(0, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(1, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(0, 1, 0), Voxel(MaterialDatabase::BRICK));
    EXPECT_EQ(world.GetVoxelCount(), 3);

    world.Clear();
    EXPECT_EQ(world.GetVoxelCount(), 0);
}

// Grid snapping tests
TEST_F(VoxelWorldTest, SnapToGrid) {
    // Voxel size is 0.05m
    Vector3 pos(0.027f, 0.048f, 0.073f);
    Vector3 snapped = world.SnapToGrid(pos);

    // Should snap to nearest grid point
    EXPECT_NEAR(snapped.x, 0.05f, 0.001f);  // Rounds to 0.05
    EXPECT_NEAR(snapped.y, 0.05f, 0.001f);  // Rounds to 0.05
    EXPECT_NEAR(snapped.z, 0.05f, 0.001f);  // Rounds to 0.05
}

TEST_F(VoxelWorldTest, SetVoxelSnapsToGrid) {
    // Set voxel at slightly off-grid position
    Vector3 pos(0.027f, 0.048f, 0.073f);
    world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));

    // Should be retrievable at snapped position
    Vector3 snapped(0.05f, 0.05f, 0.05f);
    EXPECT_TRUE(world.HasVoxel(snapped));
    EXPECT_TRUE(world.HasVoxel(pos));  // Should also work with original
}

// Neighbor tests (6-connected)
TEST_F(VoxelWorldTest, SixConnectedNeighbors) {
    Vector3 center(0, 0, 0);
    world.SetVoxel(center, Voxel(MaterialDatabase::BRICK));

    // Add all 6 face neighbors
    world.SetVoxel(center + Vector3( voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(center + Vector3(-voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(center + Vector3(0,  voxel_size, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(center + Vector3(0, -voxel_size, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(center + Vector3(0, 0,  voxel_size), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(center + Vector3(0, 0, -voxel_size), Voxel(MaterialDatabase::BRICK));

    auto neighbors = world.GetNeighbors(center, Connectivity::SIX);
    EXPECT_EQ(neighbors.size(), 6);

    int count = world.CountNeighbors(center, Connectivity::SIX);
    EXPECT_EQ(count, 6);
}

TEST_F(VoxelWorldTest, PartialNeighbors) {
    Vector3 center(0, 0, 0);
    world.SetVoxel(center, Voxel(MaterialDatabase::BRICK));

    // Add only 3 neighbors
    world.SetVoxel(center + Vector3(voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(center + Vector3(0, voxel_size, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(center + Vector3(0, 0, voxel_size), Voxel(MaterialDatabase::BRICK));

    auto neighbors = world.GetNeighbors(center, Connectivity::SIX);
    EXPECT_EQ(neighbors.size(), 3);
}

TEST_F(VoxelWorldTest, TwentySixConnectedNeighbors) {
    Vector3 center(0, 0, 0);
    world.SetVoxel(center, Voxel(MaterialDatabase::BRICK));

    // Fill entire 3x3x3 cube around center
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            for (int dz = -1; dz <= 1; dz++) {
                if (dx == 0 && dy == 0 && dz == 0) continue;  // Skip center
                Vector3 pos = center + Vector3(dx, dy, dz) * voxel_size;
                world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
            }
        }
    }

    auto neighbors = world.GetNeighbors(center, Connectivity::TWENTY_SIX);
    EXPECT_EQ(neighbors.size(), 26);

    int count = world.CountNeighbors(center, Connectivity::TWENTY_SIX);
    EXPECT_EQ(count, 26);
}

// Spatial query tests
TEST_F(VoxelWorldTest, GetVoxelsInRadius) {
    Vector3 center(0, 0, 0);

    // Create voxels at varying distances
    world.SetVoxel(center, Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(center + Vector3(voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(center + Vector3(voxel_size * 2, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(center + Vector3(voxel_size * 5, 0, 0), Voxel(MaterialDatabase::BRICK));

    // Query small radius
    auto nearby = world.GetVoxelsInRadius(center, voxel_size * 1.5f);
    EXPECT_EQ(nearby.size(), 2);  // Center + 1 neighbor

    // Query larger radius
    auto medium = world.GetVoxelsInRadius(center, voxel_size * 2.5f);
    EXPECT_EQ(medium.size(), 3);  // Center + 2 neighbors

    // Query very large radius
    auto all = world.GetVoxelsInRadius(center, voxel_size * 10.0f);
    EXPECT_EQ(all.size(), 4);  // All voxels
}

TEST_F(VoxelWorldTest, GetVoxelsInBox) {
    // Create a 3x3x3 cube of voxels
    for (int x = 0; x < 3; x++) {
        for (int y = 0; y < 3; y++) {
            for (int z = 0; z < 3; z++) {
                Vector3 pos(x * voxel_size, y * voxel_size, z * voxel_size);
                world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
            }
        }
    }

    EXPECT_EQ(world.GetVoxelCount(), 27);

    // Query exact box
    Vector3 min(0, 0, 0);
    Vector3 max(2 * voxel_size, 2 * voxel_size, 2 * voxel_size);
    auto in_box = world.GetVoxelsInBox(min, max);
    EXPECT_EQ(in_box.size(), 27);

    // Query smaller box
    Vector3 min_small(0, 0, 0);
    Vector3 max_small(voxel_size, voxel_size, voxel_size);
    auto small_box = world.GetVoxelsInBox(min_small, max_small);
    EXPECT_EQ(small_box.size(), 8);  // 2x2x2 cube
}

TEST_F(VoxelWorldTest, Raycast) {
    // Create a line of voxels along X axis
    for (int i = 0; i < 10; i++) {
        world.SetVoxel(Vector3(i * voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));
    }

    // Raycast along X axis
    Vector3 origin(-voxel_size, 0, 0);
    Vector3 direction(1, 0, 0);
    auto hits = world.Raycast(origin, direction, voxel_size * 15);

    EXPECT_GT(hits.size(), 0);
    EXPECT_LE(hits.size(), 10);  // Should hit all or most voxels
}

TEST_F(VoxelWorldTest, RaycastMiss) {
    // Create voxel at origin
    world.SetVoxel(Vector3(0, 0, 0), Voxel(MaterialDatabase::BRICK));

    // Raycast in different direction (should miss)
    Vector3 origin(1, 0, 0);
    Vector3 direction(0, 1, 0);  // Perpendicular
    auto hits = world.Raycast(origin, direction, 10.0f);

    EXPECT_EQ(hits.size(), 0);
}

// Bulk operations
TEST_F(VoxelWorldTest, GetAllVoxelPositions) {
    std::vector<Vector3> positions = {
        Vector3(0, 0, 0),
        Vector3(voxel_size, 0, 0),
        Vector3(0, voxel_size, 0),
        Vector3(0, 0, voxel_size)
    };

    for (const auto& pos : positions) {
        world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
    }

    auto retrieved = world.GetAllVoxelPositions();
    EXPECT_EQ(retrieved.size(), 4);

    // Check all positions are present (order doesn't matter)
    for (const auto& pos : positions) {
        bool found = false;
        for (const auto& ret_pos : retrieved) {
            if ((pos - ret_pos).LengthSquared() < 0.001f) {
                found = true;
                break;
            }
        }
        EXPECT_TRUE(found) << "Position not found: " << pos;
    }
}

// BoundingBox tests
TEST(BoundingBoxTest, Construction) {
    BoundingBox box(Vector3(0, 0, 0), Vector3(1, 1, 1));
    EXPECT_EQ(box.min, Vector3(0, 0, 0));
    EXPECT_EQ(box.max, Vector3(1, 1, 1));
}

TEST(BoundingBoxTest, Center) {
    BoundingBox box(Vector3(-1, -1, -1), Vector3(1, 1, 1));
    Vector3 center = box.GetCenter();
    EXPECT_EQ(center, Vector3(0, 0, 0));
}

TEST(BoundingBoxTest, Size) {
    BoundingBox box(Vector3(0, 0, 0), Vector3(2, 3, 4));
    Vector3 size = box.GetSize();
    EXPECT_EQ(size, Vector3(2, 3, 4));
}

TEST(BoundingBoxTest, Volume) {
    BoundingBox box(Vector3(0, 0, 0), Vector3(2, 3, 4));
    float volume = box.GetVolume();
    EXPECT_FLOAT_EQ(volume, 24.0f);  // 2*3*4
}

TEST(BoundingBoxTest, Contains) {
    BoundingBox box(Vector3(0, 0, 0), Vector3(1, 1, 1));

    EXPECT_TRUE(box.Contains(Vector3(0.5f, 0.5f, 0.5f)));  // Inside
    EXPECT_TRUE(box.Contains(Vector3(0, 0, 0)));           // Corner
    EXPECT_TRUE(box.Contains(Vector3(1, 1, 1)));           // Corner
    EXPECT_FALSE(box.Contains(Vector3(2, 0.5f, 0.5f)));    // Outside
    EXPECT_FALSE(box.Contains(Vector3(-1, 0.5f, 0.5f)));   // Outside
}

TEST(BoundingBoxTest, Intersects) {
    BoundingBox box1(Vector3(0, 0, 0), Vector3(1, 1, 1));
    BoundingBox box2(Vector3(0.5f, 0.5f, 0.5f), Vector3(1.5f, 1.5f, 1.5f));
    BoundingBox box3(Vector3(2, 2, 2), Vector3(3, 3, 3));

    EXPECT_TRUE(box1.Intersects(box2));   // Overlapping
    EXPECT_TRUE(box2.Intersects(box1));   // Symmetric
    EXPECT_FALSE(box1.Intersects(box3));  // Separate
}

// Performance/stress test
TEST_F(VoxelWorldTest, LargeStructure) {
    // Create 10x10x10 cube = 1000 voxels
    for (int x = 0; x < 10; x++) {
        for (int y = 0; y < 10; y++) {
            for (int z = 0; z < 10; z++) {
                Vector3 pos(x * voxel_size, y * voxel_size, z * voxel_size);
                world.SetVoxel(pos, Voxel(MaterialDatabase::CONCRETE));
            }
        }
    }

    EXPECT_EQ(world.GetVoxelCount(), 1000);

    // Test retrieval
    Vector3 test_pos(5 * voxel_size, 5 * voxel_size, 5 * voxel_size);
    EXPECT_TRUE(world.HasVoxel(test_pos));

    // Test neighbor queries (should be fast)
    auto neighbors = world.GetNeighbors(test_pos, Connectivity::SIX);
    EXPECT_EQ(neighbors.size(), 6);  // Interior voxel has all 6 neighbors
}

// Surface detection tests
TEST_F(VoxelWorldTest, IsSurfaceVoxel_Single) {
    // Single voxel is always surface (surrounded by air)
    Vector3 pos(0, 0, 0);
    world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));

    EXPECT_TRUE(world.IsSurfaceVoxel(pos));
}

TEST_F(VoxelWorldTest, IsSurfaceVoxel_Interior) {
    // Create 3x3x3 cube
    for (int x = 0; x < 3; x++) {
        for (int y = 0; y < 3; y++) {
            for (int z = 0; z < 3; z++) {
                Vector3 pos(x * voxel_size, y * voxel_size, z * voxel_size);
                world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
            }
        }
    }

    // Center voxel is interior (completely surrounded)
    Vector3 center(voxel_size, voxel_size, voxel_size);
    EXPECT_FALSE(world.IsSurfaceVoxel(center));
}

TEST_F(VoxelWorldTest, IsSurfaceVoxel_Edge) {
    // Create 3x3x3 cube
    for (int x = 0; x < 3; x++) {
        for (int y = 0; y < 3; y++) {
            for (int z = 0; z < 3; z++) {
                Vector3 pos(x * voxel_size, y * voxel_size, z * voxel_size);
                world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
            }
        }
    }

    // Corner voxel is surface
    Vector3 corner(0, 0, 0);
    EXPECT_TRUE(world.IsSurfaceVoxel(corner));

    // Edge voxel is surface
    Vector3 edge(0, voxel_size, voxel_size);
    EXPECT_TRUE(world.IsSurfaceVoxel(edge));
}

TEST_F(VoxelWorldTest, GetSurfaceVoxels_Single) {
    Vector3 pos(0, 0, 0);
    world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));

    auto& surface = world.GetSurfaceVoxels();
    EXPECT_EQ(surface.size(), 1);
    EXPECT_TRUE(surface.count(pos) > 0);
}

TEST_F(VoxelWorldTest, GetSurfaceVoxels_Cube) {
    // Create 3x3x3 cube = 27 voxels
    for (int x = 0; x < 3; x++) {
        for (int y = 0; y < 3; y++) {
            for (int z = 0; z < 3; z++) {
                Vector3 pos(x * voxel_size, y * voxel_size, z * voxel_size);
                world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));
            }
        }
    }

    auto& surface = world.GetSurfaceVoxels();

    // Only center voxel is interior, rest are surface
    // 27 total - 1 interior = 26 surface
    EXPECT_EQ(surface.size(), 26);

    // Verify center is NOT in surface
    Vector3 center(voxel_size, voxel_size, voxel_size);
    EXPECT_FALSE(surface.count(center) > 0);

    // Verify a corner IS in surface
    Vector3 corner(0, 0, 0);
    EXPECT_TRUE(surface.count(corner) > 0);
}

TEST_F(VoxelWorldTest, SurfaceCache_Caching) {
    // Create structure
    for (int x = 0; x < 5; x++) {
        world.SetVoxel(Vector3(x * voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));
    }

    // First call should populate cache
    auto& surface1 = world.GetSurfaceVoxels();
    size_t count1 = surface1.size();

    // Second call should return same cached result (same address)
    auto& surface2 = world.GetSurfaceVoxels();
    EXPECT_EQ(&surface1, &surface2);  // Same reference
    EXPECT_EQ(count1, surface2.size());
}

TEST_F(VoxelWorldTest, SurfaceCache_Invalidation) {
    // Create initial structure
    world.SetVoxel(Vector3(0, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));

    auto& surface1 = world.GetSurfaceVoxels();
    EXPECT_EQ(surface1.size(), 2);  // Both are surface

    // Add voxel - should invalidate cache
    world.SetVoxel(Vector3(2 * voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));

    auto& surface2 = world.GetSurfaceVoxels();
    EXPECT_EQ(surface2.size(), 3);  // All three are surface
}

TEST_F(VoxelWorldTest, SurfaceCache_InvalidationOnRemove) {
    // Create 3x1x1 line
    world.SetVoxel(Vector3(0, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(2 * voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));

    auto& surface1 = world.GetSurfaceVoxels();
    EXPECT_EQ(surface1.size(), 3);  // All surface

    // Remove middle voxel
    world.RemoveVoxel(Vector3(voxel_size, 0, 0));

    auto& surface2 = world.GetSurfaceVoxels();
    EXPECT_EQ(surface2.size(), 2);  // Two remaining voxels are surface
}

TEST_F(VoxelWorldTest, SurfaceCache_InvalidationOnClear) {
    // Create structure
    for (int i = 0; i < 5; i++) {
        world.SetVoxel(Vector3(i * voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));
    }

    auto& surface1 = world.GetSurfaceVoxels();
    EXPECT_GT(surface1.size(), 0);

    // Clear world
    world.Clear();

    auto& surface2 = world.GetSurfaceVoxels();
    EXPECT_EQ(surface2.size(), 0);  // No voxels = no surface
}

TEST_F(VoxelWorldTest, SurfaceDetection_Performance) {
    // Create larger structure (10x10x10 = 1000 voxels)
    for (int x = 0; x < 10; x++) {
        for (int y = 0; y < 10; y++) {
            for (int z = 0; z < 10; z++) {
                Vector3 pos(x * voxel_size, y * voxel_size, z * voxel_size);
                world.SetVoxel(pos, Voxel(MaterialDatabase::CONCRETE));
            }
        }
    }

    // First call (cache miss) - populate cache
    auto start1 = std::chrono::high_resolution_clock::now();
    auto& surface1 = world.GetSurfaceVoxels();
    auto end1 = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(end1 - start1);

    // Second call (cache hit) - should be much faster
    auto start2 = std::chrono::high_resolution_clock::now();
    auto& surface2 = world.GetSurfaceVoxels();
    auto end2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(end2 - start2);

    // Cache hit should be at least 10x faster
    EXPECT_LT(duration2.count(), duration1.count() / 10);

    // Verify surface count (10x10x10 cube has 488 surface voxels)
    // (10*10*6 faces) - (8*8*6 interior faces) = 600 - 384 = 216? No...
    // Actually: 10^3 - 8^3 = 1000 - 512 = 488
    EXPECT_EQ(surface1.size(), 488);
}
