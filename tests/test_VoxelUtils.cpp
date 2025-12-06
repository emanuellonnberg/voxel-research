#include <gtest/gtest.h>
#include "VoxelUtils.h"
#include "Material.h"

class VoxelUtilsTest : public ::testing::Test {
protected:
    const float voxel_size = 0.05f;
    VoxelWorld world{voxel_size};  // Initialize directly (VoxelWorld not copyable due to mutexes)

    void SetUp() override {
        // World is already initialized with voxel_size
        world.Clear();  // Clear for each test
    }
};

// Surface detection tests
TEST_F(VoxelUtilsTest, IsSurfaceVoxel_Single) {
    // Single voxel is a surface voxel
    Vector3 pos(0, 0, 0);
    world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));

    EXPECT_TRUE(VoxelUtils::IsSurfaceVoxel(world, pos));
}

TEST_F(VoxelUtilsTest, IsSurfaceVoxel_Interior) {
    // Create a 3x3x3 cube
    for (int x = 0; x < 3; x++) {
        for (int y = 0; y < 3; y++) {
            for (int z = 0; z < 3; z++) {
                world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, z * voxel_size),
                              Voxel(MaterialDatabase::BRICK));
            }
        }
    }

    // Center voxel is interior (not a surface voxel)
    Vector3 center(voxel_size, voxel_size, voxel_size);
    EXPECT_FALSE(VoxelUtils::IsSurfaceVoxel(world, center));
}

TEST_F(VoxelUtilsTest, IsSurfaceVoxel_Edge) {
    // Create a 3x3x3 cube
    for (int x = 0; x < 3; x++) {
        for (int y = 0; y < 3; y++) {
            for (int z = 0; z < 3; z++) {
                world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, z * voxel_size),
                              Voxel(MaterialDatabase::BRICK));
            }
        }
    }

    // Corner voxel is a surface voxel
    Vector3 corner(0, 0, 0);
    EXPECT_TRUE(VoxelUtils::IsSurfaceVoxel(world, corner));
}

TEST_F(VoxelUtilsTest, FindSurfaceVoxels) {
    // Create a 3x3x3 cube (27 voxels total)
    for (int x = 0; x < 3; x++) {
        for (int y = 0; y < 3; y++) {
            for (int z = 0; z < 3; z++) {
                world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, z * voxel_size),
                              Voxel(MaterialDatabase::BRICK));
            }
        }
    }

    auto surface = VoxelUtils::FindSurfaceVoxels(world);

    // 3x3x3 cube has 26 surface voxels (all except center)
    EXPECT_EQ(surface.size(), 26);
}

TEST_F(VoxelUtilsTest, CountExposedFaces) {
    // Single voxel has all 6 faces exposed
    Vector3 pos(0, 0, 0);
    world.SetVoxel(pos, Voxel(MaterialDatabase::BRICK));

    EXPECT_EQ(VoxelUtils::CountExposedFaces(world, pos), 6);
}

TEST_F(VoxelUtilsTest, CountExposedFaces_Partial) {
    // Create two adjacent voxels
    Vector3 pos1(0, 0, 0);
    Vector3 pos2(voxel_size, 0, 0);

    world.SetVoxel(pos1, Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(pos2, Voxel(MaterialDatabase::BRICK));

    // Each voxel now has 5 exposed faces (one is shared)
    EXPECT_EQ(VoxelUtils::CountExposedFaces(world, pos1), 5);
    EXPECT_EQ(VoxelUtils::CountExposedFaces(world, pos2), 5);
}

// Structure generation tests
TEST_F(VoxelUtilsTest, CreateBox_Solid) {
    Vector3 min(0, 0, 0);
    Vector3 max(2 * voxel_size, 2 * voxel_size, 2 * voxel_size);

    VoxelUtils::CreateBox(world, min, max, MaterialDatabase::BRICK, false);

    // 3x3x3 = 27 voxels
    EXPECT_EQ(world.GetVoxelCount(), 27);
}

TEST_F(VoxelUtilsTest, CreateBox_Hollow) {
    Vector3 min(0, 0, 0);
    Vector3 max(2 * voxel_size, 2 * voxel_size, 2 * voxel_size);

    VoxelUtils::CreateBox(world, min, max, MaterialDatabase::BRICK, true);

    // Hollow 3x3x3 = 26 voxels (all except center)
    EXPECT_EQ(world.GetVoxelCount(), 26);
}

TEST_F(VoxelUtilsTest, CreateSphere) {
    Vector3 center(0, 0, 0);
    float radius = 3 * voxel_size;

    VoxelUtils::CreateSphere(world, center, radius, MaterialDatabase::BRICK, false);

    // Should create a roughly spherical shape
    EXPECT_GT(world.GetVoxelCount(), 0);

    // All voxels should be within radius
    auto positions = world.GetAllVoxelPositions();
    for (const auto& pos : positions) {
        float distance = center.Distance(pos);
        EXPECT_LE(distance, radius + voxel_size);  // Within radius + tolerance
    }
}

TEST_F(VoxelUtilsTest, CreateCylinder) {
    Vector3 base(0, 0, 0);
    float radius = 2 * voxel_size;
    float height = 5 * voxel_size;

    VoxelUtils::CreateCylinder(world, base, radius, height, MaterialDatabase::BRICK, false);

    EXPECT_GT(world.GetVoxelCount(), 0);

    // Check bounds
    BoundingBox bounds = VoxelUtils::GetBounds(world);
    EXPECT_GE(bounds.max.y - bounds.min.y, height - voxel_size);
}

TEST_F(VoxelUtilsTest, CreateFloor) {
    Vector3 corner(0, 0, 0);
    float width = 5 * voxel_size;
    float depth = 5 * voxel_size;
    float thickness = voxel_size;

    VoxelUtils::CreateFloor(world, corner, width, depth, thickness,
                           MaterialDatabase::CONCRETE);

    // 6x1x6 = 36 voxels
    EXPECT_EQ(world.GetVoxelCount(), 36);
}

TEST_F(VoxelUtilsTest, CreatePillar) {
    Vector3 base(0, 0, 0);
    float width = voxel_size;
    float depth = voxel_size;
    float height = 10 * voxel_size;

    VoxelUtils::CreatePillar(world, base, width, depth, height,
                            MaterialDatabase::BRICK);

    // 2x11x2 = 44 voxels
    EXPECT_GT(world.GetVoxelCount(), 0);

    // Check it's tall
    BoundingBox bounds = VoxelUtils::GetBounds(world);
    EXPECT_GE(bounds.max.y - bounds.min.y, height - voxel_size);
}

// Manipulation tests
TEST_F(VoxelUtilsTest, FloodFill) {
    // Create a region of wood voxels
    for (int x = 0; x < 3; x++) {
        for (int y = 0; y < 3; y++) {
            world.SetVoxel(Vector3(x * voxel_size, y * voxel_size, 0),
                          Voxel(MaterialDatabase::WOOD));
        }
    }

    // Flood fill from corner
    VoxelUtils::FloodFill(world, Vector3(0, 0, 0), MaterialDatabase::BRICK);

    // All voxels should now be brick
    EXPECT_EQ(VoxelUtils::CountVoxelsByMaterial(world, MaterialDatabase::BRICK), 9);
    EXPECT_EQ(VoxelUtils::CountVoxelsByMaterial(world, MaterialDatabase::WOOD), 0);
}

TEST_F(VoxelUtilsTest, FloodFill_Separated) {
    // Create two separate regions
    world.SetVoxel(Vector3(0, 0, 0), Voxel(MaterialDatabase::WOOD));
    world.SetVoxel(Vector3(5 * voxel_size, 0, 0), Voxel(MaterialDatabase::WOOD));

    // Flood fill from first region
    VoxelUtils::FloodFill(world, Vector3(0, 0, 0), MaterialDatabase::BRICK);

    // Only first region should change
    EXPECT_EQ(VoxelUtils::CountVoxelsByMaterial(world, MaterialDatabase::BRICK), 1);
    EXPECT_EQ(VoxelUtils::CountVoxelsByMaterial(world, MaterialDatabase::WOOD), 1);
}

TEST_F(VoxelUtilsTest, ReplaceAll) {
    // Create mixed materials
    world.SetVoxel(Vector3(0, 0, 0), Voxel(MaterialDatabase::WOOD));
    world.SetVoxel(Vector3(voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(2 * voxel_size, 0, 0), Voxel(MaterialDatabase::WOOD));

    VoxelUtils::ReplaceAll(world, MaterialDatabase::WOOD, MaterialDatabase::CONCRETE);

    EXPECT_EQ(VoxelUtils::CountVoxelsByMaterial(world, MaterialDatabase::CONCRETE), 2);
    EXPECT_EQ(VoxelUtils::CountVoxelsByMaterial(world, MaterialDatabase::WOOD), 0);
    EXPECT_EQ(VoxelUtils::CountVoxelsByMaterial(world, MaterialDatabase::BRICK), 1);
}

TEST_F(VoxelUtilsTest, RemoveDisconnected) {
    // Create connected structure
    world.SetVoxel(Vector3(0, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(2 * voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));

    // Create disconnected voxel
    world.SetVoxel(Vector3(10 * voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));

    EXPECT_EQ(world.GetVoxelCount(), 4);

    // Remove voxels not connected to origin
    VoxelUtils::RemoveDisconnected(world, Vector3(0, 0, 0));

    // Only 3 connected voxels should remain
    EXPECT_EQ(world.GetVoxelCount(), 3);
}

// Analysis tests
TEST_F(VoxelUtilsTest, GetBounds) {
    world.SetVoxel(Vector3(0, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(5 * voxel_size, 10 * voxel_size, 3 * voxel_size),
                  Voxel(MaterialDatabase::BRICK));

    BoundingBox bounds = VoxelUtils::GetBounds(world);

    EXPECT_EQ(bounds.min, Vector3(0, 0, 0));
    EXPECT_EQ(bounds.max, Vector3(5 * voxel_size, 10 * voxel_size, 3 * voxel_size));
}

TEST_F(VoxelUtilsTest, GetBounds_Empty) {
    BoundingBox bounds = VoxelUtils::GetBounds(world);

    EXPECT_EQ(bounds.min, Vector3::Zero());
    EXPECT_EQ(bounds.max, Vector3::Zero());
}

TEST_F(VoxelUtilsTest, GetCenterOfMass) {
    // Create symmetric structure
    world.SetVoxel(Vector3(-voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(0, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));

    Vector3 com = VoxelUtils::GetCenterOfMass(world);

    // Center of mass should be at origin
    EXPECT_NEAR(com.x, 0.0f, voxel_size * 0.1f);
    EXPECT_NEAR(com.y, 0.0f, voxel_size * 0.1f);
    EXPECT_NEAR(com.z, 0.0f, voxel_size * 0.1f);
}

TEST_F(VoxelUtilsTest, GetTotalMass) {
    // Single brick voxel
    world.SetVoxel(Vector3(0, 0, 0), Voxel(MaterialDatabase::BRICK));

    float mass = VoxelUtils::GetTotalMass(world);

    const Material& brick = g_materials.GetMaterial(MaterialDatabase::BRICK);
    float expected_mass = brick.GetVoxelMass(voxel_size);

    EXPECT_FLOAT_EQ(mass, expected_mass);
}

TEST_F(VoxelUtilsTest, GetTotalMass_Multiple) {
    // Two different materials
    world.SetVoxel(Vector3(0, 0, 0), Voxel(MaterialDatabase::WOOD));
    world.SetVoxel(Vector3(voxel_size, 0, 0), Voxel(MaterialDatabase::CONCRETE));

    float mass = VoxelUtils::GetTotalMass(world);

    const Material& wood = g_materials.GetMaterial(MaterialDatabase::WOOD);
    const Material& concrete = g_materials.GetMaterial(MaterialDatabase::CONCRETE);

    float expected_mass = wood.GetVoxelMass(voxel_size) +
                          concrete.GetVoxelMass(voxel_size);

    EXPECT_FLOAT_EQ(mass, expected_mass);
}

TEST_F(VoxelUtilsTest, CountVoxelsByMaterial) {
    world.SetVoxel(Vector3(0, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(voxel_size, 0, 0), Voxel(MaterialDatabase::BRICK));
    world.SetVoxel(Vector3(2 * voxel_size, 0, 0), Voxel(MaterialDatabase::WOOD));

    EXPECT_EQ(VoxelUtils::CountVoxelsByMaterial(world, MaterialDatabase::BRICK), 2);
    EXPECT_EQ(VoxelUtils::CountVoxelsByMaterial(world, MaterialDatabase::WOOD), 1);
    EXPECT_EQ(VoxelUtils::CountVoxelsByMaterial(world, MaterialDatabase::CONCRETE), 0);
}

// StructureBuilder tests
TEST_F(VoxelUtilsTest, StructureBuilder_Basic) {
    StructureBuilder(world)
        .SetMaterial(MaterialDatabase::BRICK)
        .SetPosition(Vector3(0, 0, 0))
        .AddBox(Vector3(2 * voxel_size, 2 * voxel_size, 2 * voxel_size))
        .Build();

    // 3x3x3 = 27 voxels
    EXPECT_EQ(world.GetVoxelCount(), 27);
}

TEST_F(VoxelUtilsTest, StructureBuilder_Floor) {
    StructureBuilder(world)
        .SetMaterial(MaterialDatabase::CONCRETE)
        .SetPosition(Vector3(0, 0, 0))
        .AddFloor(5 * voxel_size, 5 * voxel_size, voxel_size)
        .Build();

    EXPECT_GT(world.GetVoxelCount(), 0);
}

TEST_F(VoxelUtilsTest, StructureBuilder_Pillar) {
    StructureBuilder(world)
        .SetMaterial(MaterialDatabase::BRICK)
        .SetPosition(Vector3(0, 0, 0))
        .AddPillar(voxel_size, voxel_size, 10 * voxel_size)
        .Build();

    EXPECT_GT(world.GetVoxelCount(), 0);

    // Check height
    BoundingBox bounds = VoxelUtils::GetBounds(world);
    EXPECT_GE(bounds.max.y - bounds.min.y, 9 * voxel_size);
}

TEST_F(VoxelUtilsTest, StructureBuilder_Room) {
    StructureBuilder(world)
        .SetMaterial(MaterialDatabase::BRICK)
        .SetPosition(Vector3(0, 0, 0))
        .AddRoom(5 * voxel_size, 5 * voxel_size, 3 * voxel_size)
        .Build();

    // Room should be hollow
    EXPECT_GT(world.GetVoxelCount(), 0);

    // Check that center is empty
    Vector3 center(2.5f * voxel_size, 1.5f * voxel_size, 2.5f * voxel_size);
    EXPECT_FALSE(world.HasVoxel(center));
}

TEST_F(VoxelUtilsTest, StructureBuilder_Multiple) {
    StructureBuilder builder(world);

    builder.SetMaterial(MaterialDatabase::CONCRETE)
           .SetPosition(Vector3(0, 0, 0))
           .AddFloor(10 * voxel_size, 10 * voxel_size, voxel_size);

    builder.SetMaterial(MaterialDatabase::BRICK)
           .SetPosition(Vector3(0, voxel_size, 0))
           .AddPillar(voxel_size, voxel_size, 5 * voxel_size);

    builder.Build();

    // Should have both floor and pillar
    EXPECT_GT(VoxelUtils::CountVoxelsByMaterial(world, MaterialDatabase::CONCRETE), 0);
    EXPECT_GT(VoxelUtils::CountVoxelsByMaterial(world, MaterialDatabase::BRICK), 0);
}

TEST_F(VoxelUtilsTest, StructureBuilder_Clear) {
    StructureBuilder builder(world);

    builder.SetMaterial(MaterialDatabase::BRICK)
           .AddBox(Vector3(2 * voxel_size, 2 * voxel_size, 2 * voxel_size));

    builder.Clear();
    builder.Build();

    // Nothing should be built after clear
    EXPECT_EQ(world.GetVoxelCount(), 0);
}
