#include <gtest/gtest.h>
#include "Material.h"

// Test fixture for Material tests
class MaterialTest : public ::testing::Test {
protected:
    void SetUp() override {
        // g_materials is automatically initialized
    }
};

// Color tests
TEST_F(MaterialTest, ColorConstruction) {
    Color c(0.5f, 0.6f, 0.7f);
    EXPECT_FLOAT_EQ(c.r, 0.5f);
    EXPECT_FLOAT_EQ(c.g, 0.6f);
    EXPECT_FLOAT_EQ(c.b, 0.7f);
}

TEST_F(MaterialTest, NamedColors) {
    Color black = Color::Black();
    EXPECT_FLOAT_EQ(black.r, 0.0f);
    EXPECT_FLOAT_EQ(black.g, 0.0f);
    EXPECT_FLOAT_EQ(black.b, 0.0f);

    Color white = Color::White();
    EXPECT_FLOAT_EQ(white.r, 1.0f);
    EXPECT_FLOAT_EQ(white.g, 1.0f);
    EXPECT_FLOAT_EQ(white.b, 1.0f);
}

TEST_F(MaterialTest, ColorLerp) {
    Color black = Color::Black();
    Color white = Color::White();
    Color gray = Color::Lerp(black, white, 0.5f);

    EXPECT_FLOAT_EQ(gray.r, 0.5f);
    EXPECT_FLOAT_EQ(gray.g, 0.5f);
    EXPECT_FLOAT_EQ(gray.b, 0.5f);
}

// Material construction tests
TEST_F(MaterialTest, DefaultConstructor) {
    Material mat;
    EXPECT_EQ(mat.name, "unknown");
    EXPECT_EQ(mat.id, 0);
    EXPECT_FLOAT_EQ(mat.density, 0.0f);
}

TEST_F(MaterialTest, ParameterizedConstructor) {
    Material mat("test", 1, 1000.0f, 5e6f);
    EXPECT_EQ(mat.name, "test");
    EXPECT_EQ(mat.id, 1);
    EXPECT_FLOAT_EQ(mat.density, 1000.0f);
    EXPECT_FLOAT_EQ(mat.compressive_strength, 5e6f);
}

TEST_F(MaterialTest, VoxelMassCalculation) {
    Material mat("test", 1, 2400.0f, 40e6f);  // Concrete-like

    const float voxel_size = 0.05f;  // 5cm
    float mass = mat.GetVoxelMass(voxel_size);

    // Volume = 0.05³ = 0.000125 m³
    // Mass = 2400 kg/m³ * 0.000125 m³ = 0.3 kg
    EXPECT_NEAR(mass, 0.3f, 0.001f);
}

// MaterialDatabase tests
TEST_F(MaterialTest, DatabaseInitialized) {
    // Database should be auto-initialized with default materials
    EXPECT_GT(g_materials.GetMaterialCount(), 0);
}

TEST_F(MaterialTest, GetMaterialByID) {
    const Material& air = g_materials.GetMaterial(MaterialDatabase::AIR);
    EXPECT_EQ(air.name, "air");
    EXPECT_EQ(air.id, MaterialDatabase::AIR);

    const Material& wood = g_materials.GetMaterial(MaterialDatabase::WOOD);
    EXPECT_EQ(wood.name, "wood");
    EXPECT_EQ(wood.id, MaterialDatabase::WOOD);
}

TEST_F(MaterialTest, GetMaterialByName) {
    const Material& wood = g_materials.GetMaterial("wood");
    EXPECT_EQ(wood.id, MaterialDatabase::WOOD);
    EXPECT_EQ(wood.name, "wood");

    const Material& brick = g_materials.GetMaterial("brick");
    EXPECT_EQ(brick.id, MaterialDatabase::BRICK);
    EXPECT_EQ(brick.name, "brick");

    const Material& concrete = g_materials.GetMaterial("concrete");
    EXPECT_EQ(concrete.id, MaterialDatabase::CONCRETE);
    EXPECT_EQ(concrete.name, "concrete");
}

TEST_F(MaterialTest, GetMaterialID) {
    uint8_t wood_id = g_materials.GetMaterialID("wood");
    EXPECT_EQ(wood_id, MaterialDatabase::WOOD);

    uint8_t brick_id = g_materials.GetMaterialID("brick");
    EXPECT_EQ(brick_id, MaterialDatabase::BRICK);
}

TEST_F(MaterialTest, InvalidMaterialThrows) {
    EXPECT_THROW(g_materials.GetMaterial("nonexistent"), std::runtime_error);
    EXPECT_THROW(g_materials.GetMaterialID("nonexistent"), std::runtime_error);
}

// Material property tests
TEST_F(MaterialTest, WoodProperties) {
    const Material& wood = g_materials.GetMaterial(MaterialDatabase::WOOD);

    EXPECT_FLOAT_EQ(wood.density, 600.0f);
    EXPECT_FLOAT_EQ(wood.compressive_strength, 10e6f);
    EXPECT_GT(wood.spring_constant, 0.0f);
    EXPECT_GT(wood.max_displacement, 0.0f);

    // Wood should be relatively flexible
    EXPECT_GT(wood.max_displacement, 0.01f);
}

TEST_F(MaterialTest, BrickProperties) {
    const Material& brick = g_materials.GetMaterial(MaterialDatabase::BRICK);

    EXPECT_FLOAT_EQ(brick.density, 1900.0f);
    EXPECT_FLOAT_EQ(brick.compressive_strength, 30e6f);

    // Brick should be stronger than wood
    const Material& wood = g_materials.GetMaterial(MaterialDatabase::WOOD);
    EXPECT_GT(brick.compressive_strength, wood.compressive_strength);
    EXPECT_GT(brick.spring_constant, wood.spring_constant);
}

TEST_F(MaterialTest, ConcreteProperties) {
    const Material& concrete = g_materials.GetMaterial(MaterialDatabase::CONCRETE);

    EXPECT_FLOAT_EQ(concrete.density, 2400.0f);
    EXPECT_FLOAT_EQ(concrete.compressive_strength, 40e6f);

    // Concrete should be strongest and stiffest
    const Material& brick = g_materials.GetMaterial(MaterialDatabase::BRICK);
    EXPECT_GT(concrete.compressive_strength, brick.compressive_strength);
    EXPECT_GT(concrete.spring_constant, brick.spring_constant);

    // Concrete should be least flexible (smallest max displacement)
    EXPECT_LT(concrete.max_displacement, brick.max_displacement);
}

TEST_F(MaterialTest, MaterialOrdering) {
    const Material& wood = g_materials.GetMaterial(MaterialDatabase::WOOD);
    const Material& brick = g_materials.GetMaterial(MaterialDatabase::BRICK);
    const Material& concrete = g_materials.GetMaterial(MaterialDatabase::CONCRETE);

    // Density ordering: wood < brick < concrete
    EXPECT_LT(wood.density, brick.density);
    EXPECT_LT(brick.density, concrete.density);

    // Strength ordering: wood < brick < concrete
    EXPECT_LT(wood.compressive_strength, brick.compressive_strength);
    EXPECT_LT(brick.compressive_strength, concrete.compressive_strength);

    // Stiffness ordering: wood < brick < concrete
    EXPECT_LT(wood.spring_constant, brick.spring_constant);
    EXPECT_LT(brick.spring_constant, concrete.spring_constant);
}

// Practical usage test
TEST_F(MaterialTest, VoxelMassComparison) {
    const float voxel_size = 0.05f;  // 5cm

    const Material& wood = g_materials.GetMaterial(MaterialDatabase::WOOD);
    const Material& brick = g_materials.GetMaterial(MaterialDatabase::BRICK);
    const Material& concrete = g_materials.GetMaterial(MaterialDatabase::CONCRETE);

    float wood_mass = wood.GetVoxelMass(voxel_size);
    float brick_mass = brick.GetVoxelMass(voxel_size);
    float concrete_mass = concrete.GetVoxelMass(voxel_size);

    // Mass ordering should match density ordering
    EXPECT_LT(wood_mass, brick_mass);
    EXPECT_LT(brick_mass, concrete_mass);

    // Concrete voxel should be ~4x heavier than wood voxel
    EXPECT_NEAR(concrete_mass / wood_mass, 4.0f, 0.5f);
}

TEST_F(MaterialTest, PhysicsProperties) {
    const Material& concrete = g_materials.GetMaterial(MaterialDatabase::CONCRETE);

    // Friction should be in valid range [0, 1]
    EXPECT_GE(concrete.static_friction, 0.0f);
    EXPECT_LE(concrete.static_friction, 1.0f);
    EXPECT_GE(concrete.dynamic_friction, 0.0f);
    EXPECT_LE(concrete.dynamic_friction, 1.0f);

    // Static friction should be >= dynamic friction
    EXPECT_GE(concrete.static_friction, concrete.dynamic_friction);

    // Restitution (bounciness) should be in valid range [0, 1]
    EXPECT_GE(concrete.restitution, 0.0f);
    EXPECT_LE(concrete.restitution, 1.0f);

    // Concrete shouldn't be very bouncy
    EXPECT_LT(concrete.restitution, 0.3f);
}

TEST_F(MaterialTest, AllDefaultMaterialsPresent) {
    // Ensure all expected default materials exist
    EXPECT_NO_THROW(g_materials.GetMaterial("air"));
    EXPECT_NO_THROW(g_materials.GetMaterial("wood"));
    EXPECT_NO_THROW(g_materials.GetMaterial("brick"));
    EXPECT_NO_THROW(g_materials.GetMaterial("concrete"));
    EXPECT_NO_THROW(g_materials.GetMaterial("steel"));
}
