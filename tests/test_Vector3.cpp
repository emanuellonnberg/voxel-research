#include <gtest/gtest.h>
#include "Vector3.h"
#include <cmath>

// Test fixture for Vector3 tests
class Vector3Test : public ::testing::Test {
protected:
    Vector3 zero;
    Vector3 one;
    Vector3 up;
    Vector3 v1;
    Vector3 v2;

    void SetUp() override {
        zero = Vector3(0, 0, 0);
        one = Vector3(1, 1, 1);
        up = Vector3(0, 1, 0);
        v1 = Vector3(1, 2, 3);
        v2 = Vector3(4, 5, 6);
    }
};

// Constructor tests
TEST_F(Vector3Test, DefaultConstructor) {
    Vector3 v;
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 0.0f);
    EXPECT_FLOAT_EQ(v.z, 0.0f);
}

TEST_F(Vector3Test, ParameterizedConstructor) {
    Vector3 v(1.0f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(v.x, 1.0f);
    EXPECT_FLOAT_EQ(v.y, 2.0f);
    EXPECT_FLOAT_EQ(v.z, 3.0f);
}

TEST_F(Vector3Test, CopyConstructor) {
    Vector3 v(v1);
    EXPECT_FLOAT_EQ(v.x, v1.x);
    EXPECT_FLOAT_EQ(v.y, v1.y);
    EXPECT_FLOAT_EQ(v.z, v1.z);
}

// Arithmetic operator tests
TEST_F(Vector3Test, Addition) {
    Vector3 result = v1 + v2;
    EXPECT_FLOAT_EQ(result.x, 5.0f);
    EXPECT_FLOAT_EQ(result.y, 7.0f);
    EXPECT_FLOAT_EQ(result.z, 9.0f);
}

TEST_F(Vector3Test, Subtraction) {
    Vector3 result = v2 - v1;
    EXPECT_FLOAT_EQ(result.x, 3.0f);
    EXPECT_FLOAT_EQ(result.y, 3.0f);
    EXPECT_FLOAT_EQ(result.z, 3.0f);
}

TEST_F(Vector3Test, ScalarMultiplication) {
    Vector3 result = v1 * 2.0f;
    EXPECT_FLOAT_EQ(result.x, 2.0f);
    EXPECT_FLOAT_EQ(result.y, 4.0f);
    EXPECT_FLOAT_EQ(result.z, 6.0f);
}

TEST_F(Vector3Test, ScalarDivision) {
    Vector3 result = v1 / 2.0f;
    EXPECT_FLOAT_EQ(result.x, 0.5f);
    EXPECT_FLOAT_EQ(result.y, 1.0f);
    EXPECT_FLOAT_EQ(result.z, 1.5f);
}

TEST_F(Vector3Test, Negation) {
    Vector3 result = -v1;
    EXPECT_FLOAT_EQ(result.x, -1.0f);
    EXPECT_FLOAT_EQ(result.y, -2.0f);
    EXPECT_FLOAT_EQ(result.z, -3.0f);
}

// Compound assignment tests
TEST_F(Vector3Test, AdditionAssignment) {
    Vector3 v = v1;
    v += v2;
    EXPECT_FLOAT_EQ(v.x, 5.0f);
    EXPECT_FLOAT_EQ(v.y, 7.0f);
    EXPECT_FLOAT_EQ(v.z, 9.0f);
}

TEST_F(Vector3Test, SubtractionAssignment) {
    Vector3 v = v2;
    v -= v1;
    EXPECT_FLOAT_EQ(v.x, 3.0f);
    EXPECT_FLOAT_EQ(v.y, 3.0f);
    EXPECT_FLOAT_EQ(v.z, 3.0f);
}

TEST_F(Vector3Test, ScalarMultiplicationAssignment) {
    Vector3 v = v1;
    v *= 2.0f;
    EXPECT_FLOAT_EQ(v.x, 2.0f);
    EXPECT_FLOAT_EQ(v.y, 4.0f);
    EXPECT_FLOAT_EQ(v.z, 6.0f);
}

// Comparison tests
TEST_F(Vector3Test, Equality) {
    Vector3 v(1, 2, 3);
    EXPECT_TRUE(v == v1);
    EXPECT_FALSE(v == v2);
}

TEST_F(Vector3Test, Inequality) {
    EXPECT_TRUE(v1 != v2);
    EXPECT_FALSE(v1 != v1);
}

// Vector operation tests
TEST_F(Vector3Test, DotProduct) {
    // (1,2,3) · (4,5,6) = 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
    float dot = v1.Dot(v2);
    EXPECT_FLOAT_EQ(dot, 32.0f);
}

TEST_F(Vector3Test, DotProductOrthogonal) {
    Vector3 x(1, 0, 0);
    Vector3 y(0, 1, 0);
    EXPECT_FLOAT_EQ(x.Dot(y), 0.0f);  // Perpendicular vectors
}

TEST_F(Vector3Test, CrossProduct) {
    Vector3 x(1, 0, 0);
    Vector3 y(0, 1, 0);
    Vector3 result = x.Cross(y);

    // x × y = z
    EXPECT_FLOAT_EQ(result.x, 0.0f);
    EXPECT_FLOAT_EQ(result.y, 0.0f);
    EXPECT_FLOAT_EQ(result.z, 1.0f);
}

TEST_F(Vector3Test, Length) {
    Vector3 v(3, 4, 0);
    // Length = sqrt(3² + 4² + 0²) = sqrt(25) = 5
    EXPECT_FLOAT_EQ(v.Length(), 5.0f);
}

TEST_F(Vector3Test, LengthSquared) {
    Vector3 v(3, 4, 0);
    // Length² = 3² + 4² + 0² = 25
    EXPECT_FLOAT_EQ(v.LengthSquared(), 25.0f);
}

TEST_F(Vector3Test, Normalize) {
    Vector3 v(3, 4, 0);
    Vector3 normalized = v.Normalized();

    EXPECT_NEAR(normalized.Length(), 1.0f, 1e-6f);
    EXPECT_FLOAT_EQ(normalized.x, 0.6f);  // 3/5
    EXPECT_FLOAT_EQ(normalized.y, 0.8f);  // 4/5
    EXPECT_FLOAT_EQ(normalized.z, 0.0f);
}

TEST_F(Vector3Test, NormalizeInPlace) {
    Vector3 v(3, 4, 0);
    v.Normalize();

    EXPECT_NEAR(v.Length(), 1.0f, 1e-6f);
    EXPECT_FLOAT_EQ(v.x, 0.6f);
    EXPECT_FLOAT_EQ(v.y, 0.8f);
}

TEST_F(Vector3Test, NormalizeZeroVector) {
    Vector3 v(0, 0, 0);
    Vector3 normalized = v.Normalized();

    // Zero vector normalization should return zero vector
    EXPECT_FLOAT_EQ(normalized.x, 0.0f);
    EXPECT_FLOAT_EQ(normalized.y, 0.0f);
    EXPECT_FLOAT_EQ(normalized.z, 0.0f);
}

TEST_F(Vector3Test, Distance) {
    Vector3 a(0, 0, 0);
    Vector3 b(3, 4, 0);

    // Distance = sqrt((3-0)² + (4-0)² + (0-0)²) = 5
    EXPECT_FLOAT_EQ(a.Distance(b), 5.0f);
    EXPECT_FLOAT_EQ(b.Distance(a), 5.0f);  // Symmetric
}

TEST_F(Vector3Test, DistanceSquared) {
    Vector3 a(0, 0, 0);
    Vector3 b(3, 4, 0);

    EXPECT_FLOAT_EQ(a.DistanceSquared(b), 25.0f);
}

// Static constructor tests
TEST_F(Vector3Test, StaticConstructors) {
    EXPECT_EQ(Vector3::Zero(), Vector3(0, 0, 0));
    EXPECT_EQ(Vector3::One(), Vector3(1, 1, 1));
    EXPECT_EQ(Vector3::Up(), Vector3(0, 1, 0));
    EXPECT_EQ(Vector3::Down(), Vector3(0, -1, 0));
    EXPECT_EQ(Vector3::Left(), Vector3(-1, 0, 0));
    EXPECT_EQ(Vector3::Right(), Vector3(1, 0, 0));
    EXPECT_EQ(Vector3::Forward(), Vector3(0, 0, 1));
    EXPECT_EQ(Vector3::Back(), Vector3(0, 0, -1));
}

// Utility function tests
TEST_F(Vector3Test, Lerp) {
    Vector3 a(0, 0, 0);
    Vector3 b(10, 10, 10);

    Vector3 mid = Vector3::Lerp(a, b, 0.5f);
    EXPECT_FLOAT_EQ(mid.x, 5.0f);
    EXPECT_FLOAT_EQ(mid.y, 5.0f);
    EXPECT_FLOAT_EQ(mid.z, 5.0f);

    EXPECT_EQ(Vector3::Lerp(a, b, 0.0f), a);
    EXPECT_EQ(Vector3::Lerp(a, b, 1.0f), b);
}

TEST_F(Vector3Test, MinMax) {
    Vector3 a(1, 5, 3);
    Vector3 b(4, 2, 6);

    Vector3 min_result = Vector3::Min(a, b);
    EXPECT_FLOAT_EQ(min_result.x, 1.0f);
    EXPECT_FLOAT_EQ(min_result.y, 2.0f);
    EXPECT_FLOAT_EQ(min_result.z, 3.0f);

    Vector3 max_result = Vector3::Max(a, b);
    EXPECT_FLOAT_EQ(max_result.x, 4.0f);
    EXPECT_FLOAT_EQ(max_result.y, 5.0f);
    EXPECT_FLOAT_EQ(max_result.z, 6.0f);
}

// Hash function test (for use in unordered_map)
TEST_F(Vector3Test, HashFunction) {
    Vector3 v1(0.05f, 0.10f, 0.15f);
    Vector3 v2(0.05f, 0.10f, 0.15f);
    Vector3 v3(0.05f, 0.10f, 0.20f);

    Vector3::Hash hasher;

    // Same vectors should have same hash
    EXPECT_EQ(hasher(v1), hasher(v2));

    // Different vectors should (probably) have different hashes
    EXPECT_NE(hasher(v1), hasher(v3));
}

// Global function tests
TEST_F(Vector3Test, GlobalFunctions) {
    EXPECT_FLOAT_EQ(Dot(v1, v2), v1.Dot(v2));
    EXPECT_EQ(Cross(v1, v2), v1.Cross(v2));
    EXPECT_EQ(Normalize(v1), v1.Normalized());
    EXPECT_FLOAT_EQ(Distance(v1, v2), v1.Distance(v2));
    EXPECT_FLOAT_EQ(DistanceSquared(v1, v2), v1.DistanceSquared(v2));
}

// Practical usage test: Voxel position
TEST_F(Vector3Test, VoxelPositionUsage) {
    const float voxel_size = 0.05f;  // 5cm

    // Create voxel at grid position (10, 20, 30)
    Vector3 voxel_pos(10 * voxel_size, 20 * voxel_size, 30 * voxel_size);

    EXPECT_FLOAT_EQ(voxel_pos.x, 0.5f);
    EXPECT_FLOAT_EQ(voxel_pos.y, 1.0f);
    EXPECT_FLOAT_EQ(voxel_pos.z, 1.5f);

    // Calculate neighbor position (one voxel up)
    Vector3 neighbor_up = voxel_pos + Vector3::Up() * voxel_size;
    EXPECT_FLOAT_EQ(neighbor_up.y, voxel_pos.y + voxel_size);
}
